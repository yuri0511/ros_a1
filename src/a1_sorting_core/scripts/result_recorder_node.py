#!/usr/bin/env python3
"""结果记录节点:任务终态 -> 外部观测位姿判定 -> CSV 台账 + 数量反馈(ROS1 移植自组长 result_recorder_node.py)。

接口(与组长版相同):
- 订阅 /task/status (TaskStatus):COMPLETED/FAILED 时判定并记录;
- 订阅 /parts/arrival (PartInfo):建立 part_id->category 表(只作台账标注);
- 发布 /result/counts (std_msgs/Int32MultiArray):[bin_A 数, bin_B 数, 失败数, 放错数];
- 发布 /result/markers (visualization_msgs/MarkerArray):料区可视框与计数标签(RViz 用)。

判定逻辑(与组长相同的原则:"以外部观测为准,不信执行节点自说自话"):
组长用 Gazebo 逻辑相机拿工件真值位姿;ROS1 版改为查询 MoveIt 规划场景
(/get_planning_scene)里工件的实际位置,用 bins.judge_position 判在哪个料区:
PLACED(放进目标区)/ MISPLACED(放错区)/ OUTSIDE(不在任何区)/ FAILED / NOT_FOUND。
ROS1 版新增:同一 part_id 再次来料登记(run_trials 下一轮)时开始新一轮的"本轮结束"判定;
另发 /result/summary(String)便于 rostopic echo。
"""
import time

import rospy
from std_msgs.msg import Int32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from a1_interfaces.msg import PartInfo, PartResult, TaskStatus
from a1_sorting_core.batch_protocol import (BATCH_TOPIC, MANIFEST_TOPIC,
                                            PART_RESULT_TOPIC, READY_PREFIX,
                                            parse_event)
from a1_sorting_core.bins import (Bin, RESULT_FAILED, RESULT_MISPLACED,
                                  RESULT_NOT_FOUND, RESULT_OUTSIDE,
                                  RESULT_PLACED, ResultBook, judge_position)
from a1_sorting_core.scene_utils import SceneClient

_MARK_COLORS = {'bin_A': (0.92, 0.45, 0.12, 0.35), 'bin_B': (0.3, 0.85, 0.3, 0.35)}  # 组长口径:A 橙红(暖) B 绿(冷)


class ResultRecorderNode(object):

    def __init__(self):
        gp = rospy.get_param
        self._bins = []
        for name in gp('/a1/bin_names', ['bin_A', 'bin_B']):
            b = gp('/a1/bins/' + name)
            self._bins.append(Bin(name, float(b['x']), float(b['y']),
                                  float(b.get('half_x', 0.1)), float(b.get('half_y', 0.1)),
                                  inner_half_x=b.get('inner_half_x'), inner_half_y=b.get('inner_half_y')))
        csv_path = str(gp('~csv_path', '~/.ros/a1_sorting_results.csv'))
        self._book = ResultBook(csv_path, [b.name for b in self._bins])
        self._expected = list(gp('~expected_parts', ['part_1', 'part_2', 'part_3']))
        self._table_z = float(gp('~table_top_z', 0.0))
        self._frame = str(gp('~markers_frame', gp('/a1/frame', 'panda_link0')))
        self._finished = set()
        self._categories = {}
        self._truth = {}
        self._commanded = {}
        self._started = {}
        self._batch_id = ''
        self._round = 0

        self._scene = SceneClient()
        self._pub_counts = rospy.Publisher('/result/counts', Int32MultiArray, queue_size=10, latch=True)
        self._pub_cumulative = rospy.Publisher('/result/cumulative', Int32MultiArray, queue_size=10, latch=True)
        self._pub_markers = rospy.Publisher('/result/markers', MarkerArray, queue_size=10, latch=True)
        self._pub_summary = rospy.Publisher('/result/summary', String, queue_size=10, latch=True)
        self._pub_part = rospy.Publisher(PART_RESULT_TOPIC, PartResult, queue_size=20)
        rospy.Subscriber('/parts/arrival', PartInfo, self._on_arrival)
        rospy.Subscriber(MANIFEST_TOPIC, PartInfo, self._on_manifest)
        rospy.Subscriber('/task/status', TaskStatus, self._on_status)
        rospy.Subscriber(BATCH_TOPIC, String, self._on_batch)
        rospy.set_param(READY_PREFIX + '/result_recorder', True)
        rospy.on_shutdown(lambda: rospy.set_param(READY_PREFIX + '/result_recorder', False))
        rospy.loginfo('结果记录器就绪,CSV: %s,料区: %s', self._book._path,
                      [(b.name, round(b.center_x, 2), round(b.center_y, 2)) for b in self._bins])
        self._publish_counts()
        self._publish_bin_markers()

    def _bin_for(self, category):
        if not category:
            return ''
        return str(rospy.get_param('/pick_place_executor/bin_for_' + category, ''))

    def _clear_round(self):
        self._finished.clear()
        self._categories.clear()
        self._truth.clear()
        self._commanded.clear()
        self._started.clear()
        self._book.start_round()

    def _on_batch(self, msg):
        kind, batch_id = parse_event(msg.data)
        if kind == 'reset':
            self._clear_round()
            self._publish_counts()
            self._publish_bin_markers()
        elif kind == 'begin':
            self._clear_round()
            self._batch_id = batch_id
            self._round += 1
            rospy.loginfo('== 第 %d 轮开始 %s ==', self._round, batch_id)
            self._publish_counts()
            self._publish_bin_markers()

    def _on_manifest(self, msg):
        if msg.ground_truth_category:
            self._truth[msg.part_id] = msg.ground_truth_category
        if msg.batch_id:
            self._batch_id = msg.batch_id

    def _on_arrival(self, msg):
        if msg.category:
            self._categories[msg.part_id] = msg.category
        if msg.ground_truth_category and msg.part_id not in self._truth:
            self._truth[msg.part_id] = msg.ground_truth_category
        if msg.batch_id:
            self._batch_id = msg.batch_id

    def _on_status(self, msg):
        if msg.state in ('CARRYING', 'PLACING') and msg.detail:
            self._commanded[msg.part_id] = msg.detail
        if msg.state == 'PICKING' and msg.part_id not in self._started:
            self._started[msg.part_id] = time.time()
            return
        if msg.state not in ('COMPLETED', 'FAILED') or msg.part_id in self._finished:
            return
        self._finished.add(msg.part_id)
        truth = self._truth.get(msg.part_id, '')
        predicted = self._categories.get(msg.part_id, '')
        commanded = self._commanded.get(msg.part_id, '')
        if msg.state == 'COMPLETED' and not commanded:
            commanded = msg.detail
        expected = self._bin_for(truth) if truth else commanded
        xy = None
        if msg.state != 'FAILED':
            try:
                pos = self._scene.world_positions(msg.part_id).get(msg.part_id)
                xy = (pos[0], pos[1]) if pos else None
            except rospy.ServiceException as exc:
                rospy.logwarn('查询规划场景失败: %s', exc)
        if msg.state == 'FAILED':
            result, final_bin = RESULT_FAILED, None
        elif xy is None:
            result, final_bin = RESULT_NOT_FOUND, None
        else:
            final_bin = judge_position(xy[0], xy[1], self._bins)
            if final_bin is None:
                result = RESULT_OUTSIDE
            elif expected and final_bin == expected:
                result = RESULT_PLACED
            elif not expected:
                result, final_bin = RESULT_NOT_FOUND, final_bin
            else:
                result = RESULT_MISPLACED
        elapsed = float(msg.elapsed_sec or 0.0)
        if elapsed <= 0.0 and msg.part_id in self._started:
            elapsed = max(0.0, time.time() - self._started[msg.part_id])
        batch_id = msg.batch_id or self._batch_id
        self._book.record(
            msg.part_id, predicted, expected or None, final_bin, result,
            arm=msg.arm or 'arm1', phase='sort', batch_id=batch_id,
            task_id=msg.task_id, ground_truth=truth, predicted=predicted,
            commanded_bin=commanded, attempt=str(msg.attempt or ''),
            elapsed_sec=('%.3f' % elapsed) if elapsed else '')
        rospy.loginfo('%s -> %s 真值=%s 预测=%s 目标=%s 执行目标=%s 实际=%s',
                      msg.part_id, result, truth or '?', predicted or '?',
                      expected or '?', commanded or '?', final_bin)
        part_result = PartResult()
        part_result.batch_id = batch_id
        part_result.task_id = msg.task_id
        part_result.part_id = msg.part_id
        part_result.ground_truth_category = truth
        part_result.predicted_category = predicted
        part_result.commanded_bin = commanded
        part_result.final_bin = final_bin or ''
        part_result.result = result
        part_result.detail = msg.detail
        part_result.elapsed_sec = elapsed
        self._pub_part.publish(part_result)
        self._publish_counts()
        self._publish_bin_markers()
        if self._expected and set(self._expected) <= self._finished:
            c = self._book.round_counts
            rospy.loginfo('== 本轮分拣结束: %s 失败=%d 放错=%d ==',
                          ' '.join('%s=%d' % (b.name, c.get(b.name, 0)) for b in self._bins),
                          c.get('FAILED', 0), c.get('MISPLACED', 0))

    # ------------------------------------------------------------- 输出
    def _count_data(self, counts):
        return [counts.get(b.name, 0) for b in self._bins] + [
            counts.get('FAILED', 0), counts.get('MISPLACED', 0)]

    def _publish_counts(self):
        # /result/counts 是本轮。累计在 /result/cumulative,避免把几轮加在一起显示。
        round_counts = self._book.round_counts
        self._pub_counts.publish(Int32MultiArray(data=self._count_data(round_counts)))
        self._pub_cumulative.publish(Int32MultiArray(data=self._count_data(self._book.counts)))
        self._pub_summary.publish(String(data=' '.join(
            ['round=%d' % self._round]
            + ['%s=%d' % (b.name, round_counts.get(b.name, 0)) for b in self._bins]
            + ['FAILED=%d' % round_counts.get('FAILED', 0),
               'MISPLACED=%d' % round_counts.get('MISPLACED', 0)])))

    def _publish_bin_markers(self):
        array = MarkerArray()
        total = 0
        for idx, b in enumerate(self._bins):
            box = Marker()
            box.header.frame_id = self._frame
            box.ns, box.id, box.type, box.action = 'bins', idx, Marker.CUBE, Marker.ADD
            box.pose.position.x, box.pose.position.y = b.center_x, b.center_y
            box.pose.position.z = self._table_z + 0.003
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = 2 * b.half_x, 2 * b.half_y, 0.005
            box.color.r, box.color.g, box.color.b, box.color.a = _MARK_COLORS.get(b.name, (0.5, 0.5, 0.9, 0.35))
            array.markers.append(box)

            text = Marker()
            text.header.frame_id = self._frame
            text.ns, text.id, text.type, text.action = 'bin_labels', idx, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y = b.center_x, b.center_y
            text.pose.position.z = self._table_z + 0.20
            text.pose.orientation.w = 1.0
            text.scale.z = 0.05
            text.color.r = text.color.g = text.color.b = 0.95
            text.color.a = 1.0
            count = self._book.round_counts.get(b.name, 0)
            total += count
            text.text = '%s: %d' % (b.name, count)
            array.markers.append(text)
        summary = Marker()
        summary.header.frame_id = self._frame
        summary.ns, summary.id, summary.type, summary.action = 'summary', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        summary.pose.position.x, summary.pose.position.z = 0.0, 0.9
        summary.pose.orientation.w = 1.0
        summary.scale.z = 0.06
        summary.color.r = summary.color.g = 1.0
        summary.color.b, summary.color.a = 0.3, 1.0
        c = self._book.round_counts
        summary.text = 'Round %d | placed %d  failed %d  misplaced %d' % (
            self._round, total, c.get('FAILED', 0), c.get('MISPLACED', 0))
        array.markers.append(summary)
        self._pub_markers.publish(array)


if __name__ == '__main__':
    rospy.init_node('result_recorder')
    ResultRecorderNode()
    rospy.spin()
