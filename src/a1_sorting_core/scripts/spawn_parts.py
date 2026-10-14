#!/usr/bin/env python3
"""工件生成节点(ROS1 移植自组长 a1_cell_sim/scripts/spawn_parts.py 的 batch 模式)。

组长版:经 Gazebo 的 /world/<world>/create 服务把 part_1..3(5cm 立方、颜色 red/green/blue/yellow)
生成到传送带槽位,由视觉节点看图登记来料。
ROS1 版:把工件作为 MoveIt 规划场景里的彩色立方体生成到来料槽,并且(默认)直接按
元数据发布 /parts/arrival(PartInfo,category = 颜色),相当于"已知类别"的来料登记;
可选拓展用 a1_vision 的视觉节点识别颜色(此时 publish_arrival:=false)。

接口:
- 发布 /parts/arrival (a1_interfaces/PartInfo)
- 服务 /parts/spawn (a1_interfaces/SpawnParts):清空 part_* 并按颜色序列重新生成(run_trials 用;
  代替组长 run_trials 里直接调用的 Gazebo create/remove)
参数(~):part_colors(red,green,red)、publish_arrival(true)、
  arrival_min_subscribers(2)/arrival_wait(30s):等调度器和记录器都订阅 /parts/arrival 再发(ROS1 无 transient_local)
"""
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from a1_interfaces.msg import PartInfo
from a1_interfaces.srv import SpawnParts, SpawnPartsResponse
from a1_sorting_core.batch_protocol import (BATCH_TOPIC, MANIFEST_TOPIC,
                                            READY_PREFIX, begin_event)
from a1_sorting_core.panda_geometry import parse_colors
from a1_sorting_core.scene_utils import COLOR_RGB, SceneClient, make_box, make_color


class PartSpawner(object):

    def __init__(self):
        gp = rospy.get_param
        self.frame = gp('/a1/frame', 'panda_link0')
        self.size = float(gp('/a1/part_size', 0.05))
        self.prefix = gp('/a1/part_prefix', 'part_')
        self.slots = list(zip(gp('/a1/feed_slots_x'), gp('/a1/feed_slots_y')))
        self.feed_z = float(gp('/a1/feed_z', 0.025))
        self.default_colors = str(gp('~part_colors', 'red,green,red'))
        self.publish_arrival = bool(gp('~publish_arrival', True))
        self.arrival_wait = float(gp('~arrival_wait', 30.0))
        self.min_subs = int(gp('~arrival_min_subscribers', 2))   # 调度器 + 记录器都订阅后再发
        self.attach_link = gp('/pick_place_executor/attach_link', 'panda_hand')

        self.scene = SceneClient()
        self._batch_seq = 0
        self.pub = rospy.Publisher('/parts/arrival', PartInfo, queue_size=10)
        self._manifest = rospy.Publisher(MANIFEST_TOPIC, PartInfo, queue_size=10)
        self._batch_pub = rospy.Publisher(BATCH_TOPIC, String, queue_size=10, latch=True)
        rospy.Service('/parts/spawn', SpawnParts, self.on_spawn)
        ok, msg = self.spawn(self.default_colors)
        rospy.loginfo('初始来料: %s', msg)

    def spawn(self, colors_text):
        colors = parse_colors(colors_text or self.default_colors)
        bad = [c for c in colors if c not in COLOR_RGB]
        if bad or not colors:
            return False, '未知颜色 %s(可用 %s)' % (bad, sorted(COLOR_RGB))
        self._batch_seq += 1
        batch_id = 'batch_%04d' % self._batch_seq
        if not self._wait_nodes(('task_dispatcher', 'result_recorder')):
            return False, '调度器或记录器未就绪,没有生成工件'

        scene = self.scene.get_scene()
        old = list(self.scene.world_positions(self.prefix))
        old += [a.object.id for a in scene.robot_state.attached_collision_objects
                if a.object.id.startswith(self.prefix)]
        if not self.scene.remove_objects(old, self.attach_link):
            return False, '清除旧工件失败,没有发布新来料'

        parts = []
        objs, cols = [], []
        for i, color in enumerate(colors, start=1):
            # 件数超过槽数时按 i%3 循环取槽(与组长 batch 模式一致;同槽会重叠,建议不超过 3 件)
            sx, sy = self.slots[(i - 1) % len(self.slots)]
            pid = '%s%d' % (self.prefix, i)
            objs.append(make_box(pid, self.frame, (self.size,) * 3, (sx, sy, self.feed_z)))
            cols.append(make_color(pid, COLOR_RGB[color]))
            parts.append((pid, color, sx, sy))
        if not self.scene.add_objects(objs, cols):
            return False, '规划场景写入失败,没有发布来料'
        try:
            placed = self.scene.world_positions(self.prefix)
        except rospy.ServiceException as exc:
            return False, '写入后读不回场景: %s' % exc
        missing = [pid for pid, _color, _sx, _sy in parts if pid not in placed]
        if missing:
            return False, '场景中缺少 %s' % ', '.join(missing)

        self._batch_pub.publish(String(data=begin_event(batch_id)))
        rospy.sleep(0.2)
        for pid, color, sx, sy in parts:
            self._manifest.publish(self._part_msg(pid, color, sx, sy, batch_id))
            rospy.sleep(0.05)
        if self.publish_arrival:
            if not self._wait_subscribers(self.pub, self.min_subs):
                return False, '来料订阅者不足,工件已在场景中但没有发布 /parts/arrival'
            rospy.sleep(0.5)
            for pid, color, sx, sy in parts:
                self.pub.publish(self._part_msg(pid, color, sx, sy, batch_id))
                rospy.sleep(0.1)
        return True, '生成 %d 件(%s): %s' % (
            len(parts), batch_id, ', '.join('%s=%s' % (p, c) for p, c, _x, _y in parts))

    def _part_msg(self, part_id, color, sx, sy, batch_id):
        msg = PartInfo()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.part_id = part_id
        msg.batch_id = batch_id
        msg.ground_truth_category = color
        # 清单和默认识别都携带给定类别。视觉节点另发来料时不会抄这个字段。
        msg.category = color
        msg.pose = Pose()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = sx, sy, self.feed_z
        msg.pose.orientation.w = 1.0
        return msg

    def _wait_nodes(self, names):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < self.arrival_wait:
            if all(rospy.get_param(READY_PREFIX + '/' + name, False) for name in names):
                return True
            rospy.sleep(0.2)
        return False

    def _wait_subscribers(self, pub, minimum):
        t0 = rospy.get_time()
        while not rospy.is_shutdown() and rospy.get_time() - t0 < self.arrival_wait:
            if pub.get_num_connections() >= minimum:
                return True
            rospy.sleep(0.2)
        return False

    def on_spawn(self, req):
        ok, msg = self.spawn(req.colors)
        rospy.loginfo('/parts/spawn: %s', msg)
        return SpawnPartsResponse(ok, msg)


if __name__ == '__main__':
    rospy.init_node('spawn_parts')
    PartSpawner()
    rospy.spin()
