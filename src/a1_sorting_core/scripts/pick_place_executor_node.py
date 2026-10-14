#!/usr/bin/env python3
"""取放执行节点:接收任务命令,规划执行 抓取 -> 搬运 -> 放置(ROS1 移植自组长 pick_place_executor_node.py)。

运行时依赖:
- move_group(panda_moveit_config demo.launch,fake controllers)负责规划与执行;
  本节点用 moveit_commander.MoveGroupCommander(组长版是自研 MoveGroup 动作客户端 motion_client.py,
  因为 ROS2 Jazzy 没有 Python 版 MoveGroupCommander);
- 夹爪:MoveIt 'hand' 规划组(组长版:ros2_control 夹爪控制器话题);
- 抓取固定:规划场景 attach / detach(组长版:Gazebo 世界服务的"仿真附着")。

接口(与组长版相同):
- 订阅 /task/command (PartInfo):调度器派发的单件任务;
- 发布 /task/status (TaskStatus):IDLE/PICKING/CARRYING/PLACING/COMPLETED/FAILED,
  COMPLETED 的 detail = 目标料区名(recorder 用它判定 PLACED/MISPLACED);
ROS1 版新增:
- 发布 /gripper/state (std_msgs/String, latch):OPEN / CLOSED / HOLDING:<part_id>(对应课程要求"夹爪状态")。

失败处理(可选拓展之一,与组长版相同):
- 每件任务最多尝试 attempts 次;规划失败 / 执行失败 / 附着失败都会触发;
- 重试前做安全恢复(松爪、解附着、回待机位);次数用完回报 FAILED,调度器跳过该件继续下一件;
- 参数 inject_failure_parts 可对指定工件注入一次"首次抓取失败",用于演示失败恢复流程;
- 未知类别 -> FatalGraspError,不重试直接 FAILED(unknown_category)。
"""
import queue
import sys
import threading
import time

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_sorting_core.batch_protocol import BATCH_TOPIC, parse_event
from a1_sorting_core.panda_geometry import (cartesian_reached, down_pose,
                                            grid_slot)
from a1_sorting_core.strategies import strategy_by_id
from a1_sorting_core.scene_utils import SceneClient
from a1_sorting_core.state_machine import (IllegalTransition, TaskState,
                                           TaskStateMachine)


class GraspError(Exception):
    """可重试的执行失败(规划失败/附着失败等)。"""


class FatalGraspError(GraspError):
    """不可重试的失败(如未知类别)。"""


class PickPlaceExecutorNode(object):

    def __init__(self):
        gp = lambda k, d=None: rospy.get_param('~' + k, d)
        self._arm_name = str(gp('arm_name', 'arm1'))
        self._pose_link = gp('pose_link', 'panda_link8')
        self._attach_link = gp('attach_link', 'panda_hand')
        self._standby = gp('standby_named', 'ready')
        self._tcp_z = float(gp('tcp_z_offset', 0.105))
        self._pre_h = float(gp('pre_grasp_height', 0.10))
        self._default_pre_h = self._pre_h
        self._place_h = float(gp('place_height', 0.002))
        self._open_w = float(gp('gripper_open', 0.08))
        self._close_w = float(gp('gripper_close', 0.05))
        self._attempts = max(1, int(gp('attempts', 2)))
        self._inject = [str(x) for x in (gp('inject_failure_parts', ['']) or []) if str(x)]
        self._color_to_bin = {c: str(gp('bin_for_' + c, '')) for c in ('red', 'green', 'yellow', 'blue')}
        self._offx = list(gp('bin_grid_offsets_x', [-0.05, 0.05, -0.05, 0.05]))
        self._offy = list(gp('bin_grid_offsets_y', [-0.035, -0.035, 0.035, 0.035]))
        self._frame = rospy.get_param('/a1/frame', 'panda_link0')
        self._part_size = float(rospy.get_param('/a1/part_size', 0.05))
        self._feed_z = float(rospy.get_param('/a1/feed_z', 0.025))
        self._bins = {n: rospy.get_param('/a1/bins/' + n) for n in rospy.get_param('/a1/bin_names', [])}
        self._place_counts = {}
        self._seq = 0
        self._at_standby = False
        self._terminal_status = None

        # ---- MoveIt(阻塞直到 move_group 就绪)----
        self._robot = moveit_commander.RobotCommander()
        self._arm = moveit_commander.MoveGroupCommander(gp('group_name', 'panda_arm'))
        hand_group = gp('hand_group', 'panda_hand')
        self._hand = moveit_commander.MoveGroupCommander(hand_group)
        self._arm.set_planning_time(float(gp('planning_time', 5.0)))
        self._arm.set_num_planning_attempts(5)
        self._default_vs = float(gp('velocity_scaling', 0.5))
        self._strategy_id = -1
        self._strategy_name = 'default'
        self._arm.set_max_velocity_scaling_factor(self._default_vs)
        self._arm.set_max_acceleration_scaling_factor(self._default_vs)
        self._arm.set_pose_reference_frame(self._frame)
        # 直线规划用规划组默认末端。把它设成和普通规划同一个 link。
        self._arm.set_end_effector_link(self._pose_link)
        self._touch_links = self._robot.get_link_names(group=hand_group)
        self._scene_if = moveit_commander.PlanningSceneInterface()
        self._scene = SceneClient()

        self._pub_status = rospy.Publisher('/task/status', TaskStatus, queue_size=20)
        self._pub_gripper = rospy.Publisher('/gripper/state', String, queue_size=5, latch=True)
        self._jobs = queue.Queue()
        self._attempt = 0
        self._task_t0 = None
        threading.Thread(target=self._worker, daemon=True).start()
        rospy.Subscriber('/task/command', PartInfo, lambda m: self._jobs.put(m))
        rospy.Subscriber(BATCH_TOPIC, String, self._on_batch_event)
        rospy.loginfo('取放执行节点就绪(%s),料区 %s,颜色路由 %s',
                      self._arm_name, sorted(self._bins), self._color_to_bin)

    # ================================================================ 任务主流程
    def _worker(self):
        while not rospy.is_shutdown():
            try:
                part = self._jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._run_task_impl(part)
            except Exception as exc:
                rospy.logerr('执行器未捕获异常: %s', exc)
                if self._terminal_status is None:
                    self._terminal_status = ('task_fault', part, TaskState.FAILED,
                                             'executor_fault:%s' % exc)
            finally:
                # 终态在回到待机位之后统一上报。回不去就改报 standby_failed,调度器不再派下一件。
                if not self._go_standby():
                    task_id, failed_part, _state, _detail = self._terminal_status or (
                        'task_fault', part, TaskState.FAILED, '')
                    self._terminal_status = (task_id, failed_part, TaskState.FAILED,
                                             'standby_failed')
                self._flush_terminal_status()

    def _run_task_impl(self, part):
        self._seq += 1
        task_id = 'task_%04d' % self._seq
        sm = TaskStateMachine()
        self._task_t0 = time.time()
        self._apply_motion_strategy(part.part_id)
        rospy.loginfo('== %s 开始: %s (类别 %s, 策略 %s) ==',
                      task_id, part.part_id, part.category, self._strategy_name)
        if not self._at_standby:
            self._go_standby()
        for attempt in range(1, self._attempts + 1):
            self._attempt = attempt
            self._task_t0 = time.time()
            if attempt > 1:
                sm.reset()   # 重试 = 新一轮状态周期(IDLE 重新出发)
            try:
                target = self._bin_for(part.category)  # FatalGraspError 直接失败
                if attempt == 1 and part.part_id in self._inject:
                    rospy.logwarn('%s: 注入一次抓取失败(演示失败恢复)', part.part_id)
                    rospy.sleep(0.5)
                    raise GraspError('injected_failure')

                # ---------------- PICKING ----------------
                self._set_state(sm, task_id, part, TaskState.PICKING, '')
                self._at_standby = False
                center = self._pick_center(part)
                self._grip(close=False)
                self._move(down_pose(center, self._tcp_z, self._pre_h), 'pre_grasp')
                self._line(down_pose(center, self._tcp_z, 0.0), 'descend_pick')
                self._attach(part.part_id)      # 先附着再闭爪:手指与工件接触被允许
                self._grip(close=True, part_id=part.part_id)
                self._line(down_pose(center, self._tcp_z, self._pre_h), 'lift')

                # ---------------- CARRYING ----------------
                self._set_state(sm, task_id, part, TaskState.CARRYING, target)
                n = self._place_counts.get(target, 0)
                b = self._bins[target]
                sx, sy = grid_slot((b['x'], b['y']), n, self._offx, self._offy)
                place_c = (sx, sy, self._part_size / 2.0 + self._place_h)
                self._move(down_pose(place_c, self._tcp_z, self._pre_h), 'carry')

                # ---------------- PLACING ----------------
                self._set_state(sm, task_id, part, TaskState.PLACING, target)
                self._line(down_pose(place_c, self._tcp_z, 0.0), 'descend_place')
                self._grip(close=False)
                self._detach(part.part_id)
                self._line(down_pose(place_c, self._tcp_z, self._pre_h), 'retreat')

                self._place_counts[target] = n + 1
                self._set_state(sm, task_id, part, TaskState.COMPLETED, target)
                return
            except FatalGraspError as e:
                self._recover(part.part_id)
                rospy.logerr('%s 失败(不重试): %s', task_id, e)
                self._set_state(sm, task_id, part, TaskState.FAILED, str(e))
                return
            except GraspError as e:
                self._recover(part.part_id)
                if attempt >= self._attempts:
                    rospy.logerr('%s 重试耗尽: %s', task_id, e)
                    self._set_state(sm, task_id, part, TaskState.FAILED, str(e))
                    return
                rospy.logwarn('%s 第 %d 次尝试失败(%s),恢复后重试', task_id, attempt, e)
            except IllegalTransition as e:
                rospy.logerr('%s 状态机异常: %s', task_id, e)
                self._terminal_status = (task_id, part, TaskState.FAILED, 'illegal_transition')
                return

    def _on_batch_event(self, msg):
        kind, _batch_id = parse_event(msg.data)
        if kind in ('reset', 'begin'):
            self._place_counts = {}
            rospy.loginfo('格位计数已按新批次清零')

    def _bin_for(self, category):
        target = self._color_to_bin.get(category, '')
        if not target or target not in self._bins:
            raise FatalGraspError('unknown_category(%s)' % category)
        return target

    def _pick_center(self, part):
        """取件中心:以规划场景中工件的当前位置为准(相当于组长 pose_from_camera=true 用真值);
        查不到时退回 PartInfo.pose。重试时工件可能已被移动,所以每次都重新查。"""
        try:
            pos = self._scene.world_positions(part.part_id).get(part.part_id)
        except rospy.ServiceException:
            pos = None
        if pos is None:
            p = part.pose.position
            pos = (p.x, p.y, p.z if p.z > 0 else self._feed_z)
        return pos

    # ================================================================ 状态上报
    def _set_state(self, sm, task_id, part, state, detail):
        sm.advance(state)
        if state in (TaskState.COMPLETED, TaskState.FAILED):
            self._terminal_status = (task_id, part, state, detail)
            rospy.loginfo('%s %s: %s %s(待收尾后上报)', task_id, part.part_id, state.value, detail)
            return
        self._publish_status(task_id, part, state, detail)

    def _flush_terminal_status(self):
        if self._terminal_status is None:
            return
        task_id, part, state, detail = self._terminal_status
        self._terminal_status = None
        self._publish_status(task_id, part, state, detail)

    def _publish_status(self, task_id, part, state, detail):
        msg = TaskStatus()
        msg.header.stamp = rospy.Time.now()
        msg.task_id = task_id
        msg.part_id = part.part_id
        msg.state = state.value
        msg.detail = detail
        msg.arm = self._arm_name
        msg.batch_id = getattr(part, 'batch_id', '') or ''
        msg.attempt = int(self._attempt or 0)
        msg.elapsed_sec = 0.0 if self._task_t0 is None else max(0.0, time.time() - self._task_t0)
        self._pub_status.publish(msg)
        rospy.loginfo('%s %s: %s %s', task_id, part.part_id, state.value, detail)

    def _apply_motion_strategy(self, part_id):
        """每件开始前读策略。没有指定时保持启动时的速度和预抓高度。"""
        sid = rospy.get_param('/a1/strategy_for/' + part_id, None)
        if sid is None:
            sid = rospy.get_param('/a1/active_strategy', -1)
        spec = strategy_by_id(sid)
        if spec is None:
            self._pre_h = self._default_pre_h
            velocity = self._default_vs
            self._strategy_id = -1
            self._strategy_name = 'default'
        else:
            self._pre_h = spec.pre_grasp_height
            velocity = spec.velocity_scaling
            self._strategy_id = spec.id
            self._strategy_name = spec.name
        self._arm.set_max_velocity_scaling_factor(velocity)
        self._arm.set_max_acceleration_scaling_factor(velocity)
        rospy.loginfo('策略 %s id=%s 速度比例 %.2f 预抓高度 %.3f',
                      self._strategy_name, self._strategy_id, velocity, self._pre_h)

    # ================================================================ 运动接口
    def _pose(self, target):
        (x, y, z), q = target
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p

    def _move(self, target, what):
        self._arm.set_pose_target(self._pose(target), self._pose_link)
        ok = self._arm.go(wait=True)
        self._arm.stop()
        self._arm.clear_pose_targets()
        if not ok:
            raise GraspError('plan_failed(%s)' % what)

    def _line(self, target, what, min_fraction=0.999):
        """笛卡尔直线段。本机 MoveIt 1.1.16 的第三个参数是 avoid_collisions。

        fraction 不到 0.999 说明没走到目标,不执行这段残缺轨迹。
        """
        try:
            plan, fraction = self._arm.compute_cartesian_path(
                [self._pose(target)], 0.01, avoid_collisions=True)
        except Exception as exc:
            rospy.logwarn('%s 直线规划不可用(%s),改用普通规划', what, exc)
            return self._move(target, what)
        if not cartesian_reached(fraction, min_fraction):
            rospy.logwarn('%s 直线规划未走到目标(fraction=%s),改用普通规划', what, fraction)
            return self._move(target, what)
        if not self._arm.execute(plan, wait=True):
            raise GraspError('exec_failed(%s)' % what)

    def _go_standby(self):
        try:
            self._arm.set_named_target(self._standby)
            ok = bool(self._arm.go(wait=True))
            self._arm.stop()
            self._at_standby = ok
            if not ok:
                rospy.logwarn('待机位运动失败')
            return ok
        except Exception as exc:
            self._at_standby = False
            rospy.logwarn('待机位运动异常: %s', exc)
            return False

    def _grip(self, close, part_id=''):
        width = self._close_w if close else self._open_w
        joints = self._hand.get_active_joints()
        self._hand.set_joint_value_target({j: width / 2.0 for j in joints})
        ok = self._hand.go(wait=True)
        self._hand.stop()
        if not ok:
            raise GraspError('gripper_failed(%s)' % ('close' if close else 'open'))
        state = ('HOLDING:%s' % part_id) if (close and part_id) else ('CLOSED' if close else 'OPEN')
        self._pub_gripper.publish(String(data=state))

    def _attach(self, part_id):
        self._scene_if.attach_box(self._attach_link, part_id, touch_links=self._touch_links)
        self._wait_attached(part_id, True)

    def _detach(self, part_id):
        self._scene_if.remove_attached_object(self._attach_link, name=part_id)
        self._wait_attached(part_id, False)

    def _wait_attached(self, part_id, attached, timeout=3.0):
        t0 = rospy.get_time()
        while rospy.get_time() - t0 < timeout and not rospy.is_shutdown():
            if (part_id in self._scene_if.get_attached_objects([part_id])) == attached:
                return
            rospy.sleep(0.1)
        raise GraspError('attach_failed' if attached else 'detach_failed')

    def _recover(self, part_id):
        """重试前安全恢复(与组长版相同思路):停止、松爪、解附着。
        解附着后工件留在当前位置(规划场景里会悬空),下一次尝试从该位置重新抓取。"""
        try:
            self._arm.stop()
            if part_id in self._scene_if.get_attached_objects([part_id]):
                self._grip(close=False)
                self._detach(part_id)
            else:
                self._grip(close=False)
        except GraspError as e:
            rospy.logwarn('恢复动作失败: %s', e)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_place_executor')
    PickPlaceExecutorNode()
    rospy.spin()
    moveit_commander.roscpp_shutdown()
