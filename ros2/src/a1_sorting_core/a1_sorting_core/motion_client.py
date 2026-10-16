"""MoveGroup 动作客户端:向 move_group 发送位姿目标,规划并执行。

自研运动接口(课程自研内容之一):
- 走标准 moveit_msgs/action/MoveGroup 动作,plan_only=False,
  由 move_group 完成规划并经其控制器管理器执行(scaled JTC);
- 目标约束用 tool0 的位置球约束(半径 1cm)+ 姿态约束;
- 起始状态留空 = move_group 侧取当前状态。

线程模型:动作调用由工作线程发起,主线程的 rclpy.spin 负责回调,
工作线程轮询 future 完成状态(避免跨线程 spin)。
"""
import time
import threading

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PositionConstraint,
                             RobotState, WorkspaceParameters)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MotionClient:
    """串行的规划-执行客户端(同一时刻一个目标)。"""

    def __init__(self, node, group_name: str, tool_frame: str,
                 planning_frame: str, server_timeout: float = 20.0,
                 vel_scale: float = 0.5, acc_scale: float = 0.5,
                 wedge_detect: bool = False,
                 tool_settle_fix: bool = False,
                 split_plan_execute: bool = False,
                 controller_action: str = '', fast_sim_waits: bool = False):
        self._node = node
        self._fast_waits = bool(fast_sim_waits)
        self._wait_future = None
        self._wait_event = None
        self._slot_event = threading.Event()
        self._group = group_name
        self._tool = tool_frame
        self._frame = planning_frame
        self._vel = vel_scale
        self._acc = acc_scale
        self._ik_retries = 3   # NO_IK 时目标微扰重试次数
        self._tool_settle_fix = bool(tool_settle_fix)   # G-B 单独开关
        # ---- R1-P1/P3 楔死检测(2026-10-04 终审实装,默认关=零行为差) ----
        # 背景(forensics + D1 实测 3×6):settle 类 goal 被 JTC "Accepted"
        # 后永不 "Goal reached"(τ 平台失速),执行器按 no-cancel 纪律自
        # 完成放行,但挂起 goal 仍占着 move_group 执行槽 ⇒ 之后每个目标
        # "Cannot push a new trajectory while another"→CONTROL_FAILED(-4)
        # 直到会话结束;单件失败被放大成整轮级联(上轮 E2 "6/6 FAILED"
        # 即此失真)。判据:本客户端出现过"早退/超时未回传"(dangling)后,
        # 连续 ≥3 次 -4 ⇒ session_dead(此后不再发目标,让上层判死/重启)。
        self._wedge_detect = bool(wedge_detect)
        self._dangling = False
        self._consec_m4 = 0
        self.session_dead = False
        self._client = ActionClient(node, MoveGroup, '/move_action')
        if not self._client.wait_for_server(timeout_sec=server_timeout):
            raise RuntimeError('move_group 动作服务器不可用')
        # 规划仍走共享的 /move_action,而且发送前照旧等它的执行槽空出来。
        # 执行改走本臂自己的关节轨迹控制器,两台臂才能同时动。
        # 不把那次等待删掉:两份规划仍要排队,悬置的规划目标也还能被看见。
        self._split = bool(split_plan_execute)
        self._traj_client = None
        self._own_goal_inflight=False
        self._own_controller_active=0
        self.last_controller_result=None
        if self._split:
            from control_msgs.action import FollowJointTrajectory
            action_name = controller_action or (
                '/arm2_joint_trajectory_controller/follow_joint_trajectory'
                if 'arm2' in group_name else
                '/joint_trajectory_controller/follow_joint_trajectory')
            self._traj_client = ActionClient(
                node, FollowJointTrajectory, action_name)
            if not self._traj_client.wait_for_server(timeout_sec=server_timeout):
                raise RuntimeError(f'控制器动作服务器不可用: {action_name}')
            node.get_logger().info(
                f'运动拆开: {group_name} 规划走 /move_action,'
                f'执行走 {action_name}')
        if self._split:
            from action_msgs.msg import GoalStatusArray
            from rclpy.qos import qos_profile_action_status_default
            def own_status(msg):
                self._own_controller_active=sum(x.status in (1,2,3) for x in msg.status_list)
            self._own_status_sub=node.create_subscription(GoalStatusArray,
                action_name+'/_action/status',own_status,qos_profile_action_status_default)

        # 执行槽观测(问题修复 2026-10-06):move_group 全局单执行槽,
        # 悬置 goal(τ 平台失速/物理到位早退)未自然收尾时发新目标必
        # Cannot push(-4),快速重试可触发 move_group 2.12 段错误,
        # 且**两臂连坐**(r3 实录:arm2 settle 梯级悬置 ⇒ arm1 两件
        # pick_anchor -4 判死)。订阅 /move_action/_action/status 计
        # 活跃 goal 数,发送前等清零(上限 wait_s;超时仍发,行为同旧)。
        from action_msgs.msg import GoalStatusArray
        self._active = [0]

        def _st_cb(m):
            # GoalStatus: UNKNOWN=0 ACCEPTED=1 EXECUTING=2 CANCELING=3
            # (SUCCEEDED=4/CANCELED=5/ABORTED=6 均为终态)
            self._active[0] = sum(
                1 for g in m.status_list if g.status in (1, 2, 3))
            if self._active[0] == 0:self._slot_event.set()
            else:self._slot_event.clear()
        self._status_sub = None
        if hasattr(node, 'create_subscription'):
            try:
                # action status 发布方 QoS 是 BEST_EFFORT(reliable 订阅
                # 与之不兼容⇒零帧,r4 实证);须显式 best_effort
                from rclpy.qos import (QoSProfile,
                                       ReliabilityPolicy)
                self._status_sub = node.create_subscription(
                    GoalStatusArray, '/move_action/_action/status',
                    _st_cb, QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.BEST_EFFORT))
            except Exception:  # noqa: BLE001 降级:无 status 订阅
                pass

    def _future_pause(self, future, seconds):
        if not getattr(self, '_fast_waits', False):
            time.sleep(seconds)
            return
        if self._wait_future is not future:
            self._wait_future = future
            self._wait_event = threading.Event()
            event = self._wait_event
            future.add_done_callback(lambda unused: event.set())
        self._wait_event.wait(seconds)

    # ---------------------------------------------------------------- 楔死
    def _wait_slot_free(self, wait_s: float = 45.0, label: str = '') -> None:
        """发送新目标前等 move_group 执行槽自然释放(见 __init__ 注释)。

        窗长 45s:r6 实测失速 settle 梯级腿(深折带载 P 下垂 > goal 容差
        lift 0.12 ⇒ open_loop JTC 永不 succeeded)占槽 44s;20s 窗不够,
        超时后仍发 ⇒ Cannot push ×3 ⇒ 判死 ⇒ **两臂连坐**(r3 连坐 2 件、
        r5 连坐 2 件)。45s 窗把单件失速的代价限制在本腿,不再放大成
        整轮雪崩。活跃 goal 清零即返回;超时仍发(由 -4 退避链兜底)。
        rclpy.spin 在主线程,回调只读写一个 int,本方法在任务线程轮询
        缓存值,无锁亦安全。"""
        t_end = time.monotonic() + wait_s
        while self._active[0] and time.monotonic() < t_end:
            if getattr(self, '_fast_waits', False):self._slot_event.wait(.1)
            else:time.sleep(.1)
        if self._active[0]:
            self._node.get_logger().warn(
                f'{label}: 执行槽 {wait_s:.0f}s 未释放'
                f'(活跃 goal={self._active[0]}),仍发送(由 -4 退避链兜底)')

    def _drain_result(self, result_future, wait_s: float = 15.0) -> None:
        """G3 竞态修复(2026-10-05):settle 早退还成功时,服务器结果往往
        只差 <0.5s 就回传(g3_r1 实测:386.17 早退、386.34 JTC Goal
        reached),而下一腿 goal 在 7ms 后发出 ⇒ move_group 执行槽仍被
        占 ⇒ "Cannot push a new trajectory" + 快速重试撞上 move_group
        2.12 的取消/冲突竞态段错误(整会话 受理超时 级联死)。
        早退前给结果回传一个排空窗:done 即返回(槽自然释放)。
        窗长 15s:g4_r1b 实证长 carry 腿(open_loop 按时间参数化,JTC 要
        走完命令时长才回 result)物理到位后仍有 ~10s+ 才 Goal reached,
        2s 窗不够 ⇒ 悬置 ⇒ -4 洪水 ⇒ wedge_detect 判死(单腿误伤整轮)。
        真·τ 平台失速(永不回传)由 -4×3 判死链兜住,只是多等 15s。
        与 no-cancel 纪律兼容——不发 cancel。"""
        t_end = time.monotonic() + wait_s
        while not result_future.done() and time.monotonic() < t_end:
            self._future_pause(result_future, 0.05)

    def _note_outcome(self, ok: bool, code: int = 0,
                      dangling: bool = False) -> None:
        """单点记账:dangling=本目标结果未回传但被放行(槽可能仍被占)。

        判据链(全部实测取证 /tmp/exp/d1_z108*_r*.log):
        ①"早退还成功"(dangling)→ 挂起 goal 占住 move_group 执行槽;
        ②随后**同一客户端**连续 ≥3 次 code=-4("Cannot push a new
        trajectory while another")⇒ 槽永久占用 = 会话中毒;
        ③任意一次**完整正常成功**(非早退)说明槽已释放 ⇒ 清 dangling。
        关闭 wedge_detect 时纯记账不改行为(session_dead 恒 False)。
        """
        if not self._wedge_detect:
            return
        if dangling:
            # 悬置放行:槽可能被占;-4 连击只统计悬置**之后**的
            # (悬置前的 -4 多为双臂瞬态互挤,与楔死无关)
            self._dangling = True
            self._consec_m4 = 0
        elif ok:                           # 完整正常成功:槽确已释放
            self._dangling = False
            self._consec_m4 = 0
            return
        elif code == -4:
            self._consec_m4 += 1
        else:
            self._consec_m4 = 0            # 其他失败(如 -31)不计楔死
        if (not self.session_dead and self._dangling
                and self._consec_m4 >= 3):
            self.session_dead = True
            self._node.get_logger().error(
                'MOTION_SESSION_WEDGED:悬置 goal 后连续 '
                f'{self._consec_m4} 次 CONTROL_FAILED(-4),执行槽已楔死;'
                '停止发送目标(上层判死 FAILED + 受控重启 move_group)')

    def _refuse_if_dead(self, label: str) -> bool:
        """session_dead 后拒发新目标;返回 True=已拒发。"""
        if self._wedge_detect and self.session_dead:
            self._node.get_logger().error(
                f'{label}: 会话已判死(motion_session_wedged),拒发新目标')
            return True
        return False

    # ---------------------------------------------------------------- 目标
    def _build_goal(self, pose: PoseStamped, attempts: int = 1,
                    planning_time: float = 5.0, linear: bool = False,
                    plan_only: bool = False) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self._group
        req.num_planning_attempts = attempts
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = self._vel
        req.max_acceleration_scaling_factor = self._acc
        # Pilz 工业规划器(PTP 关节空间点对点 / LIN 笛卡尔直线):
        # 确定性、无随机失败、轨迹短直。OMPL(RRTConnect) 在本工作站的
        # 严苛目标(小公差+折叠构型)下频繁规划失败或给出绕远解,弃用。
        # Pilz 不做避障规划;本场景障碍已由 TCP 高度约束与 SRDF 禁用排除。
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'LIN' if linear else 'PTP'
        req.start_state = RobotState()  # 空 = 当前状态

        ws = WorkspaceParameters()
        ws.header.frame_id = self._frame
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.5
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 1.5
        req.workspace_parameters = ws

        position = PositionConstraint()
        position.header.frame_id = self._frame
        position.link_name = self._tool
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        # 位置公差 2cm:6DOF 精确约束下过严的公差(如 5mm)会让 OMPL
        # 频繁失败;夹爪开口 132mm,2cm 横向偏差完全可接受
        sphere.dimensions = [0.02]
        position.constraint_region.primitives = [sphere]
        region_pose = pose.pose
        position.constraint_region.primitive_poses = [region_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = self._frame
        orientation.link_name = self._tool
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        req.goal_constraints = [constraints]

        # 拆开执行时这里只规划。轨迹交给本臂控制器,不进 move_group 的执行槽。
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    # ---------------------------------------------------------------- 调用
    def move_to_pose(self, pose: PoseStamped, label: str = '',
                     timeout: float = 60.0, linear: bool = False) -> bool:
        """规划到指定位姿并执行,阻塞直至完成;成功返回 True。

        linear=True 用 LIN(笛卡尔直线,下探/提升等短距竖直运动),
        否则用 PTP(关节空间点对点,大转移)。

        NO_IK_SOLUTION(code=-31)时对目标 xy 加 ±1.5cm 随机微扰重试
        (最多 ik_perturb_retries 次):KDL 数值 IK 从当前关节种子迭代,
        在个别分支上会陷入局部不可达,小扰动即可跳出(夹爪开口 132mm,
        1.5cm 目标偏移不影响抓放)。

        CONTROL_FAILED(code=-4)时退避 2 秒重试(最多 3 次):双臂共用
        move_group,另一臂轨迹尚在执行时本目标会被"Cannot push a new
        trajectory while another is being executed"整体拒绝,等待对方
        收尾即可恢复。
        """
        import random
        if self._refuse_if_dead(label):
            return False
        busy_retries = 0
        attempt = 0
        while True:
            target = pose
            if attempt > 0:
                target = PoseStamped()
                target.header = pose.header
                target.pose = pose.pose
                target.pose.position.x += random.uniform(-0.015, 0.015)
                target.pose.position.y += random.uniform(-0.015, 0.015)
                self._node.get_logger().info(
                    f'{label}: NO_IK,第 {attempt} 次微扰重试')
            ok, code = self._send_goal(target, label, timeout, linear)
            if ok:
                return True
            if code == -4 and busy_retries < 3:
                busy_retries += 1
                self._node.get_logger().warn(
                    f'{label}: 控制器忙(CONTROL_FAILED),退避 2s 重试'
                    f'({busy_retries}/3)')
                if self._wedge_detect and self.session_dead:
                    self._node.get_logger().error(
                        f'{label}: 会话判死,停止退避重试')
                    return False
                time.sleep(2.0)
                continue
            if code not in (-31, -2):   # 仅对 IK/泛规划失败做微扰重试
                return False
            attempt += 1
            if attempt > self._ik_retries:
                return False

    def move_to_joints(self, joint_names, values, label: str = '',
                       timeout: float = 60.0,
                       settle_check: bool = True) -> bool:
        """关节空间 PTP(目标 = JointConstraint,规划器不做 IK)。

        settle_check(默认开)启用**关节到位闸兜底**:lift goal 容差收紧
        0.05 后,含满伸展静态下垂(0.2306 rad)的目标 action 永不成功;
        且历史上背靠背目标偶发结果不回传(2026-10-02 实测物理到位静止
        而 60s 无终态)——等待期间每秒查关节缓存,误差 < 0.55 rad 且静止
        即取消目标按成功继续(补偿目标稳定误差 0.4612 属预期)。
        """
        log = self._node.get_logger()
        if self._refuse_if_dead(label):
            return False
        for busy in range(1 if label == "return_standby" else 4):
            ok = self._move_to_joints_once(joint_names, values, label,
                                           timeout, log,
                                           settle_check=settle_check)
            if ok:
                return True
            if self._wedge_detect and self.session_dead:
                log.error(f'{label}: 会话判死,停止退避重试')
                return False
            if busy < 3:
                log.warn(f'{label}: 控制器忙或失败,退避 2s 重试({busy + 1}/3)')
                time.sleep(2.0)
        return False

    def _joints_settled(self, joint_names, values,
                        pos_tol: float = 0.55, vel_tol: float = 0.3) -> bool:
        """读执行器关节缓存判定是否已物理到位且静止(数据新鲜度 <0.5s)。

        pos_tol 0.55:满伸展 anchor 目标含 +0.2306 下垂前置补偿,稳定
        误差 0.4612 rad 属预期(稳定位姿=格子),须容忍;>0.55 才视为
        真未到位。"""
        node = self._node
        wall = getattr(node, '_js_wall', None)
        if wall is None or time.time() - wall > 0.5:
            return False
        for name, want in zip(joint_names, values):
            pos = node._joints_now.get(name)
            vel = node._joint_vels.get(name)
            if pos is None or vel is None:
                return False
            if abs(pos - float(want)) > pos_tol or abs(vel) > vel_tol:
                return False
        return True

    def _move_to_joints_once(self, joint_names, values, label: str,
                             timeout: float, log,
                             settle_check: bool = False):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self._group
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self._vel
        req.max_acceleration_scaling_factor = self._acc
        req.pipeline_id = 'pilz_industrial_motion_planner'
        req.planner_id = 'PTP'
        req.start_state = RobotState()

        constraints = Constraints()
        for name, value in zip(joint_names, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = self._split
        goal.planning_options.replan = False

        self._wait_slot_free(label='joints_send')
        future = self._client.send_goal_async(goal)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                log.error(f'{label}: 目标受理超时')
                return False
            self._future_pause(future, 0.05)
        handle = future.result()
        if not handle.accepted:
            log.error(f'{label}: 目标被 move_group 拒绝')
            return False
        result_future = handle.get_result_async()
        # 到位闸兜底(settle_check):5s 起每秒查关节缓存,物理到位且静止
        # 即按成功继续。**不 cancel 挂起目标**:cancel 在 rcl_action 的
        # EXECUTING→CANCELED 窗口可触发 move_group RCLError 崩溃
        # (2026-10-03 实证,崩溃后所有后续目标受理超时);open_loop JTC
        # 的旧轨迹会按时间参数自行走完,物理臂已静止(settle_check 判据),
        # 下一个目标的受理重试(退避 2s x4)足以覆盖收尾窗口
        next_settle_check = time.monotonic() + 5.0
        while not result_future.done():
            if time.monotonic() > deadline:
                if (not self._split and settle_check
                        and self._joints_settled(joint_names, values)):
                    self._drain_result(result_future)
                    log.warn(f'{label}: 结果超时但关节已物理到位,按成功继续')
                    self._note_outcome(True, dangling=True)   # R1-P1
                    return True
                log.error(f'{label}: 执行超时(放弃等待,不取消)')
                # G-A(forensics):该分支 goal 同样悬置占槽(demo_watch
                # 10:04 arm1 settle_rise 型楔死正是走它:关节闸未过→60s
                # 超时→此后 -4 洪水),必须记 dangling 才能被抓到。
                self._note_outcome(False, dangling=True)      # R1-P1
                return False
            if (not self._split and settle_check
                    and time.monotonic() >= next_settle_check
                    and self._joints_settled(joint_names, values)):
                self._drain_result(result_future)
                log.warn(f'{label}: 动作结果未回传但关节已物理到位,按成功继续')
                self._note_outcome(True, dangling=True)       # R1-P1
                return True
            if time.monotonic() >= next_settle_check:
                next_settle_check += 1.0
            self._future_pause(result_future, 0.05)
        result_wrap = result_future.result()
        if result_wrap.status != GoalStatus.STATUS_SUCCEEDED:
            code = result_wrap.result.error_code.val
            log.error(f'{label}: 动作未成功 status={result_wrap.status} '
                      f'code={code}')
            self._note_outcome(False, code=code)              # R1-P1
            return False
        if label in ('standby','return_standby') and result_wrap.result.error_code.val != 1:
            log.error(f'{label}: MoveIt did not report SUCCESS');return False
        if self._split:
            ok = self._execute_split(
                result_wrap.result, label, timeout,
                (lambda: self._joints_settled(joint_names, values))
                if settle_check else (lambda: False))
            self._note_outcome(ok)
            return ok
        log.info(f'{label}: 完成 (关节空间)')
        self._note_outcome(True)                              # R1-P1(G-C 清零)
        return True

    def _execute_split(self, moveit_result, label: str, timeout: float,
                       settled_fn) -> bool:
        """把规划结果交给本臂的 FollowJointTrajectory。

        同一条控制器上,下一条轨迹要等这一条结束。关节已经进到原来的
        到位闸里、但控制器因为 goal_time 报了失败时,仍按成功继续,
        和以前 move_group 路径的到位闸一样。不取消 move_group。
        """
        log = self._node.get_logger()
        planned = getattr(moveit_result, 'planned_trajectory', None)
        traj = None if planned is None else planned.joint_trajectory
        if traj is None or not traj.points:
            log.error(f'{label}: 规划结果里没有关节轨迹')
            return False
        traj.header.stamp.sec = 0
        traj.header.stamp.nanosec = 0
        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        log.info(f'{label}: 规划完成,改由本臂控制器执行 '
                 f'({len(traj.points)} 点)')
        self._own_goal_inflight=True
        self.last_controller_result=None
        send_future = self._traj_client.send_goal_async(goal)
        deadline = time.monotonic() + timeout
        while not send_future.done():
            if time.monotonic() > deadline:
                log.error(f'{label}: 控制器目标受理超时')
                return False
            self._future_pause(send_future, 0.05)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            log.error(f'{label}: 控制器拒绝轨迹')
            return False
        result_future = handle.get_result_async()
        saw_settled = False
        while not result_future.done():
            if not saw_settled and settled_fn():
                saw_settled = True
            if time.monotonic() > deadline:
                log.error(f'{label}: 控制器执行超时')
                handle.cancel_goal_async()
                return False
            self._future_pause(result_future, 0.05)
        wrapped = result_future.result()
        status = wrapped.status
        error_code = getattr(wrapped.result, 'error_code', None)
        self._own_goal_inflight=False
        strict=label in ('standby','return_standby')
        self.last_controller_result={'label':label,'status':status,'error_code':error_code,'success':status==GoalStatus.STATUS_SUCCEEDED and error_code==0}
        log.info('CONTROLLER_RESULT '+__import__('json').dumps(self.last_controller_result))
        if status == GoalStatus.STATUS_SUCCEEDED and (not strict or error_code==0):
            log.info(f'{label}: 控制器完成')
            return True
        if not strict and (saw_settled or settled_fn()):
            log.warn(f'{label}: 控制器状态 {status} 但已到位,按成功继续')
            return True
        log.error(f'{label}: 控制器未成功 status={status}')
        return False

    def _tool_settled(self, pose: PoseStamped,
                      pos_tol: float = 0.025) -> bool:
        """位姿到位闸:tool0 TF 距目标 < pos_tol 且静止(0.15s 位移 <4mm)。

        lift goal 容差收紧到 0.05 后,含 lift 稳态下垂(满伸展 0.23 rad)
        的 LIN 目标 action 永不"成功"——用物理位姿判定兜底(与执行器
        到位闸同思路,面向 Cartesian 腿)。"""
        try:
            # G-B/§1.5 latent bug:此处原引用 self._tool_frame(该属性只在
            # Node 上,MotionClient 只有 self._tool)⇒ 恒 AttributeError
            # 被下方 except 吞掉 ⇒ _tool_settled 恒 False ⇒ 笛卡尔腿永远
            # 等满超时(60s)且留 dangling。修正由**单独开关**保护:关闭时
            # 直接返回 False = 与旧行为逐字等价(不引入 arm1 定案态时序变化)。
            if not self._tool_settle_fix:
                return False
            tf = self._node._attach._tf.lookup_transform(
                'world', self._tool, rclpy.time.Time())
            t1 = tf.transform.translation
            import math as _m
            d1 = _m.dist((t1.x, t1.y, t1.z),
                         (pose.pose.position.x, pose.pose.position.y,
                          pose.pose.position.z))
            if d1 > pos_tol:
                return False
            time.sleep(0.15)
            tf2 = self._node._attach._tf.lookup_transform(
                'world', self._tool, rclpy.time.Time())
            t2 = tf2.transform.translation
            return _m.dist((t1.x, t1.y, t1.z), (t2.x, t2.y, t2.z)) < 0.004
        except Exception:  # noqa: BLE001 TF 暂不可用 = 未到位
            return False

    def _send_goal(self, pose: PoseStamped, label: str, timeout: float,
                   linear: bool):
        """发送单次 MoveGroup 目标,返回 (是否成功, 错误码)。

        等待结果期间 5s 起每秒做位姿到位检查:lift goal 容差 0.05 下,
        稳态下垂会让 action 永不成功(见 _tool_settled 注释),物理到位
        即取消目标按成功继续。"""
        log = self._node.get_logger()
        self._wait_slot_free(label='pose_send')
        future = self._client.send_goal_async(
            self._build_goal(pose, linear=linear, plan_only=self._split))
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                log.error(f'{label}: 目标受理超时')
                return False, 0
            self._future_pause(future, 0.05)
        handle = future.result()
        if not handle.accepted:
            log.error(f'{label}: 目标被 move_group 拒绝')
            return False, 0

        result_future = handle.get_result_async()
        next_settle_check = time.monotonic() + 5.0
        # 不 cancel 挂起目标(理由见 _move_to_joints_once 同位注释:cancel
        # 可触发 move_group rcl_action 取消竞态崩溃)
        while not result_future.done():
            if time.monotonic() > deadline:
                if not self._split and label not in ('standby','return_standby') and self._tool_settled(pose):
                    self._drain_result(result_future)
                    log.warn(f'{label}: 结果超时但工具已物理到位,按成功继续')
                    self._note_outcome(True, dangling=True)   # R1-P1
                    return True, 1
                log.error(f'{label}: 执行超时(放弃等待,不取消)')
                self._note_outcome(False, dangling=True)      # R1-P1(G-A)
                return False, 0
            if (not self._split and label not in ('standby','return_standby') and time.monotonic() >= next_settle_check
                    and self._tool_settled(pose)):
                self._drain_result(result_future)
                log.warn(f'{label}: 动作结果未回传但工具已物理到位,按成功继续')
                self._note_outcome(True, dangling=True)       # R1-P1
                return True, 1
            if time.monotonic() >= next_settle_check:
                next_settle_check += 1.0
            self._future_pause(result_future, 0.05)

        result_wrap = result_future.result()
        status = result_wrap.status
        result = result_wrap.result
        code = result.error_code.val
        if status != GoalStatus.STATUS_SUCCEEDED:
            log.error(f'{label}: 动作未成功 status={status} code={code}')
            self._note_outcome(False, code=code)              # R1-P1
            return False, code
        if label in ('standby','return_standby') and code != 1:
            log.error(f'{label}: MoveIt did not report SUCCESS');return False,code
        if self._split:
            ok = self._execute_split(
                result, label, timeout, lambda: self._tool_settled(pose))
            self._note_outcome(ok)
            return ok, code if ok else 0
        log.info(f'{label}: 完成 (code={code})')
        self._note_outcome(True)                              # R1-P1
        return True, code
