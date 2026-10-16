"""取放执行节点:接收任务命令,规划执行 抓取 -> 搬运 -> 放置。

依赖(运行时):
- move_group(官方 UR MoveIt 配置)负责规划与轨迹执行,本节点通过
  自研的 MoveGroup 动作客户端(motion_client.py)下发位姿目标;
- 夹爪:ros2_control 的 JointGroupPositionController,话题下发目标开度;
- 抓取固定:Gazebo 世界服务的"仿真附着"(课程允许的简化,见 README)。

失败处理(可选拓展之一):
- 每件任务最多尝试 attempts 次;规划失败/执行超时/附着失败都会触发;
- 重试前做安全恢复(松爪、解附着);两次仍失败则回报 FAILED,
  调度器记录后跳过该件继续下一件;
- 参数 inject_failure_parts 可对指定工件注入一次"首次抓取失败",
  用于演示失败恢复流程。

流水线模式扩展(契约 §6/§8/§9;新参数默认值=旧行为完全回归):
- place_drop_clearance>0:放置段最后下探终点抬高 clearance(不下探到
  放置面),到位松爪让工件自由落入箱内;=0.0 下探到放置面松爪(现状);
- attach_follow='verified_only':闭爪前开始采样(夹爪 xy 取附着链接 TF,
  工件 xy 取 /overhead_logical 真值),抬升后先做水平验证探针
  (verify_probe_offset,组长仲裁方案 B),再采 verify_sample_span 秒
  后调 GraspVerifier 验证,通过才 attach 兜底,失败抛 GraspError(
  'grasp_not_verified')走既有重试/恢复;'always'(默认)跳过验证直接
  attach(现状)。verified_only 下即使 pose_from_camera=false 也订阅
  逻辑相机(无条件缓存采样)。
"""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_sorting_core.attach_helper import GzKinematicAttach
from a1_sorting_core.camera_pose import CameraPoseCache
from a1_sorting_core.dispatch_logic import (UnknownTargetBin,
                                            resolve_target_bin)
from a1_sorting_core.grasp_verifier import GraspVerifier
from a1_sorting_core.motion_client import MotionClient
from a1_sorting_core.pipeline_logic import (place_descend_target_z,
                                            verify_probe_targets)
from a1_sorting_core.state_machine import (IllegalTransition, TaskState,
                                           TaskStateMachine)


class GraspError(Exception):
    """可重试的执行失败(规划失败/附着失败等)。"""


class FatalGraspError(GraspError):
    """不可重试的失败(如未知类别)。"""


# 夹取验证采样周期(秒):夹爪 xy 来自 TF(约等于 TF 发布率),工件 xy
# 来自逻辑相机;一拍内两路都有效才记一对,保证两序列等长且时间对齐
_VERIFY_SAMPLE_PERIOD = 0.05


class _FollowSampler:
    """夹取验证采样器:后台线程收集时间对齐的夹爪 xy / 工件 xy 序列。

    gripper_xy_fn / part_xy_fn 任一返回 None(如 TF 暂不可见)时跳过
    该拍;stop() 返回 (夹爪序列, 工件序列),等长,可直接喂
    GraspVerifier.verify。线程为 daemon,stop() 幂等。
    """

    def __init__(self, gripper_xy_fn, part_xy_fn,
                 period: float = _VERIFY_SAMPLE_PERIOD):
        self._grip_fn = gripper_xy_fn
        self._part_fn = part_xy_fn
        self._period = float(period)
        self._stop = threading.Event()
        self._thread = None
        self._gripper: list[tuple[float, float]] = []
        self._part: list[tuple[float, float]] = []
        self._sample_times = []
        self._source_stamps = []
        self._source_fn = lambda: None
        self._source_fresh = lambda: True
        self._condition = threading.Condition()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            g = self._grip_fn()
            p = self._part_fn()
            if g is not None and p is not None:
                with self._condition:
                    self._gripper.append((float(g[0]), float(g[1])))
                    self._part.append((float(p[0]), float(p[1])))
                    self._sample_times.append(time.monotonic())
                    self._source_stamps.append(self._source_fn())
                    self._condition.notify_all()
            self._stop.wait(self._period)

    def wait_evidence(self, span, verifier):
        """Never shorten an unsuccessful/undersampled probe; retain original wall cap."""
        deadline = time.monotonic() + span
        with self._condition:
            while True:
                count = len(self._sample_times)
                elapsed = self._sample_times[-1]-self._sample_times[0] if count>1 else 0.
                distinct = len({x for x in self._source_stamps if x is not None})
                enough = count >= math.ceil(span/self._period)+1 and elapsed >= span and distinct >= max(2, math.ceil(span*10))
                if enough and self._source_fresh() and verifier(self._gripper,self._part)[0]:return True
                remaining = deadline-time.monotonic()
                if remaining <= 0:return False
                self._condition.wait(remaining)

    def stop(self) -> tuple[list, list]:
        """停止采样并返回两路序列(重复调用返回同一结果)。"""
        if self._thread is not None:
            self._stop.set()
            self._thread.join(timeout=2.0)
            self._thread = None
        return self._gripper, self._part


# 关节名默认值(arm2 等 tf_prefix 臂由参数文件提供前缀化的关节名)
_JOINT_NAMES_DEFAULT = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

# P1 双存键(2026-10-08):第二臂前缀表,与 gripper.urdf.xacro 的 ${prefix}
# / ur5e_cell_dual.urdf.xacro 的 tf_prefix 对齐。若 joint_states 发布者
# 发的是**裸关节名**(双臂同名在同一条消息里出现两次,按 ros2_control
# 声明顺序 arm1 在前),第 i+1 次出现的同名按本表取第 i 个前缀另存一份,
# 保证双臂都查得到;发布者自带前缀时无重名 ⇒ 行为与旧版完全一致(零回归)。
_JOINT_ALT_PREFIXES = ('arm2_',)


def should_cap(dl: float, dr: float, lead_m: float = 0.015,
               lag_m: float = 0.005) -> bool:
    """sync_cap 封顶判据(纯函数,P1 单测覆盖)。

    dl/dr = 两指自轻触段起点的闭合行程(米,正=向闭合方向)。
    单侧先行(一侧 ≥ lead_m 且另一侧 < lag_m)⇒ 轻触段提前结束,
    交给锁紧段对称合拢。纯函数化原因:此前判据内联在轮询循环里,
    键名解析失效时 dl/dr 恒 0、条件永不成立,整条触发路径没有可测
    面(10-06 复验"零触发"实证的根)——拆成纯函数后判据本身可单测,
    失效只可能出在取数环节(启动自检打印兜底,静默不可能)。"""
    return max(dl, dr) >= lead_m and min(dl, dr) < lag_m


def fingers_settled(dl: float, dr: float, tol_m: float = 0.002) -> bool:
    """指关节停稳闸判据(纯函数,P2-2a 单测覆盖)。

    dl/dr = 两指在最近 0.1s 采样窗内的位移(米)。双侧均 |Δ| < tol_m
    (默认 2mm)视为夹持停稳。口径与 sync_cap 的 should_cap 同族:
    判据纯函数化便于单测,取数经 _joint_lookup(两臂双世界都查得到)。"""
    return abs(dl) < tol_m and abs(dr) < tol_m


def fingers_released(pl: float, pr: float, release_gap: float = 0.052) -> bool:
    """释放脱件闸判据(纯函数;2026-10-09 改**间隙口径**)。

    pl/pr = 两指关节位(米,q 增大=张开,对开夹爪见 xacro)。双侧内缘
    间隙 = (0.026+ql)+(0.026+qr) = 0.052+ql+qr,间隙 ≥ release_gap
    (默认 52mm > 工件 50mm)即脱件。

    为什么从"双指各自 ≥ release_q"改成间隙口径(实机释放实录 6/8
    超时):夹爪是**单条共享 effort 命令**(JointGroupEffortController
    两指同值),旧判据要求两指都爬到 +0.001,实际常有一指被接触/邻件
    钉在闭位 -0.003 不动——+1.5N 只把**自由侧**一路推到 +0.040 全张
    限位(另一侧纹丝不动,判据永不满足),而全张指外缘到箱心 0.128,
    插进壁带 43mm,抬臂全程刮壁 = 截图"顶盒沿"的真身。间隙口径下自
    由侧只需 +0.005(外缘 0.043,±0.05 格外缘 0.093,对壁内缘 0.085
    重叠 ≤8mm 且 0N 内收收窄)即判脱件、转 0N 中立,不再放任狂奔。"""
    return (0.052 + pl + pr) >= release_gap


def fingers_clamped(ql: float, qr: float, max_gap_m: float = 0.055) -> bool:
    """夹持确认闸判据(纯函数,用户拍板 2026-10-08 晚,单测覆盖)。

    ql/qr = 两指关节位。指内缘 y = 0.032+q-0.006 = 0.026+q
    (gripper.urdf.xacro 指关节原点 ±0.032、指宽 12mm),双侧内缘
    间隙 = 0.052+ql+qr。真夹住工件 ⇔ 间隙 ≤ max_gap_m(默认 55mm):
    工件 50mm 被夹时两指停在间隙≈50mm(刚性接触,限位 -0.003 的
    3mm 越程只保证能压上);空爪闭到底 46mm 也过(无件可夹的场合
    由后续验证把关);而"一指卡全开限位"的钉死态间隙 89mm、预收
    口态 62mm 都被拒。

    为什么停稳闸不够(481k 行 effort 实录定罪,HANDOFF-20261008
    §十六):轻触段单指先行 41mm 撞上工件继续推,工件横滑楔到另一
    指(该指卡全开限位 0.04,限位反力 -32.6N)——此刻**两指都静止**
    (一撞限位一被工件顶住),停稳闸被骗过放行抬臂,间隙 89mm 零夹
    持力,工件全靠 attach 兜底。夹持真伪必须量"合没合上",不能只
    量"停没停"。"""
    return (0.052 + ql + qr) <= max_gap_m


# 各料区放置构型关节角默认值(实测标定);新料区(如 bin_out)由参数文件提供
_PLACE_JOINTS_DEFAULTS = {
    'bin_A': [-0.6424, -1.253, 0.9995, -1.3173, -1.5708, -2.2132],
    'bin_B': [0.2135, -1.253, 0.9995, -1.3173, -1.5708, -1.3573],
}


def _quat_mul(a, b):
    """四元数乘法 (x, y, z, w)。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _downward_quat(yaw: float):
    """工具竖直向下(夹持面朝下)的姿态四元数。

    由"绕 Z 轴偏航 yaw"与"绕 X 轴翻转 180 度"两个旋转复合得到。
    """
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    qx = (1.0, 0.0, 0.0, 0.0)
    return _quat_mul(qz, qx)


class PickPlaceExecutorNode(Node):

    def __init__(self):
        super().__init__('pick_place_executor')
        # ---- 双臂参数化(默认值与单臂行为完全一致) ----
        self.declare_parameter('arm_name', 'arm1')   # 执行臂标识,填 TaskStatus.arm
        self.declare_parameter('command_topic', '/task/command')  # 任务命令话题
        self.declare_parameter('pose_from_camera', False)  # true: 取件 xy 用相机真值
        self.declare_parameter('camera_xy', [0.65, 0.0])   # 逻辑相机安装 xy
        self.declare_parameter('bin_names', ['bin_A', 'bin_B'])  # 料区列表
        self.declare_parameter('joint_names', _JOINT_NAMES_DEFAULT)
        bin_names = [str(n) for n in self.get_parameter('bin_names').value]
        # ---- 运动学相关 ----
        self.declare_parameter('group_name', 'ur_manipulator')
        self.declare_parameter('planning_frame', 'world')
        self.declare_parameter('tool_frame', 'tool0')
        self.declare_parameter('tcp_z_offset', 0.10)   # tool0 到指尖的竖直距离
        # 来料/取件面工件静置中心:工件半高 0.025 + 支撑面高度。
        # legacy 默认 0.745(带面静置槽);pipeline 带顶 0.78 -> 0.805,
        # 双臂接力自箱内取件 -> 0.745(箱内底 0.72)。由 yaml/launch 覆盖,
        # 默认值保持旧行为(契约 §10)
        self.declare_parameter('feed_z', 0.745)
        # 放置落入模式(契约 §9):>0 时放置段最后下探终点 z =
        # place_z + tcp_z + place_drop_clearance(不下探到放置面),
        # 到位松爪让工件自由落入箱内;<=0(默认 0.0)下探到放置面(旧行为)
        self.declare_parameter('place_drop_clearance', 0.0)
        # 附着策略(契约 §8):always=抓取即 attach(旧行为);
        # verified_only=闭爪->抬升后 GraspVerifier 验证通过才 attach 兜底
        self.declare_parameter('attach_follow', 'always')
        # 跟随频率(搬运失真修复 2026-10-09):40Hz 名义节拍在"不丢拍"
        # 修正后已是硬下限,但 set_pose 往返 + TF 采样抖动仍留 ~5mm 级
        # 滞后;80Hz 把滞后压到亚毫米(本地 gz 服务,无网络成本)。
        # 默认 40 = legacy 零行为差。
        self.declare_parameter('attach_rate_hz', 40.0)
        self.declare_parameter('verify_gripper_move', 0.03)  # GraspVerifier 阈值
        self.declare_parameter('verify_follow_move', 0.02)
        self.declare_parameter('fast_sim_waits', False)
        self._fast_waits = bool(self.get_parameter('fast_sim_waits').value)
        self._part_source_stamps = {}
        self._part_source_walls = {}
        self.declare_parameter('verify_sample_span', 0.5)  # 抬升后追加采样秒数
        # 水平验证探针(组长仲裁方案 B):verified_only 抬升完成后、verify
        # 前往返 LIN 平移该偏移(+x 去/回),给验证器一个夹爪水平位移窗
        # (0.04 > gripper_move_thresh 0.03);0 = 禁用探针
        self.declare_parameter('verify_probe_offset', 0.04)
        # 关节空间 pan 弧探针(G3 根治,2026-10-05):非 0 时探针改走
        # move_to_joints——当前关节 + pan 关节 ±此弧度,往返一次。
        # 零在线 IK:带尾深折构型的 pose 目标(LIN 或 PTP 皆过笛卡尔 IK)
        # 分支随机,命中劣分支 ⇒ 60s 悬置 ⇒ -4 洪水(r1c/r1d 两轮实证)。
        # Δpan 与水平位移换算(离线 FK 标定 arm2 pick 构型):
        #   Δpan=-0.085 ⇒ tool0 +x 40mm(远离挡停板方向,与 LIN 探针
        #   "向带尾"同向性由符号约定);位移量仍 > gripper_move 0.03。
        # 0=禁用,走原 pose 探针。
        self.declare_parameter('verify_probe_pan_delta_rad', 0.0)
        # 探针腿规划器:true=LIN 往返(默认,历史行为;arm1@0.2 实证正常),
        # false=PTP 直达。G3(2026-10-05):arm2 深折构型的 40mm 短 LIN 被
        # Pilz 时间参数化拉成 >60s 爬行(r1c 实录:Accepted 后 60s 无 Goal
        # reached ⇒ 悬置 goal 占槽 ⇒ -4 洪水判死),同 leg2 爬行前科。
        # PTP 只在目标做一次 IK,从待机构型直达,无笛卡尔采样抖动;端点位
        # 移(+x 40mm)不变,验证器"夹爪动且工件跟随"判据不受影响,中段弧线
        # 偏移由 verify_follow 物理判据兜底(不跟随即判失败重试)。
        self.declare_parameter('verify_probe_linear', True)
        self.declare_parameter('grasp_yaw_deg', 0.0)
        self.declare_parameter('pre_grasp_height', 0.08)
        self.declare_parameter('lift_height', 0.10)
        # lift 分级爬升数(1=一步;浅折带载失速构型用 3)
        self.declare_parameter('lift_rungs', 1)
        self.declare_parameter('carry_safe_z', 1.15)  # 搬运安全巡航高度(tool0)
        self.declare_parameter('standby_x', 0.45)    # 待机位(工作区中央上方)
        self.declare_parameter('standby_y', 0.0)
        self.declare_parameter('standby_z', 1.05)
        # 待机位关节角(实测标定,与上面位姿对应):跨侧横移后 KDL 会落到
        # 翻转 IK 分支(部分放置目标在该分支无解),经由此关节角可强制
        # 回到标定分支,使所有任务的后半程路径完全一致
        self.declare_parameter('standby_joints', [
            -0.301, -1.636, 1.213, -1.148, -1.571, -1.872])
        # 放置构型关节角(按料区标定,TCP 在该料区第一格上方 lift 高度):
        # 跨侧横移后 KDL 的位姿求解有分支随机性,放置段改用关节空间直达。
        # 按料区列表动态声明 place_joints_<name>(bin_A/bin_B 默认值保持
        # 原标定值;新料区如 bin_out 由参数文件提供)。未知料区默认给
        # 6 元素零向量而非空列表:rclpy 会把空列表推断成 BYTE_ARRAY,
        # 与参数文件的 DOUBLE_ARRAY 覆盖值类型冲突直接抛异常
        for name in bin_names:
            self.declare_parameter(f'place_joints_{name}',
                                   _PLACE_JOINTS_DEFAULTS.get(name, [0.0] * 6))
        # 取件侧锚定关节角(可选;默认 NaN 数组 = 槽位禁用):跨侧取件的首段
        # 位姿 PTP 有 KDL 分支翻转风险(翻转分支的臂身会推挤工件),先以
        # 确定性关节目标到达取件区上空,再小平移+短距下探。两个槽位配
        # 各自的锚点 xy,执行时按取件目标就近选择。注意默认值不能用空
        # 列表(rclpy 会推断成 BYTE_ARRAY,与参数文件的 DOUBLE_ARRAY
        # 覆盖值类型冲突直接抛异常)
        self.declare_parameter('pick_anchor1_xy', [1e9, 1e9])
        self.declare_parameter('pick_anchor1_joints', [float('nan')] * 6)
        self.declare_parameter('pick_anchor2_xy', [1e9, 1e9])
        self.declare_parameter('pick_anchor2_joints', [float('nan')] * 6)
        # 取件深度锚(可选,默认 NaN 禁用):取件抓取深度的确定性关节
        # 目标,按取件目标 xy 就近选择。命中时跳过 pre_grasp+descend 的
        # 笛卡尔腿(在线 KDL 的 LIN 分支不可控,arm2 取件在 descend_2
        # 卡死,2026-10-03 实证),直接 PTP 到抓取构型
        for _slot in range(1, 7):
            self.declare_parameter(f'pick_grasp_anchor{_slot}_xy', [1e9, 1e9])
            self.declare_parameter(f'pick_grasp_anchor{_slot}_joints',
                                   [float('nan')] * 6)
        # 搬运(放置侧)关节锚定,按目标箱就近选择(可选,默认 NaN 禁用):
        # Cartesian carry_shift 的 KDL IK 分支随机性会选中臂身低扫(撞带/
        # 推板,规划场景拒绝后 -2 死循环);关节空间直达从待机构型出发
        # 路径确定。值来自 [标定] 自动记录(成功 carry_shift 的关节角)
        self.declare_parameter('carry_anchor1_xy', [1e9, 1e9])
        self.declare_parameter('carry_anchor1_joints', [float('nan')] * 6)
        self.declare_parameter('carry_anchor2_xy', [1e9, 1e9])
        self.declare_parameter('carry_anchor2_joints', [float('nan')] * 6)
        # 槽位 3-6:流水线每格一锚(两箱 x 3 格,z=1.08 关节直达,
        # 2026-10-02:LIN 腿在 0.65m 可达边界 KDL 分支翻转冻结,弃用)
        for _slot in range(3, 7):
            self.declare_parameter(f'carry_anchor{_slot}_xy', [1e9, 1e9])
            self.declare_parameter(f'carry_anchor{_slot}_joints',
                                   [float('nan')] * 6)
        # 放入式放置(set-down)触底下探锚定,与 carry_anchor 槽位一一对应
        # (xy 同格,z=0.99/0.95 上下两端;运行时线性插值做触底梯子,
        # 消除 P 控制下垂不确定性;默认 NaN = 禁用,回落落入式松爪)
        for _slot in range(1, 7):
            self.declare_parameter(f'carry_settle_top{_slot}_joints',
                                   [float('nan')] * 6)
            self.declare_parameter(f'carry_settle_bottom{_slot}_joints',
                                   [float('nan')] * 6)
        # 触底判定:tool0 世界 z ≤ 此值视为工件已触箱底(触底几何位
        # 0.953 = 箱内静置中心 0.745 + tcp_z 0.208;+4mm 容 TF/微振)
        self.declare_parameter('settle_touch_z', 0.957)
        # 满伸展静态下垂补偿(两段式第二段的抬升量,0=关;见 CARRYING 段)
        self.declare_parameter('carry_lift_comp', 0.0)
        # 取件下探是否用 LIN(默认 true);远距取件点置 false 见 _descend 注释
        self.declare_parameter('pick_descend_linear', True)
        # 放置侧锚定关节角(可选;默认 NaN = 禁用):近基座的放置格
        # (距基座 <0.3m)上 KDL 会确定性地选到臂身翻转的 IK 族;与取件
        # 锚定同理,先以确定性关节目标贴到放置区上方再小平移
        self.declare_parameter('place_anchor_joints', [float('nan')] * 6)
        self.declare_parameter('descend_steps', 3)
        # 运动速度/加速度缩放(Pilz 轨迹时间参数化):LIN 段的关节加/减速度
        # 需求 ∝ 1/时长²,近基座构型下 0.5 会踩到 5.0 rad/s² 限值的边缘
        # (超 0.2% 即被确定性拒绝),需要时降到 0.25(需求降至 ~1/4)
        self.declare_parameter('vel_scale', 0.5)
        self.declare_parameter('acc_scale', 0.5)
        # ---- 失败处理 ----
        self.declare_parameter('attempts', 2)          # 总尝试次数(含首试)
        self.declare_parameter('inject_failure_parts', [''])
        # ---- R1-P1/P3 楔死判死(2026-10-04 终审,forensics §2)----
        # 默认全关 = arm1 定案态逐行零行为差。wedge_detect:MotionClient
        # 记账"悬置 goal + 连续 -4"⇒ session_dead,当件直接 FAILED
        # (detail=motion_session_wedged)且不烧重试链;wedge_rearm:下一件
        # 丢弃旧连接重建(配合外层受控重启 move_group)。
        self.declare_parameter('split_plan_execute', False)
        self.declare_parameter('controller_action', '')
        self.declare_parameter('wedge_detect', False)
        self.declare_parameter('wedge_rearm', False)
        # G-B/§1.5:_tool_settled 引用不存在属性致笛卡尔腿恒超时——修正
        # 单独开关(默认关=保留旧行为,不改 arm1 时序)
        self.declare_parameter('tool_settle_fix', False)
        # ---- 夹爪 ----
        self.declare_parameter('gripper_topic', '/gripper_controller/commands')
        self.declare_parameter('gripper_status_topic', '/gripper/status')
        self.declare_parameter('gripper_open', 0.04)   # 指尖滑移行程(见夹爪 xacro)
        self.declare_parameter('gripper_closed', 0.0)
        self.declare_parameter('gripper_joint_count', 2)
        self.declare_parameter('gripper_wait', 1.0)
        # 预收口(用户拍板 2026-10-08 晚,HANDOFF-20261008 §十六):闭爪
        # 前先把两指内缘间隙收到 0.052+2*gap,把单指空扫行程从 43mm 压
        # 到 gap+咬合量,消除"一侧指推着工件楔到另一侧指限位"的钉死构型。
        # 0 = 禁用(legacy 零行为差);pipeline 配 0.005(间隙 62mm,工件
        # 两侧各余 6mm 容差)。
        self.declare_parameter('gripper_pre_close_gap', 0.0)
        self.declare_parameter('gripper_pre_close_force', -1.5)
        self.declare_parameter('gripper_pre_close_wait', 0.5)
        # 实测对中(§十六终案,用户拍板 2026-10-08 晚):收窄闭爪后钉死
        # 位从开限位 0.040 移到 +0.0248(内缘 51mm)——恰为工件远面
        # (25+26mm 偏心):指轴离工件中心 ~26mm,一侧指从下探起就压在
        # 工件面上,另一指独走到底,间隙 74mm 零夹持。到位闸 tol=20mm
        # 与名义中心都可能带着系统性横偏(重力滞后)入带。修法:下探
        # 到位后用**实测**夹爪 xy(gripper_base TF)对**最新**相机真值
        # 求偏差,抬回 pre_h 平移修正再下探(带外平移,零扫掠)。
        # 0 = 禁用(legacy 零行为差)。
        self.declare_parameter('pick_center_tol', 0.0)
        self.declare_parameter('pick_center_max', 0.05)
        # 闭爪抬深(§十六终案定罪):指碰撞底边=指尖(tool0−0.231),标称
        # 取件(tool0=feed_z+tcp_z=0.805+0.204=1.009)时指尖 0.778,扎进
        # 带顶 0.780 达 2mm;闭爪横滑/接触修正把指楔得更深,μ=2.0 指面
        # 对带摩擦 > ±8N 全命令域 → 被蹭指冻结在任意行程位(遥测铁证:
        # +0.0242 双向拉不动,抬臂离带瞬间弹回),对侧独走,间隙 74mm 假
        # 夹持。闭爪前把 tool0 上提 lift_z:0.012 时指尖 0.790 离带净空
        # 10mm,指带 0.790-0.842 对工件侧面 0.780-0.830 仍重叠 40mm,
        # 夹持充分且冻结消失(闸过 8/8,两指对称 L≈R 间隙=50mm 真咬件)。
        # 0=禁用(legacy 零行为差)。
        self.declare_parameter('gripper_close_lift_z', 0.0)
        # 下探收窄(§十六定罪):全开外缘 0.078 距带板侧缘 0.080 仅 2mm,
        # wrist 重力 sag 微旋即把一侧指顶在带缘上(钉死实录:预收口起点
        # 就 L=-0.003/R=+0.040)。进带前先把间隙收到 0.052+2*gap(默认
        # 0.082,q≈0.015:外缘离带缘 27mm,内缘离工件面 16mm 容到位闸
        # 20mm 误差)。0 = 禁用(legacy 零行为差)。
        self.declare_parameter('gripper_descend_gap', 0.0)
        # ---- 类别 -> 料区 映射 ----
        # 2026-10-03 演示扩为四色:暖色 red/yellow -> bin_A,冷色
        # green/blue -> bin_B(色池 = spawn_parts.RANDOM_COLORS,四色等
        # 概率;HSV blue 区间 a1_vision 原本就有)。未知类别在 _bin_for
        # 抛 unknown_category -> 任务 FAILED,行为不变
        self.declare_parameter('bin_for_red', 'bin_A')
        self.declare_parameter('bin_for_green', 'bin_B')
        self.declare_parameter('bin_for_yellow', 'bin_A')
        self.declare_parameter('bin_for_blue', 'bin_B')
        # ---- 料区放置位姿(z 为工件静置中心的期望高度),按料区列表声明 ----
        for name in bin_names:
            self.declare_parameter(f'{name}.x', 0.70)
            self.declare_parameter(f'{name}.y', 0.0)
            self.declare_parameter(f'{name}.z', 0.75)
        self.declare_parameter('bin_grid_offsets.x', [-0.05, 0.05, -0.05, 0.05])
        self.declare_parameter('bin_grid_offsets.y', [-0.035, -0.035, 0.035, 0.035])
        # ---- 仿真附着(运动学跟随,见 attach_helper 模块注释) ----
        self.declare_parameter('world_name', 'sorting_cell')
        self.declare_parameter('gripper_attach_link', 'gripper_base')

        gp = lambda k: self.get_parameter(k).value
        self._arm_name = str(gp('arm_name'))
        self._group = str(gp('group_name'))
        self._planning_frame = str(gp('planning_frame'))
        self._tool_frame = str(gp('tool_frame'))
        self._tcp_z = float(gp('tcp_z_offset'))
        self._feed_z = float(gp('feed_z'))
        self._place_drop_clearance = float(gp('place_drop_clearance'))
        self._attach_follow = str(gp('attach_follow'))
        if self._attach_follow not in ('always', 'verified_only'):
            raise RuntimeError(
                f'attach_follow 取值非法: {self._attach_follow}'
                '(应为 always 或 verified_only)')
        self._verify_gripper_move = float(gp('verify_gripper_move'))
        self._verify_follow_move = float(gp('verify_follow_move'))
        self._verify_sample_span = float(gp('verify_sample_span'))
        self._verify_probe_offset = float(gp('verify_probe_offset'))
        self._verify_probe_pan_delta = float(gp('verify_probe_pan_delta_rad'))
        self._verify_probe_linear = bool(gp('verify_probe_linear'))
        if (self._attach_follow == 'verified_only'
                and 0.0 < self._verify_probe_offset
                <= self._verify_gripper_move):
            self.get_logger().warn(
                f'verify_probe_offset({self._verify_probe_offset}) 未超过 '
                f'verify_gripper_move({self._verify_gripper_move}),'
                f'探针窗口开不出来,验证将因夹爪无位移窗而失败')
        self._verifier = None
        if self._attach_follow == 'verified_only':
            self._verifier = GraspVerifier(self._verify_gripper_move,
                                           self._verify_follow_move)
        self._yaw = math.radians(float(gp('grasp_yaw_deg')))
        self._pre_h = float(gp('pre_grasp_height'))
        self._lift_h = float(gp('lift_height'))
        self._lift_rungs = max(1, int(gp('lift_rungs')))
        self._carry_z = float(gp('carry_safe_z'))
        self._carry_lift_comp = float(gp('carry_lift_comp'))
        self._standby = (float(gp('standby_x')), float(gp('standby_y')),
                         float(gp('standby_z')))
        self._standby_joints = [float(v) for v in gp('standby_joints')]
        self._place_joints = {
            name: [float(v) for v in gp(f'place_joints_{name}')]
            for name in bin_names
        }
        self._pick_anchors = []
        for slot in (1, 2):
            xy = [float(v) for v in gp(f'pick_anchor{slot}_xy')]
            joints = [float(v) for v in gp(f'pick_anchor{slot}_joints')]
            if (xy[0] < 1e8 and not any(math.isnan(v) for v in joints)):
                self._pick_anchors.append((xy, joints))
        self._pick_grasps = []
        for slot in range(1, 7):
            xy = [float(v) for v in gp(f'pick_grasp_anchor{slot}_xy')]
            joints = [float(v) for v in gp(f'pick_grasp_anchor{slot}_joints')]
            if (xy[0] < 1e8 and not any(math.isnan(v) for v in joints)):
                self._pick_grasps.append((xy, joints))
        self._carry_anchors = []
        # 槽位 1-2 为每箱一锚(旧);3-6 为每格一锚(流水线 6 格)。
        # 取最近锚定:格子间距 >=5cm,格中心到自身锚最近,无歧义
        for slot in range(1, 7):
            xy = [float(v) for v in gp(f'carry_anchor{slot}_xy')]
            joints = [float(v) for v in gp(f'carry_anchor{slot}_joints')]
            if (xy[0] < 1e8 and not any(math.isnan(v) for v in joints)):
                self._carry_anchors.append((xy, joints))
        self._pick_descend_linear = bool(gp('pick_descend_linear'))
        # settle 锚对(上/下端关节角),按 carry_anchor 槽位 xy 建索引
        self._settle_pairs = {}
        for slot in range(1, 7):
            xy = [float(v) for v in gp(f'carry_anchor{slot}_xy')]
            top = [float(v) for v in gp(f'carry_settle_top{slot}_joints')]
            bot = [float(v) for v in gp(f'carry_settle_bottom{slot}_joints')]
            if (xy[0] < 1e8
                    and not any(math.isnan(v) for v in top + bot)):
                self._settle_pairs[(round(xy[0], 3), round(xy[1], 3))] = (top, bot)
        self._settle_touch_z = float(gp('settle_touch_z'))
        _place_anchor = [float(v) for v in gp('place_anchor_joints')]
        self._place_anchor = (None if any(math.isnan(v) for v in _place_anchor)
                              else _place_anchor)
        self._joint_names = [str(v) for v in gp('joint_names')]
        self._at_standby = False
        self._steps = max(1, int(gp('descend_steps')))
        self._vel_scale = float(gp('vel_scale'))
        self._acc_scale = float(gp('acc_scale'))
        self._attempts = max(1, int(gp('attempts')))
        # R1-P1/P3 开关(默认关=定案态零行为差)
        self._wedge_detect = bool(gp('wedge_detect'))
        self._wedge_rearm = bool(gp('wedge_rearm'))
        self._tool_settle_fix = bool(gp('tool_settle_fix'))
        self._split_plan_execute = bool(gp('split_plan_execute'))
        self._controller_action = str(gp('controller_action'))
        self._zone_published = False
        self._inject = [str(x) for x in gp('inject_failure_parts') if str(x)]
        self._gripper_open = float(gp('gripper_open'))
        self._gripper_closed = float(gp('gripper_closed'))
        self._pre_close_gap = float(gp('gripper_pre_close_gap'))
        self._descend_gap = float(gp('gripper_descend_gap'))
        self._center_tol = float(gp('pick_center_tol'))
        self._center_max = float(gp('pick_center_max'))
        self._close_lift = float(gp('gripper_close_lift_z'))
        self._pre_close_force = float(gp('gripper_pre_close_force'))
        self._pre_close_wait = float(gp('gripper_pre_close_wait'))
        self._gripper_joints = max(1, int(gp('gripper_joint_count')))
        self._gripper_wait = float(gp('gripper_wait'))
        self._color_to_bin = {
            'red': str(gp('bin_for_red')),
            'green': str(gp('bin_for_green')),
            'yellow': str(gp('bin_for_yellow')),
            'blue': str(gp('bin_for_blue')),
        }
        self._bins = {
            name: (float(gp(f'{name}.x')), float(gp(f'{name}.y')),
                   float(gp(f'{name}.z')))
            for name in bin_names
        }
        self._grid_x = [float(v) for v in gp('bin_grid_offsets.x')]
        self._grid_y = [float(v) for v in gp('bin_grid_offsets.y')]
        self._world = str(gp('world_name'))
        self._attach_link = str(gp('gripper_attach_link'))

        self._pub_status = self.create_publisher(TaskStatus, '/task/status', 10)
        self._pub_gripper = self.create_publisher(
            Float64MultiArray, str(gp('gripper_topic')), 10)
        self._pub_gripper_status = self.create_publisher(
            String, str(gp('gripper_status_topic')), 10)
        # ---- 观感修复①(2026-10-08):夹爪力保持 + 启动归位 ----
        # 隔离实验定罪(cell_bringup 单会话,无任务):
        # (a) 夹爪是纯 effort 控制,无指令时指**不受力**,dart 重力/
        #     接触把两指推向**相反**限位且两臂镜像(arm1 L=-0.003/
        #     R=+0.040,arm2 L=+0.040/R=-0.003)——截图"一闭一张"的
        #     出生态;干过活的臂末条指令=全开,两指都停开限位,看着
        #     "正常"。xacro 无 initial_position 通道(gz_ros2_control
        #     不支持),执行器侧归位是唯一零物理改动的修法;
        # (b) **单发指令会丢**:新建 publisher 单发 +8N,3s 后左指仍
        #     在 -0.003(DDS 首发布与控制器订阅发现竞态);持续发布则
        #     两指对称响应。旧 _grip 每段只发一次,丢包时该段退化为
        #     "指不受力自由漂"——同一条命令重发即可靠,零物理差。
        # 方案:10Hz 定时器重发最近一次力值(初值=全开 gripper_open,
        # 兼作启动归位;控制器本就保持最后收到的值,重发同值不改
        # 力学,只消灭"丢包自由漂"与"无指令出生态"两类不对称)。
        self._grip_hold = float(gp('gripper_open'))
        self._grip_hold_target = None   # 非 None=释放小开位置保持(观感②)
        self._grip_release_force = 1.5  # 位置保持 bang-bang 力(阻尼15→0.1m/s 缓推)
        self._pub_gripper_cmd(self._grip_hold)
        self.create_timer(0.1, self._grip_hold_tick)
        self.create_subscription(PartInfo, str(gp('command_topic')),
                                 self._on_command, 10)
        self._joints_now: dict[str, float] = {}
        self._joint_vels: dict[str, float] = {}
        # P1 双存键:裸关节名双份世界下第二份(=arm2)的别名表,
        # 键为前缀化名(arm2_gripper_finger_left_joint 等)
        self._joints_alias: dict[str, float] = {}
        self._joint_vels_alias: dict[str, float] = {}
        self._grip_key_checked = False   # 启动自检只打一次(首条 js 消息)
        self._js_wall: float | None = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20)

        # pose_from_camera=true:订阅俯视逻辑相机,缓存各工件真值 xy
        # (world_x=cam_x+rel_z, world_y=cam_y+rel_y,见 camera_pose 模块),
        # 任务开始以缓存 (true_x, true_y, feed_z) 为取件中心。
        # attach_follow=verified_only 时无条件订阅:夹取验证的工件 xy
        # 采样也来自该缓存(即使 pose_from_camera=false 也要采)
        self._pose_from_camera = bool(gp('pose_from_camera'))
        cam_xy = [float(v) for v in gp('camera_xy')]
        self._cam_cache = CameraPoseCache((cam_xy[0], cam_xy[1]))
        if self._pose_from_camera or self._attach_follow == 'verified_only':
            self.create_subscription(LogicalCameraImage, '/overhead_logical',
                                     self._on_logical, 10)

        self._attach = GzKinematicAttach(self, self._world,
                                         rate_hz=float(gp('attach_rate_hz')),
                                         camera_xy=(cam_xy[0], cam_xy[1]))
        self._motion = None   # 运动客户端,首件任务时惰性初始化(等 move_group)
        self._place_counts = {name: 0 for name in bin_names}
        self._busy = False
        self._pending = []    # 忙时到达的命令排队(不丢弃,防任务黑洞)
        self._terminal_status = None  # 终态暂存(回待机后再发布,见 _run_task)
        self._seq = 0
        self.get_logger().info(
            f'执行器就绪: 臂 {self._arm_name}, 规划组 {self._group}, '
            f'夹爪附着链接 {self._attach_link}, 附着策略 {self._attach_follow}'
            f'(放置落入间隙 {self._place_drop_clearance}),'
            f' 注入失败工件 {self._inject or "无"}')

    # ================================================================ 命令入口
    def _on_joint_state(self, msg: JointState):
        self._js_wall = time.time()
        self._store_joint_states(msg)
        if not self._grip_key_checked:
            self._grip_key_checked = True
            self._log_grip_key_selfcheck()

    def _store_joint_states(self, msg: JointState):
        """存关节状态(P1 双世界兜底,2026-10-08)。

        主表 _joints_now 原样存 msg.name(既有键路径零改动,前缀发布者
        世界下完全回归旧行为)。同一消息内出现**重名**关节 = 发布者发
        的是裸关节名、双臂同名各发一份(ros2_control 声明顺序 arm1 在
        前):第 1 份是 arm1 的,存主表;第 2..N 份按 _JOINT_ALT_PREFIXES
        另存别名表 _joints_alias。查询走 _joint_lookup(主表→别名表),
        于是 arm2 查 arm2_gripper_finger_* 在两种发布者世界下都查得到,
        且 arm1 的裸键永远拿第 1 份 ⇒ 不引入跨臂误读(HANDOFF-20261006
        §二 定罪的候选缺陷形态)。"""
        seen: dict[str, int] = {}
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            i = seen.get(name, 0)
            seen[name] = i + 1
            if i == 0:
                self._joints_now[name] = pos
                self._joint_vels[name] = vel
            elif i <= len(_JOINT_ALT_PREFIXES):
                alt = f'{_JOINT_ALT_PREFIXES[i - 1]}{name}'
                self._joints_alias[alt] = pos
                self._joint_vels_alias[alt] = vel

    def _joint_lookup(self, name: str):
        """按键名查当前关节角:主表优先,裸名双份世界下回落别名表。"""
        v = self._joints_now.get(name)
        return v if v is not None else self._joints_alias.get(name)

    def _finger_joint_keys(self) -> tuple[str, str]:
        """本臂两指关节键名(前缀按 gripper.urdf.xacro ${prefix} 对齐:
        arm1 无前缀,arm2 = "arm2_")。"""
        pre = '' if self._arm_name == 'arm1' else f'{self._arm_name}_'
        return (f'{pre}gripper_finger_left_joint',
                f'{pre}gripper_finger_right_joint')

    def _log_grip_key_selfcheck(self):
        """启动自检(P1):第一条 joint_states 到手即打印两指键名可解析性。

        sync_cap 的全部价值建立在"本臂两指键查得到"上;此前键名查不到
        是**静默**回退(读恒为基准值,dl/dr 恒 0,封顶永不触发,轮询白
        跑),从日志上与"物理上恰好双侧同步"无法区分。本自检把该事实
        强制打印,从此"静默失效"不可能:键查不到 = 当场在日志里明说。
        打印口径:查得到=<键名>(注明主表/别名表来源)或 查不到。"""
        jl, jr = self._finger_joint_keys()
        parts = []
        for tag, key in (('左', jl), ('右', jr)):
            if key in self._joints_now:
                parts.append(f'{tag}={key} 查得到(主表)')
            elif key in self._joints_alias:
                parts.append(f'{tag}={key} 查得到(别名表:裸名双份世界)')
            else:
                parts.append(f'{tag}={key} 查不到')
        line = '[自检] sync_cap 两指键名: ' + ' | '.join(parts)
        if any('查不到' in p for p in parts):
            self.get_logger().warn(
                line + ' —— sync_cap 将退回固定时长等待(非静默,明示)')
        else:
            self.get_logger().info(line)

    def _on_logical(self, msg: LogicalCameraImage):
        """逻辑相机观测 -> 真值 xy 缓存(只缓存 part_ 开头的模型名)。"""
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        for model in msg.model:
            if model.name.startswith('part_'):
                self._part_source_stamps[model.name] = stamp
                self._part_source_walls[model.name] = time.monotonic()
        self._cam_cache.update(
            (m.name, m.pose.position.x, m.pose.position.y, m.pose.position.z)
            for m in msg.model)

    def _pick_center(self, part: PartInfo) -> tuple[float, float, float]:
        """取件中心(世界系)。

        z 始终用 feed_z 常量:逻辑相机的深度分量有系统性偏差(见
        attach_helper 模块注释),来料都静置在同一平面。
        xy 默认取 PartInfo.pose;pose_from_camera=true 时改用逻辑相机
        真值缓存,无该件缓存时抛可重试的 GraspError('no_camera_pose')
        (相机话题稍后刷新即可命中,重试语义与规划失败一致)。
        """
        if not self._pose_from_camera:
            return (part.pose.position.x, part.pose.position.y, self._feed_z)
        true_xy = self._cam_cache.get(part.part_id)
        if true_xy is None:
            raise GraspError('no_camera_pose')
        return (true_xy[0], true_xy[1], self._feed_z)

    def _log_place_joints(self, bin_name: str):
        """诊断:打印放置完成时的臂关节角(用于标定关节空间放置目标)。"""
        vals = [round(self._joints_now.get(j, float('nan')), 4)
                for j in self._joint_names]
        self.get_logger().info(f'[标定] {bin_name} 放置关节角: {vals}')

    def _on_command(self, msg: PartInfo):
        if self._busy:
            self._pending.append(msg)
            self.get_logger().info(
                f'排队命令: 正在执行任务, {msg.part_id} 等待({len(self._pending)} 待处理)')
            return
        self._busy = True
        threading.Thread(target=self._run_task, args=(msg,), daemon=True).start()

    # ================================================================ 任务主流程
    def _run_task(self, part: PartInfo):
        self._zone_published = False
        try:
            self._run_task_impl(part)
        except Exception as e:   # noqa: BLE001 R1-P2(2026-10-04)
            # 终态发布保证:任务线程任何未捕获异常(含 _init_motion 的
            # RuntimeError 等)都必须落 FAILED 终态,否则调度器永等、
            # _busy 恒 True,整条流水线黑洞(forensics H1 定位:
            # 原 finally 里 return_standby 先于 flush,且 impl 异常时
            # _terminal_status 从未置位)。此处兜底,不掩盖 GraspError
            # (那已在 impl 内部落 FAILED)
            self.get_logger().error(f'{part.part_id} 执行器异常: {e!r}')
            if self._terminal_status is None:
                self._terminal_status = (
                    f'task_{self._seq:04d}', part, TaskState.FAILED,
                    f'executor_error({e})')
        finally:
            # 收尾:尽力回到待机位(失败不影响本件结果,只为下一件任务
            # 提供固定起点,使所有大转移都走 standby<->工作区的确定路径)
            self._try_return_standby()
            # 失败路径:没有走过 carry_clear。回到待命成功才算离开取件区。
            # FAILED 本身不放锁。已经在 carry_clear 放过的不再发。
            if self._at_standby:
                self._publish_zone_clear(part, 'return_standby')
            # 终态在回待机运动结束之后才发布:双臂共用一个 move_group 的
            # 轨迹执行器,"另一轨迹执行中"时新轨迹会被整体拒绝
            # (CONTROL_FAILED);提前报 COMPLETED 会让调度器在收尾轨迹
            # 还在跑时就派发下一棒,两臂请求重叠会卡死轨迹执行管理器
            try:
                self._flush_terminal_status()
            except Exception as e:   # noqa: BLE001 R1-P2:flush 自身异常
                self.get_logger().error(f'终态发布异常: {e!r}')
                self._terminal_status = None
            if self._pending:
                nxt = self._pending.pop(0)
                self.get_logger().info(f'继续排队任务: {nxt.part_id}')
                threading.Thread(target=self._run_task, args=(nxt,),
                                 daemon=True).start()
            else:
                self._busy = False   # 无排队任务,允许接收新命令

    def _try_return_standby(self):
        # 关节空间目标:直接命令回标定构型,不依赖 IK(位姿目标在个别
        # 分支上会无解)。motion 未初始化(_init_motion 失败/未执行)时
        # 无可回归,只告警——不能让 None 异常炸掉 finally 的终态发布
        if self._motion is None:
            self.get_logger().warn('motion 未初始化,跳过返回待机位')
            self._at_standby = False
            return
        # R1-P1:会话判死后收尾也不发目标(每个都 -4,纯拖时间)
        if (self._wedge_detect
                and getattr(self._motion, 'session_dead', False)):
            self.get_logger().warn('会话已判死,跳过返回待机位')
            self._at_standby = False
            return
        try:
            if not self._motion.move_to_joints(
                    self._joint_names, self._standby_joints,
                    'return_standby'):
                raise GraspError('return_standby_failed')
            self._at_standby = True
        except GraspError as e:
            self.get_logger().warn(f'返回待机位失败({e}),下一件任务将原地出发')
            self._at_standby = False

    def _run_task_impl(self, part: PartInfo):
        self._seq += 1
        task_id = f'task_{self._seq:04d}'
        self._active_task_id = task_id
        sm = TaskStateMachine()
        self.get_logger().info(
            f'== {task_id} 开始: {part.part_id} (类别 {part.category}) ==')
        self._init_motion()  # 首件任务时连接 move_group(阻塞至就绪)
        # R1-P1 快速失败:会话已判死时不在中毒会话内重试(每个目标都会
        # -4),直接落 FAILED 让外层停轮/受控重启,避免空转烧掉整轮读数。
        if (self._wedge_detect
                and getattr(self._motion, 'session_dead', False)):
            raise FatalGraspError('motion_session_wedged(会话未重启)')
        if not self._at_standby:
            # 先到待机位:所有工作段都从"工作区中央上方、竖直向下"的
            # 同族构型出发。跨大角度转移(如料区->下一来料位)的 PTP 会因
            # IK 分支漂移间歇性拒解;固定起点后 Pilz PTP 路径确定可复现。
            # 失败仅告警不中断(首段大转移偶发拒解,后续短段仍可成功)
            try:
                self._plan_execute(
                    self._down_pose(self._standby, self._standby[2]), 'standby')
            except GraspError as e:
                self.get_logger().warn(f'待机位运动失败({e}),继续任务')
            self._at_standby = True
        for attempt in range(1, self._attempts + 1):
            attached_joint = None
            if attempt > 1:
                sm.reset()   # 重试 = 新一轮状态周期(IDLE 重新出发)
            try:
                # 放置目标箱:类别映射 + 命令级覆盖(2026-10-06 指定箱
                # 派发)。抓取段与 target 无关(锚表路径不变),仅放置段
                # 使用本结果;覆盖箱不在本臂 bin_names 内 -> Fatal,
                # 走既有 FAILED 通道
                target = self._bin_for(
                    part.category,
                    getattr(part, 'target_bin_override', ''))
                if attempt == 1 and part.part_id in self._inject:
                    self.get_logger().warn(
                        f'{part.part_id}: 注入一次抓取失败(演示失败恢复)')
                    time.sleep(0.5)
                    raise GraspError('injected_failure')

                # ---------------- PICKING ----------------
                self._set_state(sm, task_id, part, TaskState.PICKING, '')
                # 取件中心:z 用 feed_z 常量(相机深度分量不可信);xy 默认
                # 取 PartInfo,pose_from_camera=true 时以逻辑相机真值缓存为准
                center = self._pick_center(part)
                # 取件侧锚定(可选,按目标 xy 就近选):跨侧取件首段位姿
                # PTP 有 KDL 分支翻转风险,先以确定性关节目标贴到取件区
                # 上方,再小平移+短距下探
                anchor = min(
                    self._pick_anchors,
                    key=lambda a: (a[0][0] - center[0]) ** 2
                    + (a[0][1] - center[1]) ** 2,
                    default=None) if self._pick_anchors else None
                if anchor is not None and not self._motion.move_to_joints(
                        self._joint_names, anchor[1], 'pick_anchor'):
                    raise GraspError('pick_anchor_failed')
                grasp = min(
                    self._pick_grasps,
                    key=lambda a: (a[0][0] - center[0]) ** 2
                    + (a[0][1] - center[1]) ** 2,
                    default=None) if self._pick_grasps else None
                # 下探收窄(§十六,用户拍板 2026-10-08 晚):在 pre_grasp
                # 高度(高于带面)先把开口收到 82mm 再入带——全开外缘
                # 0.078 距带板侧缘 0.080 仅 2mm,实测一侧指被顶在带缘
                # 上钉死(预收口起点即 L=-0.003/R=+0.040,双臂 100% 复
                # 现);内缘 0.041 对工件面 0.025 余 16mm,容到位闸 20mm
                # 误差。gap=0 禁用(legacy 零行为差)。
                self._close_to_gap('下探收窄', self._descend_gap)
                if grasp is not None:
                    # 取件锚命中:PTP 直达悬停下沿(0.99,det_ik 预解),
                    # 再接短 LIN 下探贴脸 —— 替代在线 IK 的长 pre_grasp+
                    # descend(arm2 的 LIN 在 descend_2 卡死,2026-10-03)。
                    # 锚 PTP 直达 0.953 深度会带 w1 重力滞后 ~0.3rad →
                    # tool0 偏 3.5cm 夹空(同日实证),LIN 贴脸段修正
                    if not self._motion.move_to_joints(
                            self._joint_names, grasp[1], 'pick_grasp'):
                        raise GraspError('pick_grasp_failed')
                    self._descend(center, center[2] + self._tcp_z,
                                  linear=self._pick_descend_linear)
                else:
                    self._plan_execute(self._down_pose(
                        center, center[2] + self._tcp_z + self._pre_h),
                        'pre_grasp')
                    self._descend(center, center[2] + self._tcp_z,
                                  linear=self._pick_descend_linear)
                # 到位闸:闭爪前确认真的下探到位(滞后闭爪=夹空)
                self._wait_tool_near(center[:2], center[2] + self._tcp_z,
                                     'pick_settle')
                # 实测对中(§十六):到位闸只保证"大致到"(tol=20mm),
                # 对 6mm 的指-件间隙太松;锚 PTP 的重力滞后实测 20-30mm
                # ⇒ 单指压件钉死。用相机最新真值实测偏心,带外平移修正
                # 后重下探(tol=0 禁用,legacy 零行为差)
                center = self._recenter_over_part(part, center)
                if self._attach_follow == 'verified_only':
                    # 闭爪前开始采样 -> 物理闭夹 -> 抬升 -> 水平验证探针
                    # -> 追加采样 verify_sample_span 秒 -> GraspVerifier
                    # 验证(契约 §8 + 组长仲裁方案 B)。验证通过后保持
                    # 纯物理夹持搬运(高摩擦指面 + effort 闭爪力),
                    # **不再 attach**:set_pose 跟随会把钉住的工件反复怼进
                    # 物理指/路径障碍,双重约束对抗中工件被挤落(联调实证
                    # 工件掉在 carry 中途),物理夹持一旦验证成立,attach
                    # 兜底有害无益
                    sampler = self._start_follow_sampler(part.part_id)
                    try:
                        # 闭爪抬深(§十六终案定罪,同步遥测铁证):标称
                        # 取件深度指尖 0.778 扎进带顶 0.780 达 2mm,闭爪
                        # 时被带面 μ=2.0 摩擦冻结在任意行程位(实录
                        # +0.0242 双向 ±8N 拉不动、抬臂离接触瞬间弹回),
                        # 对侧独走 → 间隙 74mm 假夹持。抬 12mm:指尖
                        # 0.790 离带净空 10mm,指带 0.790-0.842 对工件
                        # 侧面 0.780-0.830 仍重叠 40mm,冻结消失;遥测
                        # 实证两指对称合拢 L≈R≈-0.001、间隙=50mm 真咬件。
                        # 0 = 禁用(legacy 零行为差)。
                        if self._close_lift > 0.0:
                            self._plan_execute(self._down_pose(
                                center, center[2] + self._tcp_z
                                + self._close_lift), 'close_lift',
                                linear=True)
                        # 预收口(2026-10-08 晚,§十六):先把 132mm 全开
                        # 收到 62mm,轻触段扫掠只剩 ~6mm,消掉"单指长
                        # 距离推楔工件"的钉死构型;参数 0 = 禁用
                        self._close_to_gap('预收口')
                        # 两段闭爪:轻触(-1.5N,~0.1m/s 无冲击接触)再锁紧
                        self._grip(close=True, force=-1.5, wait=1.2, sync_cap=True)
                        self._grip(close=True, force=self._gripper_closed,
                                   wait=0.6)
                        # P2-2a 时序修正(2026-10-08):旧版锁紧段只等
                        # 固定 0.6s 即抬臂,观感"夹爪没合拢臂先起"。现
                        # 抬臂前加指关节停稳闸(两指 0.1s 窗位移均 <2mm,
                        # 软闸:超时告警放行,零新增失败路径)——观感变
                        # 成"合拢→停稳→抬起"。
                        self._wait_fingers_settled('锁紧段')
                        # 用户拍板追加(2026-10-08 晚):停稳≠合拢——钉死
                        # 态(一指卡全开限位+一指撞到底,两指都静止)曾被
                        # 停稳闸放行,抬臂时间隙 89mm 零夹持。夹持确认闸:
                        # 两指内缘必须真咬住工件(间隙≤55mm)才准抬臂,
                        # 不合拢补锁紧×2,再不合判失败走重试链。探针/验证
                        # /在场闸判据零改动,attach 兜底保留。
                        self._confirm_clamped('锁紧段')
                        self._lift(center, center[2] + self._tcp_z)
                        self._verify_probe(center)
                        t_wait = time.monotonic(); sim_wait = self.get_clock().now().nanoseconds/1e9
                        evidence = False
                        if self._fast_waits:
                            evidence = sampler.wait_evidence(self._verify_sample_span, self._verifier.verify)
                        else:
                            time.sleep(self._verify_sample_span)
                        self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'grasp_extra','wall':time.monotonic()-t_wait,'sim_start':sim_wait,'sim_end':self.get_clock().now().nanoseconds/1e9,'fast':self._fast_waits,'evidence':evidence,'pairs':len(sampler._gripper),'sample_wall_span':sampler._sample_times[-1]-sampler._sample_times[0] if len(sampler._sample_times)>1 else 0.,'distinct_source_frames':len(set(sampler._source_stamps))}))
                    finally:
                        gripper_xy, part_xy = sampler.stop()
                    ok, reason = self._verify_follow(gripper_xy, part_xy)
                    if not ok:
                        raise GraspError('grasp_not_verified')
                    # 在场闸:探针后工件可能已脱爪落回带面(探针撞挡板/
                    # 夹持滑脱),following 用的是历史采样无从分辨;attach
                    # 前查最新真值 xy 与夹爪 xy 距离,>3cm 视为不在指下,
                    # 拒绝 attach 走重试(否则会把带上的工件钉在指外随臂
                    # 横扫路径障碍,联调实证工件被挤落到 carry 中途)
                    last_gripper = self._gripper_world_xy()
                    last_part_xy = self._cam_cache.get(part.part_id)
                    if (last_gripper is None or last_part_xy is None
                            or math.hypot(last_gripper[0] - last_part_xy[0],
                                          last_gripper[1] - last_part_xy[1])
                            > 0.03):
                        raise GraspError('part_not_under_gripper')
                    # 验证通过且工件在指下,attach 兜底搬运(2026-10-01
                    # 联调:纯物理夹持大机动在 dartsim 会甩脱,attach 保证
                    # 搬运;此刻工件位姿与物理夹持一致,不构成对抗)
                    # 附着失真修复(2026-10-09):z 锚**实测**夹爪 TF 而非
                    # 命令位姿——P 控制抬升滞后 2-3cm,按命令值记录 rel,
                    # 跟随线程第一拍就把工件瞬移补上这段差(肉眼"咔哒
                    # 一跳")。夹持刚性已由闭爪闸+跟随验证背书:工件中心
                    # = 夹爪基座 z - tcp_z(竖直向)。TF 不可用回落旧值。
                    try:
                        tf = self._attach._tf.lookup_transform(
                            'world', self._attach_link, rclpy.time.Time())
                        attach_z = tf.transform.translation.z - self._tcp_z
                    except Exception:  # noqa: BLE001
                        attach_z = center[2] + self._lift_h
                    if not self._attach.attach(
                            part.part_id, self._attach_link,
                            self._down_pose(center[:2], attach_z).pose):
                        raise GraspError('attach_failed')
                    attached_joint = part.part_id
                else:
                    # always(默认):抓取即 attach(运动学跟随),旧行为
                    self._grip(close=True)
                    if not self._attach.attach(part.part_id, self._attach_link,
                                               self._down_pose(center,
                                                               center[2]).pose):
                        raise GraspError('attach_failed')
                    attached_joint = part.part_id
                    self._lift(center, center[2] + self._tcp_z)

                # ---------------- CARRYING ----------------
                self._set_state(sm, task_id, part, TaskState.CARRYING, '')
                place = self._place_pose(target)
                self.get_logger().info('POSE_TARGET '+json.dumps({'part':part.part_id,'task':task_id,'bin':target,'xyz':list(place)}))
                # 落入模式下松爪高度(clearance)可能高于抬升高度(lift_h):
                # above 取二者较大值,保证 carry_descend 不低于松爪点
                above = (place[0], place[1],
                         place[2] + self._tcp_z
                         + max(self._lift_h, self._place_drop_clearance))
                # 取点上方先升到 carry 高度。有标定关节时,再在同一点
                # 竖直抬高 0.10 m,然后才 PTP 到箱格。不从取件构型直接
                # 斜插:2026-10-15 part_6 蓝那样插过去时仿真时间几乎停住。
                # 不改 carry_safe_z,松爪高度仍是原来的巡航高度。
                # 没有关节目标时仍先回待命,再走原来的笛卡尔两段。
                # 待命关节数值本身不改,放下之后的回待命也不改。
                self._plan_execute(
                    self._down_pose(center[:2], self._carry_z), 'carry_rise')
                # 放置段首选关节空间直达(契约"放置构型关节角,跨侧横移后
                # KDL 位姿求解有分支随机性,放置段改用关节空间直达"):
                # place_joints_<bin> 有非 NaN 标定值时直接 PTP 到该构型,
                # 跳过 Cartesian 腿(2026-10-01 联调:Pilz LIN 到
                # (0.62,±0.16,1.05) 的解算终点病态停半路,控制器保持错
                # 终点,物理臂冻结在 (0.472,0.157,1.196) 30s 不动)
                place_joints = self._place_joints.get(target)
                use_place_joints = (place_joints is not None
                                    and not any(
                                        math.isnan(v) for v in place_joints))
                carry_anchor = min(
                    self._carry_anchors,
                    key=lambda a: (a[0][0] - above[0]) ** 2
                    + (a[0][1] - above[1]) ** 2,
                    default=None) if self._carry_anchors else None
                if use_place_joints or carry_anchor is not None:
                    # 转向前先竖直离开取件高度。0.10 m 是在 carry_safe_z
                    # 之上再抬的一段,流水线里 tool0 从 1.08 到 1.18。
                    self._plan_execute(
                        self._down_pose(center[:2], self._carry_z + 0.10),
                        'carry_clear', linear=True)
                    # 竖直抬高结束。手还在取件点正上方,下一句才转向箱格。
                    # 这是交接里的离区边界,不是 CARRYING 那一条状态。
                    self._publish_zone_clear(part, 'carry_clear')
                    goal_joints = (place_joints if use_place_joints
                                   else carry_anchor[1])
                    start_joints = [
                        round(self._joints_now.get(j, float('nan')), 4)
                        for j in self._joint_names]
                    self.get_logger().info(
                        'carry: 竖直抬高后到箱格关节,跳过待命 '
                        f'起始 {start_joints} 目标 '
                        f'{[round(float(v), 4) for v in goal_joints]}')
                elif not self._motion.move_to_joints(
                        self._joint_names, self._standby_joints,
                        'carry_anchor'):
                    raise GraspError('carry_anchor_failed')
                # 放置侧锚定(可选):近基座放置格上 KDL 会确定性选到
                # 臂身翻转族,先以关节目标贴到放置区上方再小平移。
                # 两份参数表里这组关节是 NaN,流水线不走这里。
                if self._place_anchor is not None and not (
                        self._motion.move_to_joints(
                            self._joint_names, self._place_anchor,
                            'place_anchor')):
                    raise GraspError('place_anchor_failed')
                if use_place_joints:
                    # 关节空间直达放置构型(见上,旧世界 6/6 验证标定值)
                    if not self._motion.move_to_joints(
                            self._joint_names, place_joints,
                            'carry_place_joints'):
                        raise GraspError('carry_place_joints_failed')
                elif carry_anchor is not None:
                    # 关节空间直达放置格上方:lift goal 容差 0.05 下
                    # goal 保持 pending,JTC 持续命令末点,臂收敛到
                    # 末点-静态下垂 = 格子构型(anchor 已含 +0.2306 前置
                    # 补偿);到位闸在收敛点(误差 0.4612<0.55)早退成功
                    if not self._motion.move_to_joints(
                            self._joint_names, carry_anchor[1],
                            'carry_anchor2'):
                        raise GraspError('carry_anchor2_failed')
                else:
                    # 两段短 LIN 绕行(2026-10-01 联调:单段 standby->箱的
                    # PTP 会被 KDL 随机重启选中低扫分支,规划场景确定拒掉;
                    # x=0.50 高位竖直向下在近基座不可达袋)。路线:先在待机
                    # 列 x=0.35 走 y(该列全程已验证可达),再走 x 到箱上方;
                    # y 段经过带上方时 part 底 0.817/指尖 0.819 均在带顶
                    # 0.785 之上,远离相机立柱(0.65,0)
                    wp_x = self._standby[0]
                    self._plan_execute(
                        self._down_pose((wp_x, above[1]), self._carry_z),
                        'carry_leg1', linear=True)
                    self._plan_execute(
                        self._down_pose((above[0], above[1]), self._carry_z),
                        'carry_leg2', linear=True)
                    # [标定] 记录成功的 carry 构型,供回填 carry_anchor
                    vals = [round(self._joints_now.get(j, float('nan')), 4)
                            for j in self._joint_names]
                    self.get_logger().info(
                        f'[标定] {target} carry 关节角: {vals}')
                if self._place_drop_clearance > 0.0:
                    # 落入模式:carry_z(1.05)直接松爪 —— 指尖 0.819 高出
                    # 箱壁顶 0.785 约 3.4cm,工件自由落入;不再下探/绕行
                    # (2026-10-01 联调:carry_z 附近的短 LIN 会被规划场景
                    # 保守拒掉,而松爪点本身无需更近)
                    pass
                else:
                    self._plan_execute(self._down_pose(above, above[2]),
                                       'carry_descend')

                # ---------------- PLACING ----------------
                self._set_state(sm, task_id, part, TaskState.PLACING, '')
                # 下探终点:place_drop_clearance>0 时抬高该间隙(落入模式,
                # 契约 §9);<=0 与旧实现一致(下探到放置面)。
                # 收尾抬升从同一终点起算,=0 时与旧行为完全一致
                place_end_z = place_descend_target_z(
                    place[2], self._tcp_z, self._place_drop_clearance)
                if self._place_drop_clearance > 0.0:
                    # 落入模式已在 carry_z 松爪(见上),这里无运动
                    pass
                else:
                    self._descend(above[:2], place_end_z)
                # 到位闸:open_loop JTC 报"完成"时物理可能滞后,松爪前须
                # 确认 tool0 真的到了释放点(滞后释放=工件被水平抛出,联调
                # 实证落点偏 9.8cm);超时按可重试失败处理。
                # 闸目标 = 实际松爪高度:落入模式在 carry_z 松爪(见上),
                # legacy 在 place_end_z(队员C 发现1:盯错目标必超时)
                release_z = (self._carry_z if self._place_drop_clearance > 0.0
                             else place_end_z)
                # 低增益(0.1)下大摆动段的指数收敛 tau≈10s,肩部滞后
                # 0.6rad 收敛到 2cm 需 ~25s:释放闸超时 30s(2026-10-01)
                # tol 0.05:带载下垂(增益 1.0 下 ~3-4cm)不可避免,闸只
                # 守"大致到箱口上方";落点精度由箱口内口(±0.06/±0.085)
                # 与松爪高度自由落体的四壁约束兜底
                self._wait_tool_near(above[:2], release_z, 'place_release',
                                     timeout=30.0, tol=0.05)
                settle_pair = None
                if carry_anchor is not None:
                    settle_pair = self._settle_pairs.get(
                        (round(carry_anchor[0][0], 3), round(carry_anchor[0][1], 3)))
                if settle_pair is not None and self._place_drop_clearance > 0.0:
                    # 放入式放置(set-down):落入式松爪的高位自由落体
                    # (~10cm)叠加指面摩擦横向喷挤(2026-10-03 监视器实证
                    # 0.46-1.66m/s 定向弹射,轻重开都避不开 stick-slip)
                    # 是 OUTSIDE/MISPLACED 的共同根因。改为:关节 PTP 下探
                    # 到工件触底(梯子逐级加深,消除 P 控制下垂不确定),
                    # 工件受箱底支撑后松爪——支撑状态下指面分离的摩擦
                    # 无横向分量,无弹射。指杆 12x12mm,入箱口对壁余量
                    # 24mm、对邻格件角线干涉 ≤1mm(慢速轻推可容忍)。
                    # 放入式放置(2026-10-03 终案):梯子 4 级下探到真触底
                    # (tool0 0.954 = 工件底 0.720 = 台面;深折构型零下垂,
                    # 命令即到位)→ detach(工件已受台面支撑,解除跟随无
                    # 对抗)→ 卸荷(闭爪 -8N 法向归零,mu=2 摩擦耦合消失)
                    # → 慢开(1.5N,近零法向下接触十余毫秒即断)→ 全开。
                    # 前史(同日排查,均为世界台面 bug 下的 36cm 深井坠落
                    # 所致,台面修复后失效):快开弹射/跟随-接触战争/
                    # 纯物理楔入/下滑摩擦转向。
                    if not self._settle_ladder(carry_anchor, 0, 3):
                        raise GraspError('settle_touch_failed')
                    # 触底软闸:个别格(B 箱 +y 格 6)带载家族屏障高于
                    # 触底线,梯子停在屏障上(格 6 已改高位锚,其余格
                    # 正常触底)——悬停高度释放同样成立(落差 cm 级 +
                    # 干净释放),告警继续;只有近乎没下探才算真失败。
                    # 失速不复盘为异常:失速级的 settle-rescue 取消可
                    # 触发 move_group rcl_action 取消竞态崩溃(2026-10-03
                    # 第 3 轮实证),软闸避免重试链放大。
                    z_end = self._tool_z_now()
                    if z_end is not None and z_end > 1.09:
                        raise GraspError(f'settle_stalled_high(z={z_end:.3f})')
                    if z_end is not None and z_end > self._settle_touch_z:
                        self.get_logger().warn(
                            f'梯子终点 z={z_end:.3f} 高于触底线 '
                            f'{self._settle_touch_z},按悬停释放继续')
                    self._attach.detach()
                    self._grip(close=False, force=0.0, wait=0.6)
                    # 观感修复②(2026-10-08):旧序列"1.5N 慢开 1.2s →
                    # 8N 全开"在**箱底**就把两指推到 +0.04 全张限位
                    # (阻尼 15 下 1.5N 末速 0.1m/s,1.2s 绰绰有余)——
                    # 外格格心 y 偏 ±0.05(锚表定值,红线),全开指外缘
                    # 到箱心 0.128 > 壁内缘 0.085,插进壁带 43mm,随后
                    # 张着满爪穿过 95mm 壁高抬臂 = 肉眼"顶住盒沿刮擦"。
                    # 新序列:位置闸小开到刚脱件(q≈0.001,与壁带重叠
                    # 仅 4mm 且随 0N 内收收窄)→ 保持小开抬臂出箱 →
                    # **箱沿之上**才全开(见 settle_rise 之后)。
                    self._open_to_release_gap()
                    t_wait = time.monotonic()
                    time.sleep(0.4)  # retained: gap alone does not prove settled release
                    self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'release_settle','wall':time.monotonic()-t_wait,'fast':self._fast_waits}))
                    # 原路抬升回同格高位 anchor(逆向下探的已验证路径),
                    # 再走既有 return_standby。抬升失败不否定本件结果:
                    # 工件已释放到位,失败只影响下一件起始构型(收尾的
                    # return_standby 会再尝试;move_group 偶发取消竞态
                    # 崩溃时重试只会拖时间,2026-10-03 实证)
                    try:
                        if not self._motion.move_to_joints(
                                self._joint_names, carry_anchor[1],
                                'settle_rise'):
                            raise GraspError('settle_rise_failed')
                    except GraspError as e:
                        self.get_logger().warn(
                            f'{task_id} 放置后抬升失败({e}),不影响本件结果')
                    # 观感修复②后半:此刻指尖整体已出箱沿(carry 高位
                    # anchor,FK 实测指尖 0.819 起、指身向上 60mm,均高于
                    # 壁顶 0.815)才全开 —— 旧序列"箱底全开"正是插壁带
                    # 43mm 的来源。全开后 _grip 解除位置保持,交回
                    # 常值重发(①),下一件取件前指在开限位(确定性)。
                    self._grip(close=False)
                else:
                    self._grip(close=False)
                    self._attach.detach()
                if self._place_drop_clearance > 0.0 and settle_pair is None:
                    # 落入模式(无 settle 标定时):等工件落底稳定再回待机——
                    # 松爪后立即摆臂会让指尖扫到下落中的工件(2026-10-02 实证)。
                    # 撤退路线实验记录(均 2026-10-02):x=0.12 侧扫 LIN 被
                    # Pilz 拒(近基座 pan 转角过大);待机列横扫 LIN@1.05
                    # 指尖距壁顶仅 4mm、@1.08 也仅 3.4cm,间歇性蹭壁会让
                    # 指头栽进箱口把已放工件踢飞 15-30cm;@1.10 LIN 终点
                    # 稳定误差超位姿闸 2.5cm 反而 60s 挂。终案 = 直接
                    # return_standby 关节 PTP(FK 实测全程工件底 ≥0.830,
                    # 高于壁顶 0.815)。
                    t_wait = time.monotonic()
                    time.sleep(1.5)
                    self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'drop_return_branch','wall':time.monotonic()-t_wait,'fast':self._fast_waits}))
                    # 全开补齐:保证下一件取件前指在开限位(状态确定性)
                    self._grip(close=False)
                if self._place_drop_clearance <= 0.0:
                    # 收尾抬升只影响下一件循环的起始状态,失败不否定本件
                    # 结果;落入模式跳过(指尖已高出箱壁顶,再抬会踩可达墙
                    # NO_IK,2026-10-01 联调)
                    try:
                        self._lift(place[:2], place_end_z)
                    except GraspError as e:
                        self.get_logger().warn(
                            f'{task_id} 放置后收尾抬升失败({e}),不影响本件结果')
                    # 全开补齐(两段松爪的后段,工件已落入箱内)
                    self._grip(close=False)

                self._place_counts[target] = self._place_counts.get(target, 0) + 1
                self._log_place_joints(target)
                self._set_state(sm, task_id, part, TaskState.COMPLETED, target)
                return
            except FatalGraspError as e:
                self._recover(attached_joint)
                self.get_logger().error(f'{task_id} 失败(不重试): {e}')
                self._set_state(sm, task_id, part, TaskState.FAILED, str(e))
                return
            except GraspError as e:
                self._recover(attached_joint)
                # R1-P1:会话判死 ⇒ 不烧重试链(中毒会话内任何目标都只会
                # -4),当件直接落 FAILED,motion_session_wedged 标记供
                # 外层(Lead 轮运行器)判死停轮 + 受控重启 move_group。
                if (self._wedge_detect
                        and getattr(self._motion, 'session_dead', False)):
                    self.get_logger().error(
                        f'{task_id} 会话楔死判死(不重试): {e}')
                    self._set_state(sm, task_id, part, TaskState.FAILED,
                                    'motion_session_wedged')
                    return
                if attempt >= self._attempts:
                    self.get_logger().error(f'{task_id} 重试耗尽: {e}')
                    self._set_state(sm, task_id, part, TaskState.FAILED, str(e))
                    return
                self.get_logger().warn(
                    f'{task_id} 第 {attempt} 次尝试失败({e}),恢复后重试')
            except IllegalTransition as e:
                self.get_logger().error(f'{task_id} 状态机异常: {e}')
                return

    def _bin_for(self, category: str, override: str = '') -> str:
        # 指定箱派发(2026-10-06):override 非空时放置目标 = override
        # (须在本臂 _bins 内),抓取路径不受影响;空 = 既有类别映射,
        # 未知类别/越臂箱名均 FatalGraspError 直接失败(旧行为不变)
        try:
            return resolve_target_bin(category, override,
                                      self._color_to_bin, self._bins)
        except UnknownTargetBin as e:
            raise FatalGraspError(str(e))

    def _set_state(self, sm: TaskStateMachine, task_id: str, part: PartInfo,
                   state: TaskState, detail: str):
        sm.advance(state)
        if state in (TaskState.COMPLETED, TaskState.FAILED):
            # 终态暂存,待回待机运动结束后统一发布(见 _run_task 注释)
            self._terminal_status = (task_id, part, state, detail)
            self.get_logger().info(
                f'{task_id} {part.part_id}: {state.value} {detail}'
                f'(待收尾后上报)')
            return
        self._publish_status(task_id, part, state, detail)

    def _flush_terminal_status(self):
        if self._terminal_status is None:
            return
        task_id, part, state, detail = self._terminal_status
        self._terminal_status = None
        self._publish_status(task_id, part, state, detail)

    def _publish_zone_clear(self, part: PartInfo, reason: str):
        """通知调度器取件区可以进下一台臂。不改状态机,也不结束本臂的槽。"""
        if self._zone_published:
            return
        self._zone_published = True
        task_id = getattr(self, '_active_task_id', '') or f'task_{self._seq:04d}'
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.part_id = part.part_id
        msg.state = 'ZONE_CLEAR'
        msg.detail = reason
        msg.arm = self._arm_name
        self._pub_status.publish(msg)
        self.get_logger().info(
            f'{task_id} {part.part_id}: ZONE_CLEAR {reason}')

    def _publish_status(self, task_id: str, part: PartInfo,
                        state: TaskState, detail: str):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = task_id
        msg.part_id = part.part_id
        msg.state = state.value
        msg.detail = detail
        msg.arm = self._arm_name
        self._pub_status.publish(msg)
        self.get_logger().info(f'{task_id} {part.part_id}: {state.value} {detail}')

    # ================================================================ 运动接口
    def _init_motion(self):
        if self._motion is not None:
            # R1-P3 re-arm 钩子(G-E):上一件判死后连接已作废;外层受控
            # 重启 move_group 后,新任务丢弃旧实例重建(旧 ActionClient
            # 句柄指向死亡服务端,不重建则永远连不上)。
            if (self._wedge_rearm
                    and getattr(self._motion, 'session_dead', False)):
                self.get_logger().info(
                    'wedge_rearm: 丢弃已判死的运动连接,重连 move_group')
                self._motion = None
            else:
                return
        self.get_logger().info('连接 move_group 动作服务器 ...')
        try:
            self._motion = MotionClient(
                self, self._group, self._tool_frame, self._planning_frame,
                vel_scale=self._vel_scale, acc_scale=self._acc_scale,
                wedge_detect=self._wedge_detect,
                tool_settle_fix=self._tool_settle_fix,
                split_plan_execute=self._split_plan_execute,
                controller_action=self._controller_action,
                fast_sim_waits=self._fast_waits)
        except Exception as e:   # noqa: BLE001 R1-P2(2026-10-04)
            # 构造失败保持 _motion=None 并转为可重试 GraspError:
            # 原样上抛 RuntimeError 会在 _run_task 的兜底层落 FAILED,
            # 下一件任务重新尝试连接(move_group 复活即自愈);
            # 原实现异常穿到 finally 的 return_standby 会 AttributeError
            # 炸掉终态发布(全链黑洞,forensics H1)
            self._motion = None
            raise GraspError(f'motion_init_failed({e})') from e

    def _plan_execute(self, pose_stamped, label: str, linear: bool = False):
        """规划到指定位姿并执行(阻塞,失败抛 GraspError)。"""
        if not self._motion.move_to_pose(pose_stamped, label, linear=linear):
            raise GraspError(f'{label}_failed')

    def _down_pose(self, center_xy_z, z: float):
        """给定目标点(世界系)与 tool0 期望高度,生成竖直向下的工具位姿。"""
        pose = PoseStamped()
        pose.header.frame_id = self._planning_frame
        pose.pose.position.x = center_xy_z[0]
        pose.pose.position.y = center_xy_z[1]
        pose.pose.position.z = z
        qx, qy, qz, qw = _downward_quat(self._yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _descend(self, center_xy_z, target_z: float, linear: bool = True):
        """分步下探(默认 LIN 笛卡尔直线,保证竖直轨迹)。

        linear=False 用 PTP:远距取件点(近基座可达边缘)的 LIN 路径采样
        会让 KDL 中途跳 IK 分支(臂身翻到推挤工件的构型);PTP 只在目标处
        做一次 IK,从锚定种子出发稳定。代价是进近带少量弧线偏移 ——
        取件时手指张开(开口 132mm 对 50mm 工件),±3cm 横漂完全可接受;
        放置段(手持工件,不允许横漂)必须保持 LIN。
        """
        start_z = target_z + self._pre_h
        for i in range(1, self._steps + 1):
            z = start_z + (target_z - start_z) * i / self._steps
            self._plan_execute(self._down_pose(center_xy_z, z),
                               f'descend_{i}/{self._steps}', linear=linear)

    def _lift(self, center_xy, from_z: float):
        # lift_rungs>1 时分多级小步爬升(浅折带载构型 P 控制稳态误差
        # ~0.4 rad,一步 lift_height(6cm)直接失速原地不动(gain 2.0
        # 撑不起 ~0.8N·m,2026-10-03 arm2 relay 实证 gripper_not_moved);
        # 小步逐级建力矩爬出,同 arm1 家族屏障的爬梯思路
        if self._lift_rungs <= 1:
            self._plan_execute(
                self._down_pose(center_xy, from_z + self._lift_h), 'lift',
                linear=True)
            return
        step = self._lift_h / self._lift_rungs
        for i in range(self._lift_rungs):
            self._plan_execute(
                self._down_pose(center_xy, from_z + step * (i + 1)),
                f'lift_{i + 1}/{self._lift_rungs}', linear=True)

    def _place_pose(self, bin_name: str):
        idx = self._place_counts.get(bin_name, 0) % len(self._grid_x)
        x, y, z = self._bins[bin_name]
        return (x + self._grid_x[idx], y + self._grid_y[idx], z)

    def _log_gripper_tf(self, tag: str):
        """诊断:打印夹爪附着链接的当前世界位姿(TF)。"""
        try:
            from tf2_ros import TransformException
            tf = self._attach._tf.lookup_transform(
                'world', self._attach_link, rclpy.time.Time())
            t = tf.transform.translation
            self.get_logger().info(
                f'[诊断] {tag} {self._attach_link} 实际位置 '
                f'({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
        except Exception as e:  # noqa: BLE001 诊断专用
            self.get_logger().warn(f'[诊断] {tag} TF 不可用: {e}')

    # ================================================================ 夹取验证
    def _verify_probe(self, center):
        """水平验证探针(组长仲裁方案 B):抬升终点高度的 +x 往返 LIN。

        纯竖直抬升的夹爪 xy 位移不足以开验证窗(gripper_not_moved),
        探针给出 > verify_gripper_move 的水平位移:夹住的工件跟随
        (part_move >= follow_move_thresh 且方向一致)判通过,没夹住的
        留在原地判失败。短 LIN 的关节加/减速度需求 ∝ 1/时长²,若踩
        Pilz 5.0 rad/s² 限值被拒,降 vel_scale/acc_scale(既有参数)。
        探针规划/执行失败按可重试的 GraspError('verify_probe_failed')
        处理。verify_probe_offset<=0 时禁用(不产生任何运动)。
        """
        if self._verify_probe_offset <= 0.0:
            return
        if self._verify_probe_pan_delta != 0.0:
            # 关节空间 pan 弧探针:零在线 IK(见参数注释,pose 版在带尾
            # 深折构型上分支随机 ⇒ 60s 悬置 ⇒ -4 洪水,G3 r1c/r1d 实证)
            cur = [self._joints_now.get(j, float('nan'))
                   for j in self._joint_names]
            if any(math.isnan(v) for v in cur):
                raise GraspError('verify_probe_failed(joint_cache_empty)')
            d = self._verify_probe_pan_delta
            out = list(cur); out[0] += d
            back = list(cur)
            if not (self._motion.move_to_joints(
                    self._joint_names, out, 'verify_probe_out')
                    and self._motion.move_to_joints(
                        self._joint_names, back, 'verify_probe_back')):
                raise GraspError('verify_probe_failed(pan_arc)')
            return
        targets = verify_probe_targets(center, self._tcp_z, self._lift_h,
                                       self._verify_probe_offset)
        out, back = targets
        try:
            self._plan_execute(self._down_pose(out, out[2]),
                               'verify_probe_out',
                               linear=self._verify_probe_linear)
            self._plan_execute(self._down_pose(back, back[2]),
                               'verify_probe_back',
                               linear=self._verify_probe_linear)
        except GraspError as e:
            raise GraspError(f'verify_probe_failed({e})') from e

    def _wait_tool_near(self, target_xy, target_z, label: str,
                        tol: float = 0.02, timeout: float = 8.0,
                        speed_tol: float = 0.02):
        """到位闸:轮询 world→tool0 TF 直到距目标 < tol 且速度 < speed_tol。

        open_loop JTC 按命令时间报完成,物理滞后(负载/接触)时命令值与
        真实位姿脱节;松爪/闭爪这类物理作用点必须等臂真正到位**且停稳**
        (只查位置会让闭爪在臂漂移中把工件挤偏,2026-10-01 联调实证:
        扫帚效应——指面推着工件走,验证假通过、在场闸真拒绝)。速度判据
        = 0.1s 间隔两次 TF 位移折算速度 < speed_tol(2026-10-03 从 5mm/s 放宽到 20mm/s:gain 10 高刚度微振 ±3mm 使 5mm/s 永不满足;把关的取件/触底均为竖直动作,微振无侧扫)。"""
        deadline = time.monotonic() + timeout
        last = None
        trail = []
        while time.monotonic() < deadline:
            try:
                tf = self._attach._tf.lookup_transform(
                    'world', self._tool_frame, rclpy.time.Time())
                t = tf.transform.translation
                last = (t.x, t.y, t.z)
                trail.append(last)
                if math.dist(last, (target_xy[0], target_xy[1], target_z)) < tol:
                    if (len(trail) >= 2
                            and math.dist(trail[-1], trail[-2])
                            < speed_tol * 0.1):
                        return
            except Exception:  # noqa: BLE001 TF 暂不可用继续等
                pass
            time.sleep(0.1)
        # 诊断:命令目标 + 物理轨迹(首/中/尾 3 点)+ 关节状态新鲜度
        js_age = (time.time() - self._js_wall) if self._js_wall else None
        picks = (trail[0], trail[len(trail) // 2], trail[-1]) if trail else ()
        joints = {j: round(self._joints_now.get(j, float('nan')), 3)
                  for j in self._joint_names}
        self.get_logger().warn(
            f'[诊断] {label} 闸超时: 目标=({target_xy[0]:.3f},{target_xy[1]:.3f},'
            f'{target_z:.3f}) 物理首/中/尾='
            + ' '.join(f'({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})' for p in picks)
            + f' 采样数={len(trail)} joint_states年龄={js_age} 关节={joints}')
        raise GraspError(f'{label}_not_settled(last={last})')

    def _wait_fingers_settled(self, label: str, timeout: float = 2.0,
                              tol_m: float = 0.002, poll: float = 0.1):
        """指关节停稳闸(P2-2a):两指 0.1s 窗内位移均 <2mm 才放行抬臂。

        锁紧段固定 wait 只保证"命令发过",不保证指已停(偏心件被慢推
        进/微振时 0.6s 末指仍在走)。抬臂前把"合拢→停稳"补齐,观感即
        "夹爪闭实再起来"。判据走 fingers_settled 纯函数,取数与 sync_cap
        同路(_joint_lookup 双世界)。

        超时/键查不到 = **软闸**告警后放行(与旧行为等价的兜底,零新增
        失败路径;夹持真伪仍由随后的探针+验证+在场闸把关)。连续两次
        安静采样才算停稳(单次会漏过 0.1s 恰好处在过零点的抖动相位)。"""
        jl, jr = self._finger_joint_keys()
        prev_l = self._joint_lookup(jl)
        prev_r = self._joint_lookup(jr)
        if prev_l is None or prev_r is None:
            self.get_logger().warn(
                f'{label} 停稳闸:两指键查不到,跳过(软闸)')
            return False
        deadline = time.monotonic() + timeout
        quiet = 0
        while time.monotonic() < deadline:
            time.sleep(poll)
            cur_l = self._joint_lookup(jl)
            cur_r = self._joint_lookup(jr)
            if cur_l is None or cur_r is None:
                self.get_logger().warn(
                    f'{label} 停稳闸:轮询中键消失,跳过(软闸)')
                return False
            if fingers_settled(prev_l - cur_l, prev_r - cur_r, tol_m):
                quiet += 1
                if quiet >= 2:
                    self.get_logger().info(
                        f'[读数] {label} 两指停稳 L={cur_l:.4f} '
                        f'R={cur_r:.4f}')
                    return True
            else:
                quiet = 0
            prev_l, prev_r = cur_l, cur_r
        self.get_logger().warn(
            f'{label} 停稳闸超时 {timeout}s(两指仍在走,'
            f'L={prev_l:.4f} R={prev_r:.4f}),放行抬臂(软闸)')
        return False

    def _open_to_release_gap(self, release_gap: float = 0.052,
                             force: float = 1.5, timeout: float = 2.0):
        """脱件小开(观感修复②核心;2026-10-09 间隙口径重写)。

        夹爪只有力接口且**两指共享同一条命令**。旧口径"双指各自到
        release_q 后交棒位置保持"在实机 6/8 超时:一侧指被接触钉在
        闭位不动,bang-bang 只把自由侧一路推到 +0.040 全张限位(外缘
        插壁带 43mm,抬臂刮壁)。新口径:间隙 0.052+ql+qr ≥ 52mm(>
        工件 50mm)即判脱件,**立即转 0N 中立**——自由侧最多走到
        +0.005(外缘对壁重叠 ≤8mm),受阻侧自锁;0N 下两指以 ~1-3mm/s
        缓慢内收,重叠收窄归零,且此刻工件已 detach 静置箱底,0.03N
        级阻尼漂移拖不动 40g 工件,落点不受扰。

        返回 True=间隙达标;False=超时/键查不到(软闸,卸荷出箱,
        箱沿之上由 settle_rise 后的全开补齐)。"""
        jl, jr = self._finger_joint_keys()
        pl = self._joint_lookup(jl)
        pr = self._joint_lookup(jr)
        if pl is None or pr is None:
            self.get_logger().warn(
                f'释放闸:两指键查不到({jl}/{jr}),退回固定等待(软闸)')
            self._grip(close=False, force=force, wait=1.2)
            return False
        deadline = time.monotonic() + timeout
        ok = False
        while time.monotonic() < deadline:
            if fingers_released(pl, pr, release_gap):
                ok = True
                break
            self._pub_gripper_cmd(force)
            time.sleep(0.05)
            # 显式判 None(0.0 是合法关节位,不能用 `or` 兜底)
            cur_l = self._joint_lookup(jl)
            cur_r = self._joint_lookup(jr)
            if cur_l is None or cur_r is None:
                self.get_logger().warn(
                    f'释放闸:轮询中键消失({jl}/{jr}),按当前开度卸荷(软闸)')
                self._pub_gripper_cmd(0.0)
                return False
            pl, pr = cur_l, cur_r
        if ok:
            # 中立卸荷(不交棒 bang-bang 位置保持:共享命令下那会把
            # 受阻侧的"债"全打在自由侧行程上,即旧版狂奔的病根)
            self._grip_hold_target = None
            self._pub_gripper_cmd(0.0)
            self.get_logger().info(
                f'[读数] 释放小开到位 间隙={(0.052 + pl + pr) * 1000:.0f}mm'
                f'(L={pl:.4f} R={pr:.4f},≥{release_gap * 1000:.0f}mm,卸荷'
                f'出箱)')
        else:
            # 超时(实测主因:一侧指被接触钉在开限位,见观感② 深挖
            # 记录)。**不再**像旧序列那样在箱底强推全开——那正是把
            # 已张开的那指再往壁带里压、并把另一指也推到全开的动作。
            # 改为卸荷 0N:两指停在当前开度,抬臂全程跨度不再增大,
            # 出箱后由 settle_rise 之后的全开补齐(状态确定性不变)。
            self.get_logger().warn(
                f'释放小开超时 {timeout}s(L={pl:.4f} R={pr:.4f}),'
                f'卸荷保持当前开度出箱(软闸,箱沿之上再全开)')
            self._pub_gripper_cmd(0.0)
        return ok

    def _gripper_center_xy(self):
        """指跨中心世界 xy(优先 gripper_base,无 TF 时回落 tool0)。

        指跨中心 ≈ gripper_base 轴心(xacro 实证两指对称于 finger_j 原点,
        该原点即 base 系 y=0);gripper_base 在 tool0 前下方固定偏置,与
        下压朝向一起由同一 FK 保证。回落 tool0 时修正量含 ~5mm 常量偏置,
        容在 tol 内。"""
        for frame in (self._attach_link, self._tool_frame):
            try:
                tf = self._attach._tf.lookup_transform(
                    'world', frame, rclpy.time.Time())
                t = tf.transform.translation
                return (t.x, t.y)
            except Exception:  # noqa: BLE001 TF 未就绪试下一 frame
                continue
        return None

    def _recenter_over_part(self, part: PartInfo, center):
        """实测对中(§十六终案,用户拍板 2026-10-08 晚):下探到位后用
        相机最新真值与夹爪基座实测 xy 求偏心,超差则抬回悬停高、在带外
        平移修正、再下探一次。

        定罪(双臂 100% 复现):下探命令值与物理值差 20-30mm——锚 PTP
        直达的重力滞后(w1 ~0.3rad ⇒ 3.5cm,模块注释在案)+ 到位闸
        tol=20mm 放行,导致一侧指从下探起就压在工件面上;闭爪时该指
        推不动工件(带缘静摩擦),另一指独走到底后把工件横拖 ~25mm,
        最后工件楔死在两指之间(L=+0.0248 内缘恰落在工件远面 51mm)。
        修正走带外悬停平移(零扫掠),修正后重新收窄+下探+到位闸。
        tol=0 禁用(legacy 零行为差)。返回值 = 后续 lift/probe 应使用的
        参考中心(未修正时为原 center;修正后为 fixed,物理臂已移到
        fixed,继续用陈旧 center 会在 lift 把带件横拖)。"""
        if self._center_tol <= 0.0:
            return center
        true_xy = self._cam_cache.get(part.part_id)
        base_xy = self._gripper_center_xy()
        if true_xy is None or base_xy is None:
            self.get_logger().warn('对中:相机/TF 数据缺失,跳过(软闸)')
            return center
        off = (true_xy[0] - base_xy[0], true_xy[1] - base_xy[1])
        dist = math.hypot(*off)
        if dist <= self._center_tol:
            self.get_logger().info(
                f'[读数] 对中无需修正(偏心 {dist * 1000:.0f}mm ≤ '
                f'{self._center_tol * 1000:.0f}mm)')
            return center
        if dist > self._center_max:
            raise GraspError(f'pick_off_center({dist * 1000:.0f}mm)')
        # 指跨中心实测在 true_xy-off(=base),工件要落到 base+off 对正:
        # tool0 目标平移量 = +off(tool0→base 为常量偏置,平移同向)。
        # 保持三元组:后续 lift/probe/attach 仍读 center[2]。
        fixed = (center[0] + off[0], center[1] + off[1], center[2])
        self.get_logger().info(
            f'[读数] 实测偏心 {dist * 1000:.0f}mm(dx={off[0] * 1000:+.0f} '
            f'dy={off[1] * 1000:+.0f})>容差 {self._center_tol * 1000:.0f}mm,'
            f'悬停平移对中重下探')
        self._plan_execute(self._down_pose(
            fixed, center[2] + self._tcp_z + self._pre_h), 'recenter')
        self._close_to_gap('重下探收窄', self._descend_gap)
        self._descend(fixed, center[2] + self._tcp_z,
                      linear=self._pick_descend_linear)
        self._wait_tool_near(fixed[:2], center[2] + self._tcp_z,
                             'pick_settle2')
        # 后续 lift/probe/attach 的参考中心改用修正后的实测对中点:
        # 物理臂已在 fixed,若继续用陈旧 center,lift 会把带件横拖 ~2.6cm
        return fixed

    def _close_to_gap(self, tag: str, gap: float | None = None):
        """位置收口:把两指内缘间隙收到 ≤ 0.052+2*gap(gap=None 取预收口值)。

        两段收窄(用户拍板 2026-10-08 晚,§十六):
        - 下探收窄(gripper_descend_gap,0.015→间隙 82mm):全开外缘
          0.078 距带板侧缘 0.080 仅 2mm,wrist 下垂微旋即把一侧指顶在
          带缘上钉死(预收口起点 L=-0.003/R=+0.040 铁证)。进带前收口,
          带缘余量 2mm→27mm。
        - 预收口(gripper_pre_close_gap,0.005→间隙 62mm):轻触段单指
          空扫从 41mm 压到 ~6mm,消掉"单指长距推楔工件"构型。
        到位即卸荷(0N)保持,不再施力。gap≤0 = 禁用(legacy 零行为差);
        超时软放行(交给夹持确认闸兜底,不新增失败路径)。
        """
        if gap is None:
            gap = self._pre_close_gap
        if gap <= 0.0:
            return
        jl, jr = self._finger_joint_keys()
        target = 0.052 + 2 * gap
        deadline = time.monotonic() + self._pre_close_wait
        while True:
            pl, pr = self._joint_lookup(jl), self._joint_lookup(jr)
            if pl is None or pr is None:
                self.get_logger().warn(f'{tag}:两指键查不到,跳过')
                return
            if (0.052 + pl + pr) <= target:
                self._pub_gripper_cmd(0.0)  # 到位卸荷,防继续压工件
                self.get_logger().info(
                    f'[读数] {tag} 到位 间隙='
                    f'{(0.052 + pl + pr) * 1000:.0f}mm(目标≤'
                    f'{target * 1000:.0f}mm)')
                return
            if time.monotonic() >= deadline:
                self._pub_gripper_cmd(0.0)
                self.get_logger().warn(
                    f'{tag} 超时 {self._pre_close_wait}s(间隙='
                    f'{(0.052 + pl + pr) * 1000:.0f}mm),软放行')
                return
            self._pub_gripper_cmd(self._pre_close_force)
            time.sleep(0.05)

    def _confirm_clamped(self, tag: str):
        """抬臂前硬闸(用户拍板 2026-10-08 晚):两指没真咬住工件,
        不许抬臂——补锁紧重验,仍不合拢判抓取失败走重试链。

        夹持真伪的下游判据(探针/跟随验证/在场闸)零改动;本闸只把
        "停稳但没合拢"(钉死态间隙 89mm,靠 attach 兜底才没掉)从
        静默放行改为拒绝抬臂。"""
        if self._wait_fingers_clamped(f'{tag} 夹持闸') is not False:
            return  # True=咬合;None=键查不到,旧行为放行
        for n in (1, 2):
            # 解楔脉冲(481k 行 effort 实录,§十六):卡住的那根指抵在
            # 被下探压进指缝的工件上,法向摩擦 > 闭爪力,补锁紧推不开
            # (臂抬离带面后 0.2s 即弹回=阻塞源是下探压力本身)。先全开
            # 0.4s 让工件回落归位,再收口重夹,把"抖掉楔子"动作显式化。
            self._grip(close=False, wait=0.4)
            self._close_to_gap(f'{tag} 解楔重收口{n}')
            self._grip(close=True, force=self._gripper_closed, wait=0.6)
            if self._wait_fingers_clamped(f'{tag} 补锁紧{n}') is not False:
                return
            self.get_logger().warn(f'{tag} 双指未闭合,第 {n} 次重夹')
        raise GraspError('fingers_not_clamped')

    def _wait_fingers_clamped(self, label: str, timeout: float = 2.5,
                              poll: float = 0.05, max_gap_m: float = 0.055):
        """夹持确认闸(用户拍板 2026-10-08 晚):两指内缘真咬住工件才放行抬臂。

        与停稳闸的分工:停稳闸管"指不再动"(防半合拢就抬),本闸管
        "合没合上"(防被工件/限位楔住的假停稳)。返回 True=已夹住,
        False=超时仍未夹住, None=关节键查不到(无法判定,不新增失败
        路径)。"""
        jl, jr = self._finger_joint_keys()
        deadline = time.monotonic() + timeout
        pl = pr = None
        while True:
            pl = self._joint_lookup(jl)
            pr = self._joint_lookup(jr)
            if pl is None or pr is None:
                self.get_logger().warn(
                    f'{label} 夹持闸:两指键查不到,无法判定(跳过)')
                return None
            if fingers_clamped(pl, pr, max_gap_m):
                self.get_logger().info(
                    f'[读数] {label} 夹持确认 间隙='
                    f'{(0.052 + pl + pr) * 1000:.0f}mm'
                    f'(L={pl:+.4f} R={pr:+.4f},≤{max_gap_m * 1000:.0f}mm)'
                    f',可抬臂')
                return True
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f'{label} 夹持闸超时 {timeout}s:间隙='
                    f'{(0.052 + pl + pr) * 1000:.0f}mm 未咬合'
                    f'(L={pl:+.4f} R={pr:+.4f})')
                return False
            time.sleep(poll)

    def _tool_z_now(self):
        """tool0 当前世界 z(TF 不可见返回 None)。"""
        try:
            tf = self._attach._tf.lookup_transform(
                'world', self._tool_frame, rclpy.time.Time())
            return tf.transform.translation.z
        except Exception:  # noqa: BLE001 TF 未就绪:本轮采样跳过
            return None

    def _settle_ladder(self, carry_anchor, lvl0: int, lvl1: int):
        """放入式放置的触底下探梯子:上/下端锚间 4 级线性插值
        (t=0,1/3,2/3,1),执行 [lvl0, lvl1] 闭区间各级关节 PTP。
        返回末级 tool0 世界 z(读不到 TF 返回 None)。每级打 [标定]
        日志供标定回填。

        失速即停:某级完成后物理 z 比上一级下降不足 3mm(撞家族
        屏障)就终止梯子,不再发后续级 —— 失速级的 JTC action 会长期
        占用 move_group 执行槽("Cannot push a new trajectory"),
        继续发级会让后续所有运动目标被拒(2026-10-03 实证);停在
        屏障上由调用方按悬停释放处理。"""
        top, bot = self._settle_pairs.get(
            (round(carry_anchor[0][0], 3), round(carry_anchor[0][1], 3)))
        z = None
        z_prev = None
        for i in range(lvl0, lvl1 + 1):
            t = i / 3.0
            joints = [a + (b - a) * t for a, b in zip(top, bot)]
            if not self._motion.move_to_joints(
                    self._joint_names, joints, 'settle_down', timeout=25.0):
                return None
            z = self._tool_z_now()
            self.get_logger().info(
                f'[标定] settle 梯级 {i + 1}/4: tool0 z='
                f'{z if z is None else round(z, 4)} (触底线 {self._settle_touch_z})')
            if (z is not None and z_prev is not None
                    and z_prev - z < 0.003 and i < lvl1):
                self.get_logger().warn(
                    f'梯级 {i + 1} 失速(z 仅降 {z_prev - z * 1000:.0f}mm),'
                    '提前终止梯子,按当前高度悬停释放')
                break
            z_prev = z
        return z

    def _gripper_world_xy(self):
        """夹爪附着链接的世界系 xy(复用 attach_helper 的 TF buffer)。

        选 gripper_attach_link 而非 tool_frame:attach 跟随的正是该链接,
        验证"工件是否随夹爪动"应测量同一坐标系;TF 不可见时返回 None
        (采样器跳过该拍)。与 attach_helper._follow 线程同款访问模式,
        tf2 Buffer 线程安全。
        """
        try:
            tf = self._attach._tf.lookup_transform(
                'world', self._attach_link, rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except Exception:  # noqa: BLE001 TF 未就绪/超时:跳过本拍采样
            return None

    def _start_follow_sampler(self, part_id: str) -> _FollowSampler:
        """启动跟随采样(夹爪 xy=TF,工件 xy=逻辑相机缓存真值)。"""
        sampler = _FollowSampler(
            gripper_xy_fn=self._gripper_world_xy,
            part_xy_fn=lambda: self._cam_cache.get(part_id))
        sampler._source_fn = lambda: self._part_source_stamps.get(part_id)
        sampler._source_fresh = lambda: (time.monotonic()-self._part_source_walls.get(part_id,0.) < .5 and abs(self.get_clock().now().nanoseconds/1e9-self._part_source_stamps.get(part_id,-100.)) < .2)
        sampler.start()
        return sampler

    def _verify_follow(self, gripper_xy, part_xy) -> tuple[bool, str]:
        """调 GraspVerifier 判定工件是否跟随夹爪,结果打 info 日志。"""
        try:
            ok, reason = self._verifier.verify(gripper_xy, part_xy)
        except Exception as e:  # noqa: BLE001 验证器异常按验证失败处理
            ok, reason = False, f'verifier_error({e})'
        self.get_logger().info(
            f'夹取验证{"通过" if ok else "失败"}: {reason}'
            f'({len(gripper_xy)} 组采样, 夹爪阈值 {self._verify_gripper_move}, '
            f'跟随阈值 {self._verify_follow_move})')
        if not ok and getattr(self._verifier, 'last_windows', None):
            for w in self._verifier.last_windows:
                self.get_logger().info(
                    f'[诊断] 窗口[{w.get("start")}:{w.get("end")}] '
                    f'夹爪位移 {w.get("gripper_move"):.3f} 工件位移 '
                    f'{w.get("part_move"):.3f} 点积 {w.get("dot"):.4f} '
                    f'跟随={w.get("following")}')
        return ok, reason

    # ================================================================ 夹爪
    def _grip(self, close: bool, force: float | None = None,
              wait: float | None = None, sync_cap: bool = False):
        """夹爪命令(force 覆盖默认闭/开力,wait 覆盖等待秒)。

        两段闭爪(verified_only 路径):先轻触力让指面无冲击接触工件,
        再换夹持力锁紧 —— 一次性夹持力经指关节阻尼折算的闭合速度
        (F/15)高达 ~0.5-1.7 m/s,撞击会把工件从指间弹出去(扫帚效应
        的另一诱因,2026-10-01 联调实证)。

        sync_cap(轻触段对称封顶,问题③修复 2026-10-06):r9 指时序
        实录——块偏心贴一侧指面时,该指被静摩擦钉住,轻触 1.5N 推不动,
        另一指 0.5s 内空扫 43mm 到限位,肉眼即"左右不协调";扫掠还
        挤压工件(与扫帚效应同族)。开启后轻触段轮询两指行程,一旦
        **单侧先行 ≥15mm** 即提前结束本段(交给锁紧段对称合拢),
        消除空扫;双侧同步正常时行为与原 wait 一致(仅提前满足)。

        P1 修复(2026-10-08):① 键名解析——arm2 的键在裸名双份发布世界
        下查不到(根因见 _store_joint_states 注释),现经 _joint_lookup
        回落别名表,两臂都查得到;② 收尾等待的逻辑 bug——旧版
        `if not (capped or polled): time.sleep(dur)` 在 polled=True 且
        未 capped 时**一睡不睡**(死分支),轻触段退化为"发命令即返回",
        轮询跑满 dur 却零睡眠,正是肉眼可见的不协调放大器。现按
        "轮询耗时 + 补齐睡眠 = dur"重写:封顶提前 break 时不补睡(本
        就该提前),未封顶(含键查不到的降级路径)必等满 dur。"""
        if force is None:
            value = self._gripper_closed if close else self._gripper_open
        else:
            value = force
        self._pub_gripper_cmd(value)               # 记入保持值并重发
        self._grip_hold_target = None   # 新命令接管,解除位置保持(观感②)
        state = String()
        state.data = 'closed' if close else 'opened'
        self._pub_gripper_status.publish(state)
        dur = self._gripper_wait if wait is None else wait
        waited = 0.0
        if sync_cap and close:
            # 指关节名按本臂前缀解析(复验修正 2026-10-06 定口径,
            # P1 修好取数 2026-10-08):arm1 无前缀,arm2 = "arm2_"
            # (gripper.urdf.xacro prefix)。候选原稿钉死无前缀名,relay
            # 节点上永远查不到 ⇒ 静默回退旧行为(复验 dual 轮实证零触
            # 发);若改用 unprefixed 兜底会读到**对臂**的指,更糟——
            # _joint_lookup 的别名回落只在"同消息重名双份"时成立,主表
            # 裸键恒为第 1 份(arm1),不会把 arm2 误接到 arm1 数据上。
            jl, jr = self._finger_joint_keys()
            base_l = self._joint_lookup(jl)
            base_r = self._joint_lookup(jr)
            if base_l is None or base_r is None:
                missing = [k for k, v in ((jl, base_l), (jr, base_r))
                           if v is None]
                self.get_logger().warn(
                    f'sync_cap 键查不到{missing}(自检已打印,见 [自检] 行),'
                    f'轻触段退回固定等待 {dur}s')
            else:
                self.get_logger().info(
                    f'[读数] 轻触段起读 {jl}={base_l:.4f} '
                    f'{jr}={base_r:.4f}')
                t_end = time.monotonic() + dur
                while time.monotonic() < t_end:
                    cur_l = self._joint_lookup(jl)
                    cur_r = self._joint_lookup(jr)
                    if cur_l is None or cur_r is None:  # 键中途消失(不应发生)
                        self.get_logger().warn(
                            f'sync_cap 轮询中键消失({jl}/{jr}),补齐固定等待')
                        break
                    dl = base_l - cur_l
                    dr = base_r - cur_r
                    if should_cap(dl, dr):
                        self.get_logger().warn(
                            f'轻触段单侧先行(L={dl * 1000:.0f}mm '
                            f'R={dr * 1000:.0f}mm),封顶提前进入锁紧段')
                        return self._grip_done(state, value)
                    time.sleep(0.05)
                    waited = dur - max(0.0, t_end - time.monotonic())
                else:
                    # 轮询跑满未封顶:打印终读两指行程(证明取数链路活着,
                    # 双侧同步的正常形态;静默失效在日志上从此可分辨)
                    tl = (base_l - self._joint_lookup(jl)) * 1000
                    tr = (base_r - self._joint_lookup(jr)) * 1000
                    self.get_logger().info(
                        f'[读数] 轻触段终读 L={tl:.0f}mm '
                        f'R={tr:.0f}mm(双侧同步)')
        # 未封顶(或不开轮询/降级):睡满剩余 dur —— P1 修的正是这里
        if remainder := dur - waited:
            time.sleep(max(0.0, remainder))
        return self._grip_done(state, value)

    def _pub_gripper_cmd(self, value: float):
        """发布夹爪力命令并记入保持值(观感修复①,见 __init__ 注释)。

        单发会因 DDS 首发布竞态丢失,故所有命令统一经此入口:
        立即发一次 + 交 10Hz 定时器重发。重发同值对 effort 控制器
        是幂等的(它本就保持最后收到的值),零物理差。"""
        msg = Float64MultiArray()
        msg.data = [value] * self._gripper_joints   # 左右指同向滑移
        self._pub_gripper.publish(msg)
        self._grip_hold = float(value)

    def _grip_hold_tick(self):
        """10Hz 保持器:重发最近夹爪命令(① 消灭丢包自由漂);
        _grip_hold_target 非 None 时改做释放小开的位置保持(②):
        低于目标发外推力、高于目标+容差发内收力、带内发 0N。"""
        target = self._grip_hold_target
        if target is None:
            value = self._grip_hold
        else:
            jl, jr = self._finger_joint_keys()
            pl = self._joint_lookup(jl)
            pr = self._joint_lookup(jr)
            if pl is None or pr is None:
                value = self._grip_hold
            elif min(pl, pr) < target:
                value = self._grip_release_force
            elif max(pl, pr) > target + 0.002:
                value = -self._grip_release_force
            else:
                value = 0.0
        msg = Float64MultiArray()
        msg.data = [value] * self._gripper_joints
        self._pub_gripper.publish(msg)

    def _grip_done(self, state: String, value: float):
        """_grip 的统一收尾日志(封顶早退与睡满两条路径共用)。"""
        self.get_logger().info(f'夹爪: {state.data} (force={value})')

    # ================================================================ 失败恢复
    def _recover(self, attached_part):
        """安全恢复:松爪、停止跟随(若已附着),为重试腾出干净状态。"""
        try:
            self._grip(close=False)
        except Exception:  # noqa: BLE001 恢复阶段不再抛错
            pass
        if attached_part:
            self._attach.detach()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
