"""任务调度节点:来料登记 -> 任务队列 -> 串行派发(可选双臂接力)。

接口(均为自定义接口,见 a1_interfaces):
- 订阅 /parts/arrival (PartInfo):视觉节点发布的来料登记;
- 订阅 /task/status (TaskStatus):执行节点(arm1/arm2)的状态回报,
  用于推进队列;msg.arm 空串视为 arm1(兼容旧消息);
- 发布 /task/command (PartInfo):向 arm1 执行节点派发单件任务;
- 发布 relay_command_topic (PartInfo,默认 /task/command_arm2):
  relay_mode=true 时,当前件 arm1 COMPLETED 后延迟 relay_delay 秒,
  把该件(目录缓存里的 PartInfo)派发给 arm2 码放;
- 服务 /sort_task (SortTask):start_batch / dispatch_single /
  dispatch_to 外部入口。dispatch_to = "指定箱派发"(2026-10-06
  语言指挥模式):抓取仍走既有锚表路径,仅放置目标箱由命令指定——
  决策逻辑在纯逻辑模块 dispatch_logic.plan_sort_task(可离线单测),
  本节点只做 ROS 桥接;合法箱名单与箱->臂路由由参数 bin_arm_map
  (如 ['bin_blue:arm1','bin_red:arm2',...])给出,非法箱名拒绝并在
  message 回执说明。

串行保证(接力或单臂):同一时刻整条流水线只有一个任务在跑——
当前件的 arm1 分拣 -> (接力)arm2 码放全部到达终态后才派发下一件;
arm1 FAILED 则跳过接力直接下一件。
并联分色(arm2_colors 非空)改为每条臂一个执行槽,两件可以同时在途。
取件区仍是一把全局锁:派发时占用,收到 ZONE_CLEAR 才放开。
ZONE_CLEAR 不是 CARRYING。没接 Q 表时来料按「谁闲推谁」。
接了 Q 表以后,下一件颜色问表;一次仍只推一件,不给正忙的臂预送。

推进决策在纯逻辑类 RelaySequencer(便于单元测试),本节点只做
ROS 桥接:终态喂给 on_terminal,按返回的延迟动作起墙钟 timer
(节点 use_sim_time=True,/clock 中断时 sim-time timer 永不触发会
卡死派发,故用 threading.Timer),到期后调用 try_dispatch /
on_relay_ready 并发布对应命令。

流水线模式扩展(契约 §5/§6;新参数默认值=旧行为完全回归):
- demand_spawn=true:启动即发一条 spawn_topic 请求(data='random')
  并令推板前进(belt_speed);首条请求/前进命令延迟
  belt_first_cmd_delay(墙钟,默认 20s)发布,防止早于 ros_gz 桥/gz
  JointController 订阅就绪被丢(volatile 命令丢件 -> 第一件冻结带尾
  死锁)。此后每件完全终态(relay_mode 下含 arm2 终态,判据=
  on_terminal 受理且无 RELAY 动作)且已请求件数 < demand_parts 时,
  延迟 dispatch_delay 再发一条并再推进(发生在 30s+ 后,桥已就绪,
  不再延迟)。spawn 发布者用 transient_local QoS(depth=1):spawn
  常驻节点由 cell_bringup 延迟启动,volatile 会丢首件请求,
  transient_local 由迟到订阅者补收;
- belt_control=true(推板控制,契约 §4 推板方案:TrackController 表面
  速度在 dartsim 上不稳定已放弃;参数名沿用 belt_* 减少 yaml 面积):
  订阅 /overhead_logical,任一未完全终态工件 world_x(=camera_x+rel_z,
  见 camera_pose 模块)>= belt_stop_x 时发 belt_speed_topic 一条
  -0.25(推板回位命令,不是 0.0!)并打"停推进"日志。停板不再用墙钟
  belt_settle_delay:同一条逻辑相机消息里读模型名 pusher 的 world_x,
  回到起点附近(SDF 姿态 x=0.12,容差 0.02)才发 0.0 并重触发
  try_dispatch。消息时间戳是仿真时间,超过 home_timeout_sim 仍未
  回到起点才按仿真时间兜底停板。回位期间 try_dispatch 挂起。
  belt_control=false 时零订阅零命令发布(旧行为)。
  已终态工件(已入箱,x>=0.56)不参与判停推进,否则从第二件起会在
  推进开始瞬间误触发回位。

L1 方案 a 扩展(2026-10-06 用户拍板;belt_control=true 且 demand_spawn=
false 的 legacy 轮专属,pipeline/dual 零差):
- 串推链 prime:桥就绪延迟(同 belt_first_cmd_delay)且来料满 3 件后
  advance(belt_speed)定时 belt_chain_advance_s(=5s)-> 回位(-0.25)->
  settle(2.6s)停板,此后推板零命令。机理(六轮取证定案):三件出生
  占带面 0.50/0.58/0.66 且件间 3cm 空档,取件推碰沿带接力漂移,第 3
  件 0.94-1.12 入 NO_IK 死区(L1);串推把整条链**匀速推过 2cm 矮挡停
  板**落入板后停放带(x≈0.67-0.95,件间零空档、已离带,后续取件推碰
  不再有接力行程),推行准静态无弹射。曾试"贴板轻串"(observe 0.675
  判停)反而把指压挤压集中到中间件、件被侧弹下带((0.895,-0.182)
  pre_grasp FAILED),弃;纯 5s 定时+宽窗 12/12 PLACED 定案。prime 期间
  派发挂起(_priming);旧 observe 判停在 legacy 轮旁路(出生帧 0.58/
  0.66 在线上会首帧误触发与 prime 链打架)。
- 停位窗派发闸:派发前队首件 world_x(逻辑相机缓存)须落在
  [0.40, 1.00](停放带/出生带恒放行;>1.00 = L1 真死区,<0.40 = 串推
  失败留带尾,越窗挂起 belt_dispatch_recheck_s 复检,连续超
  belt_dispatch_wait_max(=3) 次兜底放行并告警——绝不卡轮,执行器既有
  NO_IK/重试通道读数不失真)。无观测放行(no_camera_pose 重试语义兜底)。
  纯逻辑与计数在 pipeline_logic.DispatchWindowGate(离线可单测)。
"""
import json
import threading
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import Float64, String

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_interfaces.srv import SortTask
from a1_sorting_core.camera_pose import CameraPoseCache, world_xy_from_rel
from a1_sorting_core.dispatch_logic import parse_bin_arm_map, plan_sort_task
from a1_sorting_core.pipeline_logic import (BeltController,
                                            DispatchWindowGate,
                                            choose_idle_feed)
from a1_sorting_core.q_feed import (COLOR_INDEX, COLORS, decide_feed,
                                    describe_q_load, load_q_policy,
                                    resolve_q_paths)
from a1_sorting_core.relay_sequencer import DISPATCH, RELAY, RelaySequencer

TERMINAL_STATES = ('COMPLETED', 'FAILED')


class TaskDispatcherNode(Node):

    def __init__(self):
        super().__init__('task_dispatcher')
        self.declare_parameter('fast_sim_waits', False)
        self._fast_waits = bool(self.get_parameter('fast_sim_waits').value)
        self._startup_logical_wall = 0.0
        self._startup_pusher = None
        self._startup_rgb_wall = 0.0
        self._first_feed_t0 = time.monotonic()
        self._first_feed_logged = False
        self.declare_parameter('four_slot_trial_permit','')
        self._trial_permit=str(self.get_parameter('four_slot_trial_permit').value)
        self.declare_parameter('four_slot_mode',False)
        self.declare_parameter('four_slot_config','')
        self._four_slot=bool(self.get_parameter('four_slot_mode').value)
        if self._four_slot:
            from .four_slot_capacity import load_motion_candidate
            load_motion_candidate(self.get_parameter('four_slot_config').value,self._trial_permit)
            self._trial_counts=tuple(json.loads(Path(self._trial_permit).read_text()).get('counts',(0,1,0,0))) if self._trial_permit else None
        self.declare_parameter('voice_batch_mode', False)
        self._voice_mode = bool(self.get_parameter('voice_batch_mode').value)
        self.declare_parameter('auto_start', False)   # 来料后是否自动开始分拣
        self.declare_parameter('dispatch_delay', 1.0)  # 上一件结束后延迟派发(秒)
        # ---- 双臂接力 ----
        self.declare_parameter('relay_mode', False)   # true: arm1 完成后接力 arm2
        self.declare_parameter('relay_command_topic', '/task/command_arm2')
        self.declare_parameter('relay_delay', 1.0)    # arm1 完成到派发 arm2 的延迟(秒)
        # ---- 双臂并联分色(2026-10-04 Phase1) ----
        # 非空时启用:逗号分隔的颜色类别(如 'yellow')在派发时直接路由给
        # arm2 执行节点(经 relay_command_topic),arm2 从带尾取件投其
        # bin_for_* 目标箱;其余颜色照旧 arm1。并联激活时接力语义
        # (arm1 完成后二次码放)整体关闭——arm1 的终态即完全终态,
        # arm2 路由件的 arm2 终态即完全终态。空 = 经典接力/单臂不变。
        self.declare_parameter('arm2_colors', '')
        # ---- R1-P1 停轮闸(2026-10-04 终审;默认关=定案态零行为差)----
        # 收到 detail=motion_session_wedged 的终态 ⇒ 执行器所在 move_group
        # 会话已中毒(后续目标一律 -4),立即停派发+停来料并丢弃队列剩余件。
        # 否则一轮 6 件会空转出 6 个 FAILED,失败率读数被架构缺陷放大
        # (上轮 E2 的 6/6 FAILED 即此失真,P2 复核注记)。停轮后由外层
        # 受控重启 move_group 再整轮重跑。
        self.declare_parameter('halt_on_wedge', False)
        # ---- 指定箱派发(2026-10-06 语言指挥模式) ----
        # 箱名 -> 执行臂 的合法名单与路由表,字符串列表形如
        # ['bin_blue:arm1', 'bin_red:arm2']。空表 = 调度器不校验箱名、
        # 不按箱路由(dispatch_to 仍接受,箱名交由执行器按本臂
        # bin_names 校验;路由沿用既有按色路由)——默认空,旧行为零差。
        # 默认 [''] 占位(rclpy 空数组会被推断成 BYTE_ARRAY 类型,与
        # 参数文件里的字符串数组冲突;'' 在 parse_bin_arm_map 中忽略,
        # 等价空表 = 不校验箱名,旧行为零差)
        self.declare_parameter('bin_arm_map', [''])
        self._bin_arm_map = parse_bin_arm_map(
            ','.join(str(v) for v in
                     self.get_parameter('bin_arm_map').value))
        # part_id -> 放置目标箱覆盖(dispatch_to 写入,派发时注入命令,
        # 该件到达完全终态时清除,防串到下一轮同 id 件)
        self._bin_overrides: dict[str, str] = {}
        # ---- 按需来料(契约 §5) ----
        self.declare_parameter('demand_spawn', False)  # true: 按需逐件生成
        # 单轮总来料数;<=0 = 无限持续来料(演示模式:每件完全终态后
        # 请求下一件,配合 recorder 入箱回收长期运行不堆料)
        self.declare_parameter('demand_parts', 3)
        # 按需来料颜色:'random' 随机 red/green;指定 red/green 则强制
        # 同色(验收压测用:三件同箱触发全部 3 格含 +y 边缘格)
        self.declare_parameter('demand_color', 'random')
        self.declare_parameter('spawn_topic', '/parts/request_spawn')  # String
        # ---- 推板控制(契约 §4 推板方案,参数名沿用 belt_* 减少 yaml 面积) ----
        self.declare_parameter('belt_control', False)  # true: 判停推进+派发门控
        self.declare_parameter(
            'belt_speed_topic', '/model/pusher/joint/pusher_slide/cmd_vel')
        self.declare_parameter('belt_speed', 0.1)      # 推板前进速度 m/s(+x)
        self.declare_parameter('belt_stop_x', 0.40)    # 停推进 world_x 阈值
        self.declare_parameter('belt_settle_delay', 2.6)  # 回位命令->板停(派发放行)等待 s
        self.declare_parameter('camera_xy', [0.65, 0.0])  # 逻辑相机安装 xy
        # 首条 spawn 请求/推板前进命令的延迟(墙钟秒):dispatcher 启动即发
        # 会早于 ros_gz 桥与 gz TrackController 的订阅就绪(两段发现
        # 握手),volatile 带速命令被丢 -> 第一件冻结带尾死锁;统一
        # 延迟 N 秒发布(spawn 是 transient_local 本不怕迟到,一并延迟
        # 简单可靠)。0 = 不延迟(立即发布)
        self.declare_parameter('belt_first_cmd_delay', 20.0)
        # ---- L1 方案 a(2026-10-06 拍板):legacy 串推链 + 停位窗派发闸 ----
        # legacy 轮(belt_control=true 且 demand_spawn=false)专属,两段:
        # ① prime 一次性串推(三命令时序复用 pipeline:前进/回位/停板,
        #    同 belt_first_cmd_delay 等桥就绪):advance belt_speed 定时
        #    belt_chain_advance_s -> 回位 -0.25 -> belt_settle_delay 后
        #    停板 0.0 放行首件派发。三件被匀速推过 2cm 矮挡停板落入板后
        #    停放带(x≈0.67-0.95,件间零空档、已离带),取件推碰不再有
        #    沿带接力行程,L1 漂移(0.94-1.12 NO_IK)源头消解。此后再无
        #    任何推进命令(legacy 无逐件来料)。prime 期间派发挂起
        #    (_priming 闸)。旧 observe 判停在 legacy 轮旁路:出生帧
        #    0.58/0.66 在 0.58 线上/前,首帧即回位与 prime 链打架。
        # ② 派发前队首件 world_x 须在停位窗 [0.40, 1.00]:越窗挂起
        #    belt_dispatch_recheck_s 复检,超 belt_dispatch_wait_max 次
        #    兜底放行并告警(绝不卡轮;执行器 NO_IK 通道不受影响)。
        # demand_spawn 轮(pipeline/dual)两段逻辑均不激活,行为零差;
        # belt_control=false 时亦全关(回退旧行为)。
        self.declare_parameter('belt_chain_advance_s', 5.0)
        # 停位窗(实测标定,2026-10-06 六轮取证定案):串推 5s 定时链把
        # 三件匀速推过 2cm 矮挡停板、停进板后停放带 x≈0.67-0.95
        # ("贴板轻串"变体实证会把中间件"瓜籽"侧弹下带,弃)。窗
        # [0.40, 1.00]:停放带+出生带恒放行;>1.00 = L1 真死区(NO_IK
        # 样本 1.00-1.12)越窗挂起复检;<0.40 = 串推失败留带尾,同拦。
        self.declare_parameter('belt_dispatch_window_lo', 0.40)
        self.declare_parameter('belt_dispatch_window_hi', 1.00)
        self.declare_parameter('belt_dispatch_recheck_s', 2.0)
        self.declare_parameter('belt_dispatch_wait_max', 3)
        # Q 表路径空着就还是谁闲推谁。路径写上以后只读那两份 JSON，不改它们。
        self.declare_parameter('q_package_dir', '')
        self.declare_parameter('q_config_path', '')
        self.declare_parameter('q_table_path', '')
        self.declare_parameter('q_batch', [2, 2, 2, 2])

        self._auto_start = bool(self.get_parameter('auto_start').value)
        self._dispatch_delay = float(self.get_parameter('dispatch_delay').value)
        self._demand_spawn = bool(self.get_parameter('demand_spawn').value)
        self._demand_parts = int(self.get_parameter('demand_parts').value)
        # demand_parts <= 0 -> 无限持续来料(演示模式,见 §5 注释)
        self._demand_unlimited = self._demand_parts <= 0
        self._demand_color = str(self.get_parameter('demand_color').value)
        self._spawn_topic = str(self.get_parameter('spawn_topic').value)
        self._belt_control = bool(self.get_parameter('belt_control').value)
        self._belt_speed_topic = str(
            self.get_parameter('belt_speed_topic').value)
        self._belt_speed = float(self.get_parameter('belt_speed').value)
        self._belt_stop_x = float(self.get_parameter('belt_stop_x').value)
        self._belt_settle_delay = float(
            self.get_parameter('belt_settle_delay').value)
        self._belt_first_cmd_delay = float(
            self.get_parameter('belt_first_cmd_delay').value)
        # L1 方案 a 参数(demand_spawn=true 的 pipeline/dual 轮全部不激活)
        self._legacy_prime = self._belt_control and not self._demand_spawn
        self._belt_chain_advance_s = float(
            self.get_parameter('belt_chain_advance_s').value)
        self._win_lo = float(self.get_parameter('belt_dispatch_window_lo').value)
        self._win_hi = float(self.get_parameter('belt_dispatch_window_hi').value)
        self._recheck_s = float(
            self.get_parameter('belt_dispatch_recheck_s').value)
        self._wait_max = int(self.get_parameter('belt_dispatch_wait_max').value)
        self._priming = self._legacy_prime   # prime 期间挂起派发
        self._prime_timer = None             # advance->回位的保险 timer
        self._prime_started = False          # 起链唯一性闸(双路触发)
        self._prime_retracting = False       # 回位唯一性闸(observe/定时双入口)
        self._prime_t0 = time.monotonic()
        self._prime_ready_at = 0.0           # legacy_prime 时覆盖为桥就绪延迟
        self._gate = DispatchWindowGate(
            self._win_lo, self._win_hi, self._wait_max)
        self._window_timer = None

        self._lock = threading.Lock()
        from a1_sorting_core.voice_batch_state import VoiceBatchState
        self._voice = VoiceBatchState(retain_capacity=4 if self._four_slot else 0) if self._voice_mode else None
        self._voice_visible = set()
        self._voice_logical_wall = 0.0
        self._voice_logical_sim = None
        self._catalog: dict[str, PartInfo] = {}  # part_id -> 最新来料信息
        self._order: list[str] = []              # 来料顺序
        self._arm2_colors = {
            c.strip() for c in str(
                self.get_parameter('arm2_colors').value).split(',') if c.strip()}
        self._halt_on_wedge = bool(self.get_parameter('halt_on_wedge').value)
        self._halted = False   # R1-P1 停轮标志(halt_on_wedge 开启且收到判死)
        # 并联分色激活时接力语义关闭:非路由色 arm1 终态即完全终态,
        # 路由色由 arm2 一次做完(见 RelaySequencer 并联注释)
        relay_mode = bool(self.get_parameter('relay_mode').value)
        parallel_active = bool(self._arm2_colors)
        # 指定箱派发的可用臂:只有双臂确实在线(relay 或并联)时才允许把
        # 件路由到 arm2;单臂配置下 bin_arm_map 里的 arm2 箱在入队即被
        # 拒绝(回执说明),避免派发到无人订阅的 /task/command_arm2 卡死
        self._available_arms = {'arm1'} | (
            {'arm2'} if (relay_mode or parallel_active) else set())
        if self._bin_arm_map:
            self._bin_arm_map = {
                b: a for b, a in self._bin_arm_map.items()
                if a in self._available_arms}
        # 并联才开每臂一槽。接力/单臂仍是全局一个 _current。
        self._per_arm_slots = parallel_active
        self._feed_inflight = False
        self._belt_part_id = None
        self._off_belt: set[str] = set()
        self._last_feed_arm = None
        self._feed_counts = {'blue': 0, 'yellow': 0, 'red': 0, 'green': 0}
        # 臂上正在做的那件:颜色序号,以及离开取件区的仿真秒。终态后清掉。
        self._arm_post = {'arm1': None, 'arm2': None}
        self._last_sim_sec = 0.0
        self._q_config = None
        self._q_table = None
        self._q_package_dir = str(self.get_parameter('q_package_dir').value)
        self._q_batch = [int(n) for n in self.get_parameter('q_batch').value]
        if self._voice_mode and (not self._demand_spawn or not self._per_arm_slots or not self._fast_waits or not self._belt_control or self._legacy_prime or str(self.get_parameter('q_table_path').value).strip()):
            raise ValueError('voice mode requires dual demand spawn, v3 and explicit rule, no legacy prime or Q')
        self._load_q_policy()
        self._seq = RelaySequencer(
            relay_mode=relay_mode and not parallel_active,
            relay_delay=float(self.get_parameter('relay_delay').value),
            dispatch_delay=float(self.get_parameter('dispatch_delay').value),
            require_arm_ready=True,
            per_arm_slots=self._per_arm_slots)
        self._next_timer = None   # dispatch_delay 定时器
        self._relay_timer = None  # relay_delay 定时器
        self._spawn_timer = None  # dispatch_delay 延迟的来料请求定时器
        self._settle_timer = None  # 回位命令 -> 板停(settle)墙钟定时器

        # ---- 流水线:按需来料 + 带控状态 ----
        self._spawned = 0                       # 已请求来料数(_spawned 计数器)
        self._finished_parts: set[str] = set()  # 完全终态件(判停推进时排除)
        self._pub_spawn = None
        self._pub_belt = None
        self._cam_cache = None
        self._belt = None
        if self._demand_spawn:
            # transient_local(depth=1):spawn 常驻节点晚于本节点启动时
            # 补收首件请求(volatile 会丢,见模块注释)
            self._pub_spawn = self.create_publisher(
                String, self._spawn_topic,
                QoSProfile(depth=1,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=QoSReliabilityPolicy.RELIABLE))
        if self._demand_spawn or self._belt_control:
            # 推板速度命令发布:前进(demand_spawn 路径)/回位/停板
            # (belt_control 路径),值分别为 belt_speed / -0.25 / 0.0
            self._pub_belt = self.create_publisher(
                Float64, self._belt_speed_topic, 10)
        if self._belt_control:
            cam_xy = [float(v) for v in self.get_parameter('camera_xy').value]
            self._cam_cache = CameraPoseCache((cam_xy[0], cam_xy[1]))
            self._belt = BeltController(stop_x=self._belt_stop_x,
                                        settle_delay=self._belt_settle_delay,
                                        advance_speed=self._belt_speed)
            self.create_subscription(LogicalCameraImage, '/overhead_logical',
                                     self._on_logical, 10)
        if self._legacy_prime:
            # L1 方案 a:串推定时链(与 BeltController 解耦——legacy 无逐件
            # 来料,belt.start() 永不调用,observe 恒 None,旧 settle 门控
            # 对 legacy 恒放行;停位保障由本 prime 链 + 派发窗门控承担)。
            # prime 三命令时序复用 pipeline:advance(belt_speed)->
            # retract(-0.25)-> stop(0.0,间隔 belt_settle_delay)。触发条件:
            # 桥就绪延迟 belt_first_cmd_delay 已过 且 目录已有 >=3 件
            # (batch 三件一次生成;不足 3 件时 +15s 兜底照推,窗口门控
            # 仍保护);此后再无任何推进命令。保险见 _prime_retract。
            self._prime_ready_at = self._belt_first_cmd_delay
            self._prime_t0 = time.monotonic()
            self._prime_timer = threading.Timer(
                self._prime_ready_at, lambda: self._prime_maybe_start(False))
            self._prime_timer.daemon = True
            self._prime_timer.start()
            self.get_logger().info(
                f'L1 串推链已排程:≥{self._belt_first_cmd_delay}s 且来料'
                f'满 3 件后前进 {self._belt_chain_advance_s}s -> 回位 -> '
                f'{self._belt_settle_delay}s 停板放行派发'
                f'(停位窗 [{self._win_lo}, {self._win_hi}])')

        self._pub_command = self.create_publisher(PartInfo, '/task/command', 10)
        relay_topic = str(self.get_parameter('relay_command_topic').value)
        self._pub_relay = self.create_publisher(PartInfo, relay_topic, 10)
        self.create_subscription(PartInfo, '/parts/arrival', self._on_arrival, 10)
        self.create_subscription(TaskStatus, '/task/status', self._on_status, 10)
        self.create_subscription(TaskStatus,'/task/placement_result',self._on_status,10)
        self.create_service(SortTask, '/sort_task', self._on_sort_task)
        if parallel_active:
            mode = (f'双臂并联分色(arm2 专收: {sorted(self._arm2_colors)}'
                    f', 每臂一个执行槽, 取件区互斥, 来料=谁闲推谁)')
        elif relay_mode:
            mode = '双臂接力'
        else:
            mode = '单臂'
        self.get_logger().info(
            f'调度器就绪({mode}): /sort_task 服务可用 '
            f'(start_batch/dispatch_single/dispatch_to)'
            + (f', 指定箱合法名单: {sorted(self._bin_arm_map)}'
               if self._bin_arm_map else ''))
        if self._demand_spawn or self._belt_control:
            if self._belt is not None:
                home_note = (
                    f'回位后等推板回到 x<='
                    f'{self._belt.home_x + self._belt.home_tol:.2f},'
                    f'仿真时间兜底 {self._belt.home_timeout_sim:.0f}s')
            else:
                home_note = f'回位后 settle {self._belt_settle_delay}s'
            self.get_logger().info(
                f'流水线: 按需来料 {self._demand_spawn}'
                f'(计划 '
                 + ('语音动态计划待确认，未授权零送料' if self._voice_mode else ('持续无限' if self._demand_unlimited
                    else f'{self._demand_parts} 件'))
                 + f', 话题 {self._spawn_topic}), '
                f'推板控制 {self._belt_control}'
                f'(前进 {self._belt_speed} m/s, 停推进线 x>={self._belt_stop_x}, '
                f'{home_note})')
        if getattr(self,'_voice_mode',False):
            from a1_interfaces.srv import SubmitBatchPlan
            from std_srvs.srv import Trigger
            self._voice_pub = self.create_publisher(String, '/voice_batch/status', QoSProfile(depth=1,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,reliability=QoSReliabilityPolicy.RELIABLE))
            self.create_service(SubmitBatchPlan, '/voice_batch/submit', self._voice_on_submit)
            self.create_service(Trigger, '/voice_batch/stop', self._voice_on_stop)
            self.create_subscription(String, '/voice_batch/recycle', self._voice_on_recycle, 10)
            with self._lock:self._voice_flush_locked()
            self._voice_stopping=False
            self._voice_watchdog_thread=threading.Thread(target=self._voice_watchdog,daemon=True)
            self._voice_watchdog_thread.start()
        if self._demand_spawn:
            # 首件请求 + 推板前进。首条命令延迟 belt_first_cmd_delay(墙钟
            # Timer)发布:防止早于 gz 桥/TrackController 订阅就绪被丢
            # (见参数注释);延迟期间无任何来料,门控/计数均不受影响
            if self._fast_waits:
                self._start_ready_feed()
            elif self._belt_first_cmd_delay > 0.0:
                self._destroy_spawn_timer()
                self._spawn_timer = threading.Timer(
                    self._belt_first_cmd_delay,
                    self._request_spawn_and_start_belt)
                self._spawn_timer.daemon = True
                self._spawn_timer.start()
                self.get_logger().info(
                    f'首件来料请求与推板前进延迟 {self._belt_first_cmd_delay}s '
                    f'发布(等待 ros_gz 桥/gz 世界就绪)')
            else:
                self._request_spawn_and_start_belt()

    # ---------------------------------------------------------------- 来料
    def _on_arrival(self, msg: PartInfo):
        if getattr(self,'_voice_mode',False):
            with self._lock:
                fresh=self._voice.arrival(msg.part_id,msg.category,time.monotonic())
                self._voice_flush_locked()
                if self._halted or not fresh:return
        with self._lock:
            if msg.part_id not in self._catalog:
                self._order.append(msg.part_id)
                self.get_logger().info(
                    f'来料登记: {msg.part_id} (类别 {msg.category})')
                if self._auto_start or getattr(self,'_voice_mode',False):
                    self._seq.enqueue(msg.part_id)
                if self._per_arm_slots and self._belt_part_id is None:
                    self._belt_part_id = msg.part_id
            self._catalog[msg.part_id] = msg
            hook = (self._legacy_prime and not self._prime_started
                    and len(self._catalog) >= 3)
        if hook:
            # L1 串推:满 3 件即试探(_prime_maybe_start 内自校验桥就绪,
            # 未就绪则等排程 timer,幂等安全)
            self._prime_maybe_start(False)
        if self._auto_start or getattr(self,'_voice_mode',False):
            self._try_dispatch()

    # ---------------------------------------------------------------- 服务
    def _on_sort_task(self, request, response):
        if getattr(self,'_voice_mode',False):
            response.accepted=False;response.message='voice_batch exclusive: use confirmed /voice_batch/submit';return response
        if self._halted:
            response.accepted=False;response.message="ARM_FAULT: controlled full restart required"
            return response
        with self._lock:
            known = set(self._catalog)
            # 既有按色路由结果作为 default_arm(并联激活且队列为空时
            # 无从 peek,这里只影响新入队件的初始建议;真正派发时刻在
            # _try_dispatch 按覆盖箱名再定)
            decision = plan_sort_task(
                request.command, request.part_id,
                getattr(request, 'target_bin', ''),
                known, queue_size=self._seq.queue_size,
                bin_arm_map=self._bin_arm_map,
                busy_parts=self._seq.busy_parts())
            if decision.accepted and decision.enqueue_all:
                # 在途件(排队中/执行中)跳过:enqueue 只对 _queued/_done
                # 去重,对执行中的 _current 会重新入队 -> 该件终态后被
                # 二次派发(语音/外部 start_batch 撞 auto_start 时实证)
                busy = self._seq.busy_parts()
                for part_id in self._order:
                    if part_id not in busy:
                        self._seq.enqueue(part_id)
                decision.message = (f'已入队 {self._seq.queue_size} 件,'
                                    '开始分拣')
            elif decision.accepted and decision.enqueue:
                if decision.clear_override:
                    self._bin_overrides.pop(decision.part_id, None)
                    cached = self._catalog.get(decision.part_id)
                    if cached is not None:
                        cached.target_bin_override = ''
                if decision.target_bin:
                    self._bin_overrides[decision.part_id] = \
                        decision.target_bin
                    cached = self._catalog.get(decision.part_id)
                    if cached is not None:
                        cached.target_bin_override = decision.target_bin
                self._seq.enqueue(decision.part_id)
        response.accepted = decision.accepted
        response.message = decision.message
        self.get_logger().info(
            f'/sort_task[{request.command}]: {decision.message}')
        if response.accepted:
            self._try_dispatch()
        return response

    # ---------------------------------------------------------------- 状态
    def _on_status(self, msg: TaskStatus):
        if getattr(self,'_voice_mode',False):
            with self._lock:
                self._voice.status(msg.part_id,msg.arm or 'arm1',msg.state,msg.detail)
                self._voice_flush_locked()
                if self._halted:return
        self._note_sim(msg.header.stamp)
        if msg.state == 'ZONE_CLEAR':
            self._on_zone_clear(msg)
            return
        arm = msg.arm or 'arm1'
        outcome_event=msg.state in ('PART_PLACED','PART_REJECTED','FAILED')
        if outcome_event:
            with self._lock:
                accepted=self._seq.note_outcome(msg.part_id,arm,'FAILED' if msg.state=='PART_REJECTED' else msg.state)
                seen_ready=self._seq.has_ready(msg.part_id,arm)
                if self._seq.is_faulted(arm):self._halted=True
            if self._halted:
                self._destroy_spawn_timer();self._destroy_next_timer();self._destroy_relay_timer()
                stop=Float64();stop.data=0.;self._pub_belt.publish(stop)
                self.get_logger().error('ARM_OUTCOME_FAULT '+msg.part_id+' '+arm)
                return
            if not accepted or not seen_ready:return
        if msg.state == 'ARM_FAULT':
            with self._lock:
                accepted=self._seq.arm_fault(msg.part_id,arm)
                if accepted:self._halted=True
            if accepted:
                self._destroy_spawn_timer(); self._destroy_next_timer(); self._destroy_relay_timer()
                stop=Float64();stop.data=0.;self._pub_belt.publish(stop)
                self.get_logger().error('ARM_FAULT_HALT '+msg.part_id+' '+arm+' '+msg.detail)
            return
        if msg.state != 'ARM_READY' and not outcome_event:
            return
        # R1-P1 停轮:执行器报告 move_group 会话楔死判死 ⇒ 停派发+停来料
        # (中毒会话内任何后续件都只会产出 -4 假 FAILED,读数失真)
        if (self._halt_on_wedge and msg.state == 'FAILED'
                and 'motion_session_wedged' in (msg.detail or '')
                and not self._halted):
            self._halted = True
            with self._lock:
                dropped = self._seq.queue_size
            self._destroy_spawn_timer()
            self._destroy_next_timer()
            self.get_logger().error(
                f'HALT_ON_WEDGE: {msg.part_id} 报会话楔死,停派发停来料,'
                f'队列剩余 {dropped} 件不再消耗(等受控重启 move_group 后'
                f'整轮重跑;失败率读数只计到判死件为止)')
            return
        actions = None
        full_terminal = False
        with self._lock:
            accepted, actions, terminal_state = self._seq.arm_ready(
                msg.part_id, arm)
            if accepted:
                arm = msg.arm or 'arm1'   # 空串视为 arm1(兼容旧消息)
                self._arm_post[arm] = None
                self.get_logger().info('ARM_SLOT_RELEASE '+json.dumps({'part':msg.part_id,'arm':arm,'slot_arm1':self._seq.slot('arm1'),'slot_arm2':self._seq.slot('arm2'),'zone_arm':self._seq.zone_arm,'trigger':msg.state}))
                stage = 'arm2 接力' if arm == 'arm2' else 'arm1 分拣'
                summary = (f'{msg.part_id} {stage}完成'
                           if terminal_state == 'COMPLETED'
                           else f'{msg.part_id} {stage}失败({msg.detail})')
                self.get_logger().info(f'任务终态: {summary},'
                                       f'队列剩余 {self._seq.queue_size} 件')
                # 完全终态 = on_terminal 受理且无 RELAY 动作(单臂任意
                # 终态 / relay 模式 arm1 FAILED 或 arm2 终态);复用
                # DISPATCH 动作同源的判断,此后才允许来下一件
                full_terminal = not any(kind == RELAY for kind, _ in actions)
                if full_terminal:
                    if getattr(self,'_voice_mode',False):
                        self._voice.release(msg.part_id,arm,time.monotonic())
                        self._voice_flush_locked()
                    self._finished_parts.add(msg.part_id)
                    # 指定箱覆盖防串:完全终态后清除该件残留覆盖
                    # (缓存命令同步清空,防同名件下一轮串用)
                    self._bin_overrides.pop(msg.part_id, None)
                    cached = self._catalog.get(msg.part_id)
                    if cached is not None:
                        cached.target_bin_override = ''
                    # L1 停位窗计数随件销账(同名件下一轮重新计数)
                    self._gate.forget(msg.part_id)
        if actions:
            self._schedule(actions)
        if full_terminal and self._demand_spawn:
            if self._per_arm_slots:
                self._request_spawn_and_start_belt()
            else:
                self._schedule_next_spawn()

    def _on_zone_clear(self, msg: TaskStatus):
        """安全退出。只开取件区锁,不把这条臂标成空闲。"""
        arm = msg.arm or 'arm1'
        with self._lock:
            released = self._seq.release_zone(msg.part_id, arm)
            if released:
                self._off_belt.add(msg.part_id)
                job = self._arm_post.get(arm)
                if job is not None:
                    job['since'] = self._last_sim_sec
                if self._belt_part_id == msg.part_id:
                    self._belt_part_id = None
                    self._feed_inflight = False
        if not released:
            return
        self.get_logger().info(
            f'取件区释放: {msg.part_id} {arm} 事件 {msg.detail}')
        self._try_dispatch()
        if self._per_arm_slots and self._demand_spawn:
            self._request_spawn_and_start_belt()

    # ---------------------------------------------------------------- 来料请求
    def _schedule_next_spawn(self):
        """完全终态后来下一件(契约 §5):延迟 dispatch_delay 再发请求。"""
        if self._halted:      # R1-P1 停轮:不再来料
            return
        with self._lock:
            if not self._demand_unlimited \
                    and self._spawned >= self._demand_parts:
                return
        self._destroy_spawn_timer()
        self._spawn_timer = threading.Timer(
            self._dispatch_delay, self._request_spawn_and_start_belt)
        self._spawn_timer.daemon = True
        self._spawn_timer.start()

    def _start_ready_feed(self):
        """Wall-bounded readiness; no feed on timeout. Physical/scheduling gates remain."""
        import subprocess
        from controller_manager_msgs.srv import ListControllers
        from ros_gz_interfaces.srv import SpawnEntity
        from sensor_msgs.msg import Image
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from gz.transport13 import Node as GzNode
        self._ready_control = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._ready_spawn = self.create_client(SpawnEntity, '/world/sorting_cell/create')
        self._startup_readiness_complete = False
        def observe_startup_rgb(message):
            self._startup_rgb_wall = time.monotonic()
            if self._startup_readiness_complete and not getattr(self,'_voice_mode',False) and self._ready_image is not None:
                subscription = self._ready_image
                self._ready_image = None
                # Destroy our temporary observer inside its own executor callback.
                # Gazebo, the original vision node and the performance probe remain.
                self.destroy_subscription(subscription)
                self.get_logger().info('STARTUP_OBSERVER_RELEASED: temporary RGB readiness subscriber only')
        self._ready_image = self.create_subscription(
            Image, '/overhead_camera', observe_startup_rgb,
            qos_profile_sensor_data)
        self._ready_gz = GzNode()
        self._ready_move = ActionClient(self, MoveGroup, '/move_action')
        def run():
            deadline = time.monotonic() + 60.0
            required = {'joint_state_broadcaster', 'joint_trajectory_controller',
                        'arm2_joint_trajectory_controller', 'gripper_controller',
                        'arm2_gripper_controller'}
            active = set(); future = None; last_sim = 0.0; checks = {}
            query_started = 0.0; response_at = 0.0; next_query = 0.0
            transport_at = 0.0; next_cli = 0.0; next_diagnostic = 0.0
            query_timeouts = 0; query_error = ''; transport_error = ''
            while rclpy.ok() and not getattr(self,'_voice_stopping',False) and time.monotonic() < deadline:
                now = time.monotonic(); sim = self.get_clock().now().nanoseconds / 1e9
                if future is not None and future.done():
                    try:
                        active = {c.name for c in future.result().controller if c.state == 'active'}
                        response_at = now; query_error = ''
                    except Exception as exc:
                        active = set(); query_error = str(exc)
                    future = None; next_query = now + .5
                if future is not None and now - query_started >= 2.0:
                    # One lost RPC must not occupy the whole startup deadline.
                    self._ready_control.remove_pending_request(future)
                    future.cancel(); future = None
                    query_timeouts += 1; next_query = now + .1
                if future is None and now >= next_query and self._ready_control.service_is_ready():
                    future = self._ready_control.call_async(ListControllers.Request())
                    query_started = now
                try:
                    _, subscribers = self._ready_gz.topic_info(self._belt_speed_topic)
                    if subscribers:
                        transport_at = time.monotonic(); transport_error = ''
                except Exception as exc:
                    transport_error = str(exc)
                if now - transport_at > 3.0 and now >= next_cli:
                    # Harmonic Python discovery can miss an existing subscriber.
                    # Independently require a real typed subscriber; never bypass.
                    remaining = deadline - time.monotonic()
                    if remaining > .1:
                        try:
                            result = subprocess.run(
                                ['gz', 'topic', '-i', '-t', self._belt_speed_topic],
                                capture_output=True, text=True,
                                timeout=min(2.0, remaining), check=False)
                            subscriber_section = result.stdout.partition('Subscribers [Address, Message Type]:')[2]
                            if result.returncode == 0 and any(
                                    'tcp://' in line and 'gz.msgs.Double' in line
                                    for line in subscriber_section.splitlines()):
                                transport_at = time.monotonic(); transport_error = ''
                            else:
                                transport_error = 'no typed subscriber in CLI discovery'
                        except (subprocess.TimeoutExpired, OSError) as exc:
                            transport_error = str(exc)
                    next_cli = time.monotonic() + 2.0
                now = time.monotonic()
                transport_ok = now - transport_at < 3.0
                checks = {
                    'world_clock_advances': sim > last_sim > 0.0,
                    'logical_fresh': now-self._startup_logical_wall < 1.0,
                    'rgb_fresh': now-self._startup_rgb_wall < 1.0,
                    'pusher_at_home': self._startup_pusher is not None and self._startup_pusher <= .14,
                    'spawn_bridge_service': self._ready_spawn.service_is_ready(),
                    'spawn_subscriber': self._pub_spawn.get_subscription_count() > 0,
                    'pusher_bridge_subscriber': self._pub_belt.get_subscription_count() > 0,
                    'pusher_transport_subscriber': transport_ok,
                    'controllers_active': required.issubset(active) and now-response_at < 3.0,
                    'move_group_ready': self._ready_move.server_is_ready(),
                }
                if all(checks.values()):
                    self.get_logger().info('STARTUP_READY '+json.dumps({'checks':checks,'active':sorted(active),'wall_wait':now-self._first_feed_t0,'sim':sim}))
                    self._startup_readiness_complete = True
                    if getattr(self,'_voice_mode',False):
                        with self._lock:
                            self._voice.ready()
                            self._voice_flush_locked()
                    self._request_spawn_and_start_belt()
                    return
                if now >= next_diagnostic:
                    self.get_logger().info('STARTUP_CHECK '+json.dumps({
                        'checks': checks, 'active': sorted(active),
                        'controller_reply_age_s': now-response_at if response_at else None,
                        'controller_rpc_timeouts': query_timeouts,
                        'controller_error': query_error, 'transport_error': transport_error}))
                    next_diagnostic = now + 5.0
                last_sim = sim
                time.sleep(.1)
            if getattr(self,'_voice_stopping',False):return
            self._halted = True
            self.get_logger().error('STARTUP_READINESS_TIMEOUT: no feed; '+json.dumps(checks))
        self._startup_ready_thread=threading.Thread(target=run, daemon=True)
        self._startup_ready_thread.start()

    def _note_sim(self, stamp):
        """记下状态消息上的仿真秒。没有戳时保持上一次的值。"""
        sim = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if sim > 0.0:
            self._last_sim_sec = sim

    def _load_q_policy(self):
        """路径都写了才加载。失败或路径为空就留空,来料仍走谁闲推谁。"""
        config_path = str(self.get_parameter('q_config_path').value).strip()
        table_path = str(self.get_parameter('q_table_path').value).strip()
        if not config_path or not table_path:
            self._q_config = None
            self._q_table = None
            self.get_logger().info(describe_q_load(False, '没有填写 Q 表路径'))
            return
        try:
            package_dir, config_path, table_path = resolve_q_paths(
                self._q_package_dir, config_path, table_path)
            self._q_config, self._q_table = load_q_policy(
                package_dir, config_path, table_path)
        except Exception as exc:
            self._q_config = None
            self._q_table = None
            self.get_logger().error(describe_q_load(False, str(exc)))
            return
        self._q_package_dir = package_dir
        from pathlib import Path
        import hashlib
        self.get_logger().info('Q_AUDIT_LOAD '+json.dumps({
            'config_path': config_path, 'table_path': table_path,
            'config_sha256': self._q_table['config_sha256'],
            'config_file_sha256': hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
            'table_file_sha256': hashlib.sha256(Path(table_path).read_bytes()).hexdigest(),
            'allow_prefeed_busy_arm': self._q_config['allow_prefeed_busy_arm'],
            'deployment_ready': self._q_table['deployment_ready'],
            'overlap_validated': self._q_config['overlap_validated']}))
        self.get_logger().info(describe_q_load(
            True,
            f'{len(self._q_table["q_values"])} 个状态, '
            f'批次 {list(self._q_batch)}。忙着的臂不预送。'
            f'配置文件里的 overlap_validated 仍是 '
            f'{self._q_config["overlap_validated"]}。'
            f'表文件 {table_path}'))

    def _choose_q_feed_locked(self):
        """在已持有 _lock 时问 Q 表。返回 (选择或 None, 日志短句)。

        选择是 (臂, 颜色)。None 且短句非空时,调用方把短句打出来并且不推件。
        缺离区时刻、或表选了忙臂时,退回谁闲推谁。
        """
        idle = self._seq.idle_arms()
        if not idle:
            return None, ''
        now = float(self._last_sim_sec)
        arms = []
        for name in ('arm1', 'arm2'):
            if name in idle:
                arms.append({'phase': 'READY', 'color': -1, 'phase_start': now})
                continue
            job = self._arm_post.get(name)
            if not job or job.get('since') is None:
                choice = choose_idle_feed(
                    idle, self._last_feed_arm, self._feed_counts)
                note = ''
                if choice:
                    note = f', 闲臂 {choice[0]}（Q表缺离区时刻,改用谁闲推谁）'
                return choice, note
            arms.append({
                'phase': 'POST_PICK',
                'color': int(job['color']),
                'phase_start': float(job['since']),
            })
        remaining = [
            int(self._q_batch[index]) - int(self._feed_counts.get(COLORS[index], 0))
            for index in range(4)]
        choice, stats = decide_feed(
            self._q_config, self._q_table, remaining, arms, now,
            self._q_package_dir)
        from dual_arm_rl.model import Model
        from dual_arm_rl.qtable import state_key
        audit_config = dict(self._q_config, allow_prefeed_busy_arm=False)
        audit_model = Model(audit_config, remaining)
        audit_model.time = now
        for target, obs in zip(audit_model.arms, arms):
            target.phase = obs['phase']; target.color = obs['color']
            target.phase_start = obs['phase_start']
        if any(a.phase != 'READY' for a in audit_model.arms):
            audit_model._schedule(1.0, 'ARM_READY', 0)
        self.get_logger().info('Q_DECISION '+json.dumps({
            'state_key': state_key(audit_model.state()),
            'legal_actions': audit_model.legal_actions(), 'sim': now,
            'choice': choice, 'action': COLOR_INDEX[choice[1]] if choice else 4,
            'stats': stats, 'remaining': remaining, 'arms': arms}))
        if choice is None:
            return None, 'Q 来料: 这步选择等待,先不推新的一件'
        arm, _color = choice
        if arm not in idle:
            choice = choose_idle_feed(
                idle, self._last_feed_arm, self._feed_counts)
            note = ''
            if choice:
                note = f', 闲臂 {choice[0]}（Q表选了忙着的臂,不预送）'
            return choice, note
        note = f', Q表 {arm}'
        if stats.get('unknown_state_fallback_steps'):
            note += '（表里没有这个状态,改用规则）'
        return choice, note

    def _request_spawn_and_start_belt(self):
        """发一条来料请求并令推板前进;计数进 _spawned。

        并联时不发 random。接了 Q 表就问表,否则谁闲推谁。
        一次只占一个送料槽。取件区还被占、或已经有一件在送,这次直接返回。
        """
        if getattr(self,'_voice_mode',False):
            self._voice_request_spawn()
            return
        with self._lock:
            if self._halted:
                return
            if not self._demand_unlimited \
                    and self._spawned >= self._demand_parts:
                return
            color = self._demand_color
            feed_note = ''
            if self._per_arm_slots:
                if self._feed_inflight or self._seq.zone_arm is not None:
                    return
                if self._q_config is not None:
                    choice, feed_note = self._choose_q_feed_locked()
                else:
                    choice = choose_idle_feed(
                        self._seq.idle_arms(), self._last_feed_arm,
                        self._feed_counts)
                    feed_note = f', 闲臂 {choice[0]}' if choice else ''
                if choice is None:
                    if feed_note:
                        self.get_logger().info(feed_note)
                    return
                arm, color = choice
                self._last_feed_arm = arm
                self._feed_counts[color] = self._feed_counts.get(color, 0) + 1
                self._feed_inflight = True
            self._spawned += 1
            spawned = self._spawned
            if self._belt_control:
                self._destroy_settle_timer()
                self._belt.start()   # 复位本轮回位/settle,门控关闭
        with self._lock:
            if self._halted:return
            request = String()
            request.data = color
            if not self._first_feed_logged:
                self._first_feed_logged = True
                self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'first_feed','wall':time.monotonic()-self._first_feed_t0,'fast':self._fast_waits,'sim':self.get_clock().now().nanoseconds/1e9}))
            self._pub_spawn.publish(request)
            self.get_logger().info(
                f'按需来料: 已请求 {spawned}'
                + ('' if self._demand_unlimited
                   else f'/{self._demand_parts}')
                + f'({self._spawn_topic} <- {color}{feed_note})')
            if self._belt_control:
                cmd = Float64()
                cmd.data = self._belt_speed
                self._pub_belt.publish(cmd)
                self.get_logger().info(f'推板前进: {self._belt_speed} m/s')

    # ---------------------------------------------------------------- 推板控制
    def _prime_maybe_start(self, force=False):
        """prime 触发判定:桥就绪 + (来料满 3 件 或 force 兜底)。

        两条腿:① 桥就绪时刻到 -> 本方法非 force 判定,满 3 件即推;
        ② 满件前 arrival 钩子调本方法(force=False)提前试探;
        ③ 就绪 +15s 仍未满 3 件(spawn 病态)force 兜底照推,窗门控仍
        保护。未满足且非 force -> +2s 自检重试(arrival 钩子双保险)。
        _prime_started 保证 advance 只发一次(多路触发去重)。
        """
        self._prime_timer = None
        if self._prime_started:
            return
        if time.monotonic() < self._prime_t0 + self._prime_ready_at:
            # 桥未就绪:等就绪时刻的排程 timer,不自作主张推板
            return
        with self._lock:
            n = len(self._catalog)
            enough = n >= 3
        if not enough and not force:
            self._prime_timer = threading.Timer(
                2.0, lambda: self._prime_maybe_start(
                    time.monotonic() >= self._prime_t0
                    + self._prime_ready_at + 15.0))
            self._prime_timer.daemon = True
            self._prime_timer.start()
            return
        if not enough and force:
            self.get_logger().warn(
                f'L1 串推兜底触发:来料仅 {n} 件(未满 3),照常串推')
        self._prime_started = True
        self._prime_advance()

    def _prime_advance(self):
        """L1 串推链第 1 步:定时推进 belt_chain_advance_s(=5s)后回位。

        取证定案(2026-10-06 六轮):"贴板轻串"(observe 0.675 判停)
        反而制造新缺陷——链贴板后指压挤压集中到中间件,件被当"瓜籽"
        侧弹下带(r3 part_2 -> (0.895,-0.182) 出带 pre_grasp FAILED);
        定时 5s 把整条链匀速推过 2cm 矮板落入停放带(0.82-0.95,板后
        桌面无带面低摩擦),推行全程准静态无弹射(v1 三轮 12/12 PLACED
        实证),取件推碰不再沿带接力(件间零空档且已离带)。此后再无
        任何推进命令(legacy 无逐件来料,永不 second start)。
        """
        cmd = Float64()
        cmd.data = self._belt_speed
        self._pub_belt.publish(cmd)
        self.get_logger().info(
            f'[L1 串推] 推板前进 {self._belt_speed} m/s × '
            f'{self._belt_chain_advance_s}s(链推过矮板入停放带 0.82-0.95)')
        self._prime_timer = threading.Timer(
            self._belt_chain_advance_s, self._prime_retract)
        self._prime_timer.daemon = True
        self._prime_timer.start()

    def _prime_retract(self):
        """L1 串推链第 2 步:回位命令(同 pipeline 回位值 -0.25)。

        单一入口 = advance 定时 5s 到点;_prime_retracting 幂等闸保留
        (timer 重复触发/停机竞态防护)。
        """
        with self._lock:
            if self._prime_retracting:
                return
            self._prime_retracting = True
            self._prime_timer = None
        cmd = Float64()
        cmd.data = self._belt.retract_speed
        self._pub_belt.publish(cmd)
        self.get_logger().info(
            f'[L1 串推] 推板回位 {self._belt.retract_speed} m/s,'
            f'{self._belt_settle_delay}s 后停板放行派发')
        self._prime_timer = threading.Timer(
            self._belt_settle_delay, self._prime_settled)
        self._prime_timer.daemon = True
        self._prime_timer.start()

    def _prime_settled(self):
        """L1 串推链第 3 步:停板(0.0)并解除派发挂起。此后推板零命令。"""
        cmd = Float64()
        cmd.data = self._belt.stop_speed
        self._pub_belt.publish(cmd)
        self._prime_timer = None
        with self._lock:
            self._priming = False
        self.get_logger().info('[L1 串推] 停板完成,链已串紧,放行派发')
        self._try_dispatch()

    def _destroy_prime_timer(self):
        if self._prime_timer is not None:
            self._prime_timer.cancel()
            self._prime_timer = None

    def _pusher_world_x(self, msg: LogicalCameraImage) -> float | None:
        """逻辑相机里 pusher 模型的世界 x。看不见时返回 None。"""
        cam_xy = self._cam_cache.camera_xy
        for model in msg.model:
            if model.name != 'pusher':
                continue
            position = model.pose.position
            return world_xy_from_rel(
                cam_xy, position.x, position.y, position.z)[0]
        return None

    def _on_logical(self, msg: LogicalCameraImage):
        """判停推进:未完全终态工件 world_x 越线 -> 发推板回位命令一次。

        已终态工件(已入箱,x>=0.56)被排除,否则从第二件起会在推进
        开始瞬间误触发回位;world_x 换算见 camera_pose 模块(rel_z 分量)。
        回位命令(-0.25,不是 0.0)之后,用同一条消息里的 pusher 世界坐标
        等它回到起点附近再发板停;时间戳用消息上的仿真时间做兜底,
        不再用墙钟 2.6 秒。
        """
        self._startup_logical_wall = time.monotonic()
        if getattr(self,'_voice_mode',False):
            with self._lock:
                self._voice_visible={m.name for m in msg.model if m.name.startswith('part_')}
                self._voice_logical_wall=time.monotonic()
                self._voice_logical_sim=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self._startup_pusher=self._pusher_world_x(msg)
                self._voice.observe_entities(self._voice_visible)
                self._voice_flush_locked()
                if self._halted:return
        self._startup_pusher = self._pusher_world_x(msg)
        self._cam_cache.update(
            (m.name, m.pose.position.x, m.pose.position.y, m.pose.position.z)
            for m in msg.model)
        if self._per_arm_slots:
            # 只看这一件正在送的工件。上一件已经离区、还在搬运时,
            # 它的坐标不能再触发停推进。
            part = self._belt_part_id
            names = ([part] if part and part not in self._off_belt
                     and part not in self._finished_parts else [])
        else:
            names = [name for name in self._cam_cache.known_parts()
                     if name not in self._finished_parts]
        xs = []
        for name in names:
            xy = self._cam_cache.get(name)
            if xy is not None:
                xs.append(xy[0])
        sim_now = (float(msg.header.stamp.sec)
                   + float(msg.header.stamp.nanosec) * 1e-9)
        pusher_x = self._pusher_world_x(msg)
        with self._lock:
            # L1 legacy 轮旁路:prime 定时链自带回位,旧 observe 判停在
            # legacy 出生帧(0.58/0.66 在 0.58 线上/前)会首帧误触发,
            # 与 prime 链打架 -> _legacy_prime 时不喂 _belt.observe。
            cmd_value = (None if self._legacy_prime
                         else self._belt.observe(xs, sim_now))
            stop_value = None
            stop_reason = ''
            if not self._legacy_prime:
                stop_value = self._belt.observe_pusher(pusher_x, sim_now)
                if stop_value is not None:
                    stop_reason = self._belt.home_reason
        if cmd_value is not None:
            # 取证(2026-10-06 问题①):触发帧打印参与判停的全部件坐标
            det = {}
            for n in self._cam_cache.known_parts():
                xy = self._cam_cache.get(n)
                det[n] = round(xy[0], 3) if xy else None
            self.get_logger().info(
                f'[停推进取证] xs={sorted(round(x, 3) for x in xs)} '
                f'known={det} finished={sorted(self._finished_parts)} '
                f'pusher_x={None if pusher_x is None else round(pusher_x, 3)}')
            cmd = Float64()
            cmd.data = cmd_value
            self._voice_safe_belt(cmd)
            self._destroy_settle_timer()
            self.get_logger().info(
                f'停推进: 工件 world_x >= {self._belt_stop_x},'
                f'推板回位({cmd_value} m/s),'
                f'等到推板回到起点 x<='
                f'{self._belt.home_x + self._belt.home_tol:.2f} 再停板')
        if stop_value is not None:
            cmd = Float64()
            cmd.data = stop_value
            self._voice_safe_belt(cmd)
            shown = '无' if pusher_x is None else f'{pusher_x:.3f}'
            if stop_reason == 'position':
                self.get_logger().info(
                    f'推板已回到起点附近(world_x={shown}),停板并允许派发')
            else:
                self.get_logger().warn(
                    f'推板回位已过仿真时间 {self._belt.home_timeout_sim:.0f}s'
                    f'仍未到起点(world_x={shown}),按仿真时间停板并允许派发')
            self._try_dispatch()

    def _schedule(self, actions):
        if self._halted:return
        """按 RelaySequencer 给出的延迟动作起墙钟 timer(见模块注释)。"""
        for kind, delay in actions:
            if kind == RELAY:
                self._destroy_relay_timer()
                self._relay_timer = threading.Timer(delay, self._send_relay)
                self._relay_timer.daemon = True
                self._relay_timer.start()
            elif kind == DISPATCH:
                self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'dispatch_delay_path','configured':delay,'fast':self._fast_waits,'per_arm':self._per_arm_slots}))
                if self._fast_waits and self._per_arm_slots:
                    self._try_dispatch()  # existing atomic slot/zone/belt/dedup gates
                    continue
                if self._next_timer is None:
                    self._next_timer = threading.Timer(delay, self._try_dispatch)
                    self._next_timer.daemon = True
                    self._next_timer.start()

    # ---------------------------------------------------------------- 派发
    def _try_dispatch(self):
        self._destroy_next_timer()
        if self._halted:      # R1-P1 停轮闸(见 halt_on_wedge)
            return
        with self._lock:
            if self._halted:return
            # L1 方案 a(仅 legacy prime 链激活):串推未完成前挂起派发
            if self._priming:
                self.get_logger().info('派发挂起: L1 串推链进行中')
                return
            # belt_control=true 的门控:推板未回位停稳则挂起,等 settle
            # 完成事件或下一次 arrival 再触发(契约 §6 步骤 5)。
            # legacy 轮 BeltController.start() 永不触发(无逐件来料),
            # _running=False -> 本门控恒放行;pipeline 行为零差。
            if (self._belt_control
                    and not self._belt.may_dispatch(time.monotonic())):
                self.get_logger().info('派发挂起: 等待推板回位停稳(settle)')
                return
            # 并联分色:按队首选件类别决定执行臂(peek 不出队,真正
            # 的出队与阶段登记在 try_dispatch(arm) 一次完成)。
            # 指定箱派发(2026-10-06)优先于按色路由:该件有
            # target_bin 覆盖时按 bin_arm_map 定臂;映射表未含该箱
            # (bin_arm_map 为空)则维持按色路由,箱名交由执行器校验。
            next_id = self._seq.peek()
            # ---- L1 方案 a②:停位窗派发闸(仅 legacy prime 轮激活;
            # 每拍现场读缓存,复检只由 2s 保险 timer 驱动——越窗件若
            # 不再被相机刷新,计数兜底放行,绝不因缺观测死锁)----
            if (self._legacy_prime and next_id is not None
                    and self._cam_cache is not None):
                xy = self._cam_cache.get(next_id)
                wx = xy[0] if xy else None
                verdict, note = self._gate.decide(next_id, wx)
                if verdict == 'hold':
                    self.get_logger().info(f'派发挂起(停位窗): {note}')
                    if self._window_timer is None:
                        self._window_timer = threading.Timer(
                            self._recheck_s, self._window_recheck)
                        self._window_timer.daemon = True
                        self._window_timer.start()
                    return
                if verdict == 'force_allow':
                    self.get_logger().warn(
                        f'停位窗兜底放行(挂起超限): {note}'
                        '(该件将按既有通道执行,NO_IK 读数不失真)')
            to_arm2 = (next_id is not None and self._arm2_colors
                       and self._catalog[next_id].category
                       in self._arm2_colors)
            if next_id is not None and next_id in self._bin_overrides:
                ovr_arm = self._bin_arm_map.get(
                    self._bin_overrides[next_id])
                if ovr_arm is not None:
                    to_arm2 = ovr_arm == 'arm2'
            part_id = self._seq.try_dispatch(
                'arm2' if to_arm2 else 'arm1')
            command = None
            if part_id is None and next_id is not None and self._per_arm_slots:
                self.get_logger().info(
                    f'派发挂起: {next_id} 取件区='
                    f'{self._seq.zone_arm or "空"} '
                    f'arm1槽={self._seq.slot("arm1") or "空"} '
                    f'arm2槽={self._seq.slot("arm2") or "空"}')
            if part_id is not None:
                command = self._catalog[part_id]
                color_index = COLOR_INDEX.get(command.category)
                if color_index is not None:
                    self._arm_post['arm2' if to_arm2 else 'arm1'] = {
                        'color': color_index, 'since': None}
                ovr = self._bin_overrides.get(part_id, '')
                if ovr and command.target_bin_override != ovr:
                    # 派发副本注入覆盖(不污染缓存:缓存即命令本身时
                    # 已在服务回调写入,这里仅兜底同步)
                    command.target_bin_override = ovr
        with self._lock:
            if self._halted:return
            if command is not None:
                ovr_note = (f' [指定箱 {command.target_bin_override}]'
                            if getattr(command, 'target_bin_override', '')
                            else '')
                if to_arm2:
                    self._pub_relay.publish(command)
                    self.get_logger().info(
                        f'并联派发: {part_id} (类别 {command.category})'
                        f' -> arm2{ovr_note}'
                        + ('，取件区上锁' if self._per_arm_slots else ''))
                else:
                    self._pub_command.publish(command)
                    self.get_logger().info(
                        f'派发任务: {part_id} (类别 {command.category})'
                        f'{ovr_note}'
                        + ('，取件区上锁' if self._per_arm_slots else ''))

    # ------------------------------------------------ independent voice batch
    def _voice_flush_locked(self):
        if self._voice.state=='HALTED':
            self._halted=True
            self._destroy_spawn_timer();self._destroy_next_timer();self._destroy_relay_timer();self._destroy_settle_timer()
            if self._pub_belt is not None:
                stop=Float64();stop.data=0.;self._pub_belt.publish(stop)
        while self._voice.events:
            event=self._voice.events.pop(0)
            self.get_logger().info('VOICE_BATCH '+json.dumps({'wall':time.time(),'sim':self.get_clock().now().nanoseconds/1e9,**event},ensure_ascii=False))
        message=String();message.data=json.dumps(self._voice.snapshot(),ensure_ascii=False)
        self._voice_pub.publish(message)

    def _voice_on_submit(self,request,response):
        from a1_sorting_core.voice_batch_plan import validate_candidate,PlanRejected
        try:
            plan=validate_candidate(json.dumps({'schema_version':1,'action':'sort_batch','counts':dict(zip(('blue','yellow','red','green'),(request.blue,request.yellow,request.red,request.green))),'priority_first':request.priority_first,'proposed_order':list(request.proposed_order)}),max_per_color=4 if self._four_slot else 3)
            if self._trial_permit and (plan.counts!=getattr(self,'_trial_counts',(0,1,0,0)) or plan.order!=(('yellow',) if plan.total==1 else ('yellow','red','green','green','green'))):raise PlanRejected('single_yellow_trial_only')
            from a1_sorting_core.voice_batch_safety import ensure_geometry_review
            if not self._four_slot:ensure_geometry_review(plan)
            with self._lock:
                self._voice.accept(request.session_id,request.request_id,request.user_confirmed,plan,request.plan_sha256,time.monotonic())
                self._voice_flush_locked()
            response.accepted=True;response.message='confirmed plan accepted; feed remains readiness/slot/vision/recycle gated'
            self._request_spawn_and_start_belt()
        except PlanRejected as exc:
            response.accepted=False;response.message=str(exc)
        return response

    def _voice_on_stop(self,request,response):
        with self._lock:
            self._voice.halt('operator_stop');self._voice_flush_locked()
        response.success=True;response.message='New feeding/dispatch halted; active motion requires supervised pause/cleanup, not an emergency stop guarantee'
        return response

    def _voice_on_recycle(self,msg):
        try:
            record=json.loads(msg.data)
            if set(record)!={'part_id','deleted','sim_time'} or record['deleted'] is not True:raise ValueError('invalid recycle ACK')
            with self._lock:
                if type(record['sim_time']) not in (int,float) or not math.isfinite(record['sim_time']) or record['sim_time']<0:raise ValueError('invalid recycle timestamp')
                self._voice.recycle(record['part_id'],time.monotonic(),record['sim_time']);self._voice_flush_locked()
        except (ValueError,TypeError,KeyError):
            with self._lock:self._voice.halt('invalid_recycle_ack');self._voice_flush_locked()

    def _voice_safe_belt(self,command):
        if not self._voice_mode:
            self._pub_belt.publish(command);return
        with self._lock:
            if not self._halted or command.data==0.:self._pub_belt.publish(command)

    def _voice_request_spawn(self):
        with self._lock:
            if self._halted:return
            now=time.monotonic()
            home=getattr(self,'_startup_pusher',None)
            can_feed=(not self._feed_inflight and not self._seq.busy_parts() and self._seq.zone_arm is None and home is not None and home<=.14 and now-self._startup_rgb_wall<1. and now-self._voice_logical_wall<1.)
            color=self._voice.reserve(now,can_feed,self._voice_visible,self._voice_logical_wall,getattr(self,'_voice_logical_sim',None))
            if color is None:return
            self._spawned+=1;self._feed_counts[color]+=1;self._feed_inflight=True
            self._destroy_settle_timer();self._belt.start()
            self._voice_flush_locked()
            request=String();request.data=color;self._pub_spawn.publish(request)
            command=Float64();command.data=self._belt_speed;self._pub_belt.publish(command)

    def _voice_watchdog(self):
        while rclpy.ok() and not getattr(self,'_voice_stopping',False):
            time.sleep(.2)
            if not rclpy.ok() or getattr(self,'_voice_stopping',False):return
            with self._lock:
                if self._halted and self._voice.state!='HALTED':self._voice.halt('base_dispatcher_halted')
                self._voice.check_timeout(time.monotonic());self._voice_flush_locked()
            self._request_spawn_and_start_belt()

    def destroy_node(self):
        if getattr(self,'_voice_mode',False):
            self._voice_stopping=True
            for name in ('_voice_watchdog_thread','_startup_ready_thread'):
                thread=getattr(self,name,None)
                if thread is not None and thread is not threading.current_thread():thread.join(timeout=3.)
        return super().destroy_node()

    def _window_recheck(self):
        """停位窗越窗后的复检触发(2s 墙钟;窗口自愈=件被碰回带内,
        或计数到上限兜底放行)。"""
        self._window_timer = None
        self._try_dispatch()

    def _send_relay(self):
        self._destroy_relay_timer()
        with self._lock:
            if self._halted:return
            part_id = self._seq.on_relay_ready()
            command = self._catalog[part_id] if part_id is not None else None
        with self._lock:
            if self._halted:return
            if command is not None:
                self._pub_relay.publish(command)
                self.get_logger().info(
                    f'派发接力任务: {part_id} -> arm2 (类别 {command.category})')

    def _destroy_next_timer(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None

    def _destroy_relay_timer(self):
        if self._relay_timer is not None:
            self._relay_timer.cancel()
            self._relay_timer = None

    def _destroy_spawn_timer(self):
        if self._spawn_timer is not None:
            self._spawn_timer.cancel()
            self._spawn_timer = None

    def _destroy_settle_timer(self):
        if self._settle_timer is not None:
            self._settle_timer.cancel()
            self._settle_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = TaskDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
