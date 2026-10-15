"""流水线模式任务层纯逻辑(与 ROS 解耦,便于单元测试)。

队员C 的流水线集成点(契约 §4/§6/§9)中可脱离 rclpy 测试的部分抽到
本模块;节点文件(pick_place_executor_node / task_dispatcher_node)只做
ROS 桥接。本仓库的单测跑法不 source ROS(PYTHONPATH=. python3 -m
pytest test/),纯逻辑必须放在无 rclpy 依赖的模块里。

- place_descend_target_z:放置段最后下探的 tool0 目标高度
  (place_drop_clearance 落入模式,契约 §9);
- verify_probe_targets:夹取验证水平探针的往返目标坐标(组长仲裁
  方案 B:纯竖直抬升无法证伪"没夹住",探针给出 >gripper_move_thresh
  的水平位移窗);
- BeltController:dispatcher 的推板推进/回位/停板与派发门控状态机
  (契约 §4 推板方案:哑带 + pusher 真实接触推进,替代已实证放弃的
  TrackController 表面速度方案;参数名沿用 belt_* 以减少 yaml 面积)。

线程约定:BeltController 自身不加锁(logical 回调与 threading.Timer
线程的并发访问由 task_dispatcher_node 的锁保护);时间统一用墙钟秒,
时间戳由调用方注入(float),便于单元测试(sim-time 定时不可靠,见
task_dispatcher_node 模块注释)。
"""


# 谁闲推谁:蓝/黄只给 arm1,红/绿只给 arm2。和 sorting_params 的分工一致。
ARM_FEED_COLORS = {
    'arm1': ('blue', 'yellow'),
    'arm2': ('red', 'green'),
}


def choose_idle_feed(idle_arms, last_arm, counts):
    """给空闲的臂选下一次来料颜色。没有空闲臂时返回 None。

    两台都闲:优先不是上一件那台,这样另一台还能接着放的时候,
    这台去取。都没接过活时先 arm1。
    同一台的两种颜色:先选已经请求得少的那种,一样少就按蓝黄、红绿的顺序。
    counts 只统计已经发出的请求,调用方负责加一。
    """
    idle = [arm for arm in ('arm1', 'arm2') if arm in idle_arms]
    if not idle:
        return None
    if len(idle) == 2 and last_arm == 'arm1':
        arm = 'arm2'
    elif len(idle) == 2 and last_arm == 'arm2':
        arm = 'arm1'
    else:
        arm = idle[0]
    colors = ARM_FEED_COLORS[arm]
    color = min(colors, key=lambda name: (int(counts.get(name, 0)),
                                           colors.index(name)))
    return arm, color


def place_descend_target_z(place_z: float, tcp_z: float,
                           drop_clearance: float) -> float:
    """放置段最后下探终点(tool0 高度)。

    drop_clearance > 0(落入模式,契约 §9):终点 = place_z + tcp_z +
    drop_clearance,不下探到放置面;到位松爪让工件脱附自由落入箱内,
    四壁约束防弹飞。松爪时工件中心高度 = place_z + drop_clearance。
    drop_clearance <= 0(默认 0.0):终点 = place_z + tcp_z,下探到
    放置面松爪(旧行为,完全回归;负值视为关闭)。
    """
    base = place_z + tcp_z
    if drop_clearance > 0.0:
        return base + drop_clearance
    return base


def verify_probe_targets(center_xy_z, tcp_z: float, lift_h: float,
                         probe_offset: float):
    """夹取验证水平探针的往返目标(世界系 tool0 位姿目标,契约 §8)。

    组长仲裁方案 B:verified_only 模式下抬升完成后、verify 调用前,
    先 LIN 到 (center_x - probe_offset, center_y) 再 LIN 回取件中心
    (探针取 -x 带尾方向:+x 紧邻挡停板,会把贴指的工件怼进板里挤脱,
    给 GraspVerifier 一个夹爪水平位移 > gripper_move_thresh 的窗口
    (默认 0.04 > 0.03):夹住的工件跟随 ~0.04 判通过,没夹住的留在
    原地(位移 ~0 < follow_move_thresh)判失败。

    返回 ((去程 x, y, z), (回程 x, y, z)),高度取抬升终点
    (center_z + tcp_z + lift_h);probe_offset <= 0 返回 None(禁用
    探针,负值视为关闭)。
    """
    if probe_offset <= 0.0:
        return None
    z = center_xy_z[2] + tcp_z + lift_h
    out = (center_xy_z[0] - probe_offset, center_xy_z[1], z)
    back = (center_xy_z[0], center_xy_z[1], z)
    return out, back


class BeltController:
    """推板推进/回位/停板与派发门控纯逻辑(dispatcher 专用,契约 §4)。

    带驱动已切换为推板真实接触推进(dartsim 上 TrackController 表面
    速度不稳定,已实证放弃):哑带 + pusher(prismatic x + gz
    JointController,速度话题 /model/pusher/joint/pusher_slide/cmd_vel)。
    dispatcher 侧的 belt_* 参数名沿用、语义升级为推板控制。

    一轮生命周期(每件来料一轮,三条命令值互不相同,不得合并——
    回位不是停板,停板在回位完成之后):

        start()           推板前进:复位本轮状态,门控关闭
                          (节点发 advance_speed,推板 +x 推工件);
        observe(xs, now)  每帧喂"参与判停的工件 world_x 列表":
                          首次越过 stop_x 的那一帧返回 retract_speed
                          (节点发回位命令并打"停推进"日志,板 -x 回位),
                         其余帧返回 None(回位命令只发一次);
        observe_pusher    回位命令发出后,每帧喂推板自身 world_x 与仿真
                          时间:首次回到起点附近返回 stop_speed(板停)。
                          仿真时间超过 home_timeout_sim 仍未到起点时也
                          返回 stop_speed,作为收不到位姿时的兜底。
                          may_dispatch 只在这一步之后放行。

    说明:
    - 已终态工件的过滤(已入箱件 x>=0.56 不参与判停推进,否则从第二
      件起会在推进开始瞬间误触发回位)是节点侧职责,本类只看数值;
    - 线程约定:本类自身不加锁(logical 回调与 timer 线程的并发访问
      由节点侧锁保护);observe / observe_pusher 的 now 由调用方注入,
      流水线路径必须传仿真时间,不能传墙钟。
    - settle_delay 仍保留给 legacy 串推链的墙钟定时器,流水线回位
      不再用它放行。起点是 SDF 里 pusher 模型 pose x=0.12、关节
      下限 0,世界坐标 0.12 就是关节回到起点。
    """

    def __init__(self, stop_x: float, settle_delay: float,
                 advance_speed: float = 0.1,
                 retract_speed: float = -0.25,
                 stop_speed: float = 0.0,
                 home_x: float = 0.12,
                 home_tol: float = 0.02,
                 home_timeout_sim: float = 12.0):
        self._stop_x = float(stop_x)
        self._settle_delay = max(0.0, float(settle_delay))
        self.advance_speed = float(advance_speed)  # 前进命令值(m/s,推板 +x)
        self.retract_speed = float(retract_speed)  # 回位命令值(m/s,负)
        self.stop_speed = float(stop_speed)        # 回到起点后板停命令值
        self._home_x = float(home_x)
        self._home_tol = max(0.0, float(home_tol))
        self._home_timeout_sim = max(0.0, float(home_timeout_sim))
        self._running = False        # 是否已开始推进(start 后保持 True)
        self._retracted = False      # 本轮已发回位命令
        self._retract_at: float | None = None   # 回位命令时刻(仿真时间)
        self._at_home = False
        self.home_reason = ''        # 'position' 或 'sim_timeout'

    # ------------------------------------------------------------- 只读视图
    @property
    def stop_x(self) -> float:
        return self._stop_x

    @property
    def settle_delay(self) -> float:
        return self._settle_delay

    @property
    def running(self) -> bool:
        return self._running

    @property
    def retracted(self) -> bool:
        return self._retracted

    @property
    def home_x(self) -> float:
        return self._home_x

    @property
    def home_tol(self) -> float:
        return self._home_tol

    @property
    def home_timeout_sim(self) -> float:
        return self._home_timeout_sim

    @property
    def at_home(self) -> bool:
        return self._at_home

    # ---------------------------------------------------------------- 推进
    def start(self) -> None:
        """推板前进(新一件来料):复位本轮回位状态,门控关闭。"""
        self._running = True
        self._retracted = False
        self._retract_at = None
        self._at_home = False
        self.home_reason = ''

    def observe(self, world_x_values, now: float) -> float | None:
        """喂一帧工件 world_x 采样;返回本帧应发布的命令值。

        仅"首次越过 stop_x"的那一帧返回 retract_speed(节点发回位
        命令一次);其余帧(含越线后的持续帧)返回 None。
        """
        if self._running and not self._retracted:
            if any(float(x) >= self._stop_x for x in world_x_values):
                self._retracted = True
                self._retract_at = float(now)
                return self.retract_speed
        return None

    def observe_pusher(self, world_x, now: float) -> float | None:
        """回位过程中喂推板 world_x。回到起点附近,或仿真时间超时,返回停板命令一次。"""
        if not self._running or not self._retracted or self._at_home:
            return None
        # 1e-9 只抵消 0.12+0.02 这类二进制小数,不放宽 2cm 容差。
        if (world_x is not None
                and float(world_x) <= self._home_x + self._home_tol + 1e-9):
            self._at_home = True
            self.home_reason = 'position'
            return self.stop_speed
        if (self._retract_at is not None
                and float(now) - self._retract_at >= self._home_timeout_sim):
            self._at_home = True
            self.home_reason = 'sim_timeout'
            return self.stop_speed
        return None

    def may_dispatch(self, now: float) -> bool:
        """派发门控:未推进恒放行;推进后要等推板回到起点(或仿真时间兜底)。

        now 保留是为了调用点不用改。墙钟走完 settle_delay 不再放行。
        """
        del now
        if not self._running:
            return True
        if not self._retracted:
            return False
        return self._at_home


def dispatch_window_ok(world_x, lo: float, hi: float) -> bool:
    """legacy 停位窗派发闸(L1 方案 a,2026-10-06 用户拍板)。

    world_x = 队首件在逻辑相机缓存里的最新 world_x:
    - None(尚无观测,如相机未刷新/件不在视野)-> 放行:缺观测由
      执行器既有 no_camera_pose 重试通道兜底,调度器不再加一层死锁;
    - lo <= x <= hi -> 放行(件在带面停位/出生带内);
    - 越窗(如 L1 漂移样本 0.94-1.12)-> False,调用方挂起复检。
    本函数为纯判断;挂起上限与兜底放行(绝不卡轮)在节点侧计数。
    """
    if world_x is None:
        return True
    return float(lo) <= float(world_x) <= float(hi)


class DispatchWindowGate:
    """停位窗门控 + 越窗复检计数(纯逻辑,L1 方案 a;线程约定同
    BeltController:节点锁保护,时间/决策由调用方落地)。

    decide(part_id, world_x) 返回 (verdict, note):
    - 'allow':      在窗内(或尚无观测,note='no-obs'),计数清零;
    - 'hold':       越窗且复检未超上限,调用方挂起 belt_dispatch_recheck_s
                    后重查;
    - 'force_allow':连续越窗超过 wait_max 次 -> 兜底放行(绝不卡轮),
                    计数清零,调用方须打告警(该件多半会走执行器
                    NO_IK 既有失败通道,读数不失真)。
    计数器按 part_id 独立;件完全终态后调用方 forget(part_id) 清账。
    """

    def __init__(self, lo: float, hi: float, wait_max: int):
        self._lo = float(lo)
        self._hi = float(hi)
        self._wait_max = max(0, int(wait_max))
        self._waits: dict[str, int] = {}

    def decide(self, part_id: str, world_x):
        if dispatch_window_ok(world_x, self._lo, self._hi):
            self._waits.pop(part_id, None)
            return ('allow', 'no-obs' if world_x is None
                    else f'x={world_x:.3f}')
        n = self._waits.get(part_id, 0) + 1
        self._waits[part_id] = n
        note = f'{part_id} world_x={float(world_x):.3f} 越窗 ' \
               f'[{self._lo:.2f},{self._hi:.2f}] 第 {n}/{self._wait_max} 次'
        if n > self._wait_max:
            self._waits.pop(part_id, None)
            return ('force_allow', note)
        return ('hold', note)

    def forget(self, part_id: str) -> None:
        self._waits.pop(part_id, None)
