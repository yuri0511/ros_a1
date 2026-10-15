"""流水线模式任务层纯逻辑(pipeline_logic)单元测试:不初始化 rclpy。

覆盖(契约 §4 推板方案 + §9 + 组长仲裁方案 B):
- place_descend_target_z:place_drop_clearance 落入模式的放置下探终点;
- verify_probe_targets:夹取验证水平探针的往返目标坐标;
- BeltController:推板 回位命令时机(-0.25)/ 板停命令(0.0)/
  settle 门控 / 每件一轮的复位。
时间戳由测试注入(流水线路径按仿真时间理解),不依赖真实定时器。
"""
import pytest

from a1_sorting_core.pipeline_logic import (BeltController,
                                            place_descend_target_z,
                                            verify_probe_targets)


# ------------------------------------------------------------ 放置下探终点 z
def test_choose_idle_feed_uses_the_free_arm():
    from a1_sorting_core.pipeline_logic import choose_idle_feed
    assert choose_idle_feed(['arm1', 'arm2'], None, {}) == ('arm1', 'blue')
    assert choose_idle_feed(['arm1', 'arm2'], 'arm1', {'blue': 1}) == (
        'arm2', 'red')
    assert choose_idle_feed(['arm1'], 'arm2', {'blue': 1}) == (
        'arm1', 'yellow')
    assert choose_idle_feed(['arm2'], 'arm1', {'red': 1, 'green': 1}) == (
        'arm2', 'red')
    assert choose_idle_feed([], None, {}) is None


def test_place_drop_clearance_zero_is_legacy():
    """clearance=0:终点 = place_z + tcp_z,下探到放置面(旧行为)。"""
    assert place_descend_target_z(0.756, 0.211, 0.0) == pytest.approx(0.967)
    assert place_descend_target_z(0.745, 0.231, 0.0) == pytest.approx(0.976)
    assert place_descend_target_z(0.75, 0.10, 0.0) == pytest.approx(0.85)


def test_place_drop_clearance_positive_stops_short():
    """>0:终点抬高 clearance(松爪时工件中心 = place_z + clearance)。"""
    z = place_descend_target_z(0.745, 0.231, 0.06)   # 契约 §9 起调值
    assert z == pytest.approx(0.745 + 0.231 + 0.06)  # 1.036
    assert z - 0.231 == pytest.approx(0.805)         # 工件中心高(箱口上)
    assert place_descend_target_z(0.756, 0.211, 0.02) == pytest.approx(0.987)


def test_place_drop_clearance_negative_treated_as_legacy():
    """负值视为关闭(与 0 同):不会算出低于放置面的终点。"""
    assert (place_descend_target_z(0.745, 0.231, -0.05)
            == place_descend_target_z(0.745, 0.231, 0.0))
    assert (place_descend_target_z(0.756, 0.211, -1e-9)
            == place_descend_target_z(0.756, 0.211, 0.0))


# ------------------------------------------------------------ 水平验证探针
def test_verify_probe_targets_roundtrip():
    """默认偏移:去程 -x 偏移(带尾方向,避开挡停板)、回程回取件中心,
    高度=抬升终点 tool0 z。"""
    out, back = verify_probe_targets((0.44, -0.01, 0.805),
                                     tcp_z=0.231, lift_h=0.06,
                                     probe_offset=0.04)
    assert out == pytest.approx((0.40, -0.01, 0.805 + 0.231 + 0.06))
    assert back == pytest.approx((0.44, -0.01, 1.096))
    # 仲裁依据:探针位移必须超过验证器夹爪阈值(0.04 > 0.03)
    assert abs(out[0] - back[0]) == pytest.approx(0.04)
    assert out[1] == back[1] and out[2] == back[2]


def test_verify_probe_targets_disabled():
    """offset=0 或负值 -> None(禁用探针,不产生任何运动)。"""
    assert verify_probe_targets((0.44, 0.0, 0.805), 0.231, 0.06, 0.0) is None
    assert verify_probe_targets((0.44, 0.0, 0.805), 0.231, 0.06,
                                -0.04) is None


# ------------------------------------------------------------ 推板控制/门控
def make_belt(stop_x: float = 0.40, settle_delay: float = 2.6
              ) -> BeltController:
    return BeltController(stop_x=stop_x, settle_delay=settle_delay)


def test_pusher_not_started_gate_open_and_no_retract():
    """未推进:门控放行(belt_control 与 demand_spawn 自由组合的兼容),
    且任何采样都不触发回位(推板不归本控制器管)。"""
    b = make_belt()
    assert b.running is False
    assert b.may_dispatch(now=0.0) is True
    assert b.observe([0.45], 0.1) is None      # 越线也不回位(未推进)
    assert b.retracted is False


def test_pusher_crossing_retracts_once_with_command_value():
    """首帧越线返回回位命令值 -0.25(不是 0.0);只发一次;回到起点才放行。"""
    b = make_belt()                            # stop_x=0.40, home=0.12
    b.start()
    assert b.may_dispatch(now=0.5) is False    # 推进中未回位:门控关闭
    assert b.observe([0.30, 0.38], 1.0) is None
    assert b.observe([0.40], 2.0) == -0.25     # 首帧越线 -> 回位命令
    assert b.retracted is True
    assert b.observe([0.42, 0.50], 2.1) is None    # 不重复发
    assert b.observe([0.40], 2.2) is None
    assert b.may_dispatch(now=1000.0) is False  # 墙钟走完也不放行
    assert b.observe_pusher(0.30, 3.0) is None
    assert b.observe_pusher(0.14, 3.2) == 0.0   # 起点 0.12 + 容差 0.02
    assert b.home_reason == 'position'
    assert b.may_dispatch(now=3.2) is True
    assert b.observe_pusher(0.12, 3.3) is None  # 停板命令只发一次


def test_pusher_command_values_distinct_and_settle_covers_travel():
    """三条命令值互不相同(前进/回位/停板);settle 时延覆盖回位行程
    (0.3m @ 0.25 = 1.2s)+ 余量(契约 §4 推板方案)。"""
    b = make_belt()
    assert b.advance_speed == 0.1
    assert b.retract_speed == -0.25
    assert b.stop_speed == 0.0
    assert len({b.advance_speed, b.retract_speed, b.stop_speed}) == 3
    assert b.settle_delay >= 0.3 / 0.25 + 1.0  # 回位 1.2s + 余量


def test_pusher_restart_rearms_cycle():
    """下一件 start() 复位状态:重新判停、门控重新关闭(逐件循环)。"""
    b = make_belt()
    b.start()
    assert b.observe([0.40], 1.0) == -0.25
    assert b.observe_pusher(0.12, 2.0) == 0.0
    assert b.may_dispatch(now=2.0) is True
    b.start()                                  # 下一件:推板再前进
    assert b.may_dispatch(now=3.7) is False    # 门控重新关闭
    assert b.observe([0.20], 3.8) is None      # 新件未到停推进线
    assert b.observe([0.41], 5.0) == -0.25     # 新件越线再回位
    assert b.observe_pusher(0.50, 6.0) is None
    assert b.may_dispatch(now=7.61) is False
    assert b.observe_pusher(0.13, 7.0) == 0.0
    assert b.may_dispatch(now=7.0) is True


def test_pusher_boundary_threshold_and_home_edge():
    """恰好等于 stop_x 判停推进(含边界);恰好等于起点容差才停板。"""
    b0 = BeltController(stop_x=0.40, settle_delay=2.6)
    b0.start()
    assert b0.observe([0.3999], 1.0) is None   # 严格小于不回位
    assert b0.observe([0.40], 1.0) == -0.25    # 命中阈值
    assert b0.may_dispatch(now=1.0) is False   # 已回位命令,人还没到起点
    assert b0.observe_pusher(0.141, 1.2) is None
    assert b0.observe_pusher(0.14, 1.3) == 0.0
    assert b0.may_dispatch(now=1.3) is True
    b1 = make_belt()
    b1.start()
    assert b1.observe([], 0.0) is None         # 空帧不回位


def test_pusher_sim_timeout_opens_when_pose_never_homes():
    """仿真时间兜底:位姿一直在前面时,到 home_timeout_sim 才停板。"""
    b = BeltController(stop_x=0.40, settle_delay=2.6, home_timeout_sim=6.0)
    b.start()
    assert b.observe([0.40], 10.0) == -0.25
    assert b.observe_pusher(0.70, 15.9) is None
    assert b.observe_pusher(None, 16.0) == 0.0
    assert b.home_reason == 'sim_timeout'
    assert b.may_dispatch(now=16.0) is True


# ================================================= L1 方案 a:停位窗派发闸
# (2026-10-06 拍板;纯逻辑 DispatchWindowGate/dispatch_window_ok,
#  节点只做 ROS 桥接与 timer 落地)
from a1_sorting_core.pipeline_logic import (DispatchWindowGate,
                                            dispatch_window_ok)

# 实配窗口(legacy 定案基表,2026-10-06 取证修订):串推 5s 定时链把三件
# 匀速推过 2cm 矮板入板后停放带(实测 0.67-0.95,y≈0,可取——12/12
# PLACED 实证);窗 [0.40,1.00]:停放带+出生带恒放行,>1.00 = L1 真死区
# (NO_IK 样本 1.00-1.12)越窗挂起,<0.40 = 串推失败留带尾,同拦。
LO, HI = 0.40, 1.00


def test_window_ok_boundary_and_no_obs():
    """窗判定:含边界放行;None(尚无观测)放行——no_camera_pose 由
    执行器重试通道兜底,调度器不加死锁;越窗 False。"""
    assert dispatch_window_ok(LO, LO, HI) is True
    assert dispatch_window_ok(HI, LO, HI) is True
    assert dispatch_window_ok(0.5, LO, HI) is True
    assert dispatch_window_ok(None, LO, HI) is True
    assert dispatch_window_ok(0.34, LO, HI) is False   # 下越窗(串推失败留带尾=异常)
    assert dispatch_window_ok(0.95, LO, HI) is True    # 停放带最远样本放行
    assert dispatch_window_ok(1.00, LO, HI) is True    # 窗上沿(含边界)
    assert dispatch_window_ok(1.12, LO, HI) is False   # L1 死区最坏样本


def test_window_gate_hold_then_force_allow_and_reset():
    """同一件连续越窗:wait_max 次内 hold,第 wait_max+1 次 force_allow
    (绝不卡轮);任何一次窗内读数把计数清零(自愈)。"""
    g = DispatchWindowGate(LO, HI, wait_max=3)
    for i in (1, 2, 3):
        v, note = g.decide('part_3', 1.05)            # 死区样本越窗
        assert v == 'hold', (i, note)
        assert f'第 {i}/3' in note
    v, note = g.decide('part_3', 1.05)
    assert v == 'force_allow' and '第 4/3' in note
    # 兜底放行后计数清零:再次越窗重新从 hold 起
    assert g.decide('part_3', 1.05)[0] == 'hold'
    # 中途自愈:窗内读数清零
    for _ in range(2):
        assert g.decide('part_3', 1.05)[0] == 'hold'
    assert g.decide('part_3', 0.68)[0] == 'allow'
    assert g.decide('part_3', 1.05)[0] == 'hold'      # 计数已清零重计


def test_window_gate_per_part_counters_and_no_obs_note():
    """计数按件独立;no-obs 放行但 note 标注(取证可读)。"""
    g = DispatchWindowGate(LO, HI, wait_max=1)
    assert g.decide('part_1', 0.525)[0] == 'allow'
    assert g.decide('part_2', 1.05)[0] == 'hold'      # part_2 计数 1
    assert g.decide('part_1', 1.05)[0] == 'hold'      # part_1 独立计数 1
    assert g.decide('part_2', 1.05)[0] == 'force_allow'
    v, note = g.decide('part_3', None)
    assert v == 'allow' and note == 'no-obs'
    # 终态销账:forget 后同件重新计
    g.forget('part_1')
    assert g.decide('part_1', 1.05)[0] == 'hold'


def test_window_gate_legacy_parking_and_deadzone_samples():
    """实测样本口径(2026-10-06 六轮取证):串推停放带读数(0.670-0.946,
    v1 三轮的附着校正值)全部恒放行——这是 12/12 PLACED 的正当路径,
    不再靠兜底;L1 真死区样本(1.00-1.12,NO_IK 域)越窗挂起复检。"""
    g = DispatchWindowGate(LO, HI, wait_max=10**6)   # 只验窗,不验上限
    for x in (0.670, 0.824, 0.853, 0.890, 0.896, 0.940,
              0.946):                                 # v1 停放带实录
        assert g.decide('p', x)[0] == 'allow', x
    for x in (1.024, 1.10, 1.12):                    # L1 漂移死区样本
        assert g.decide('p', x)[0] == 'hold', x


def test_belt_controller_untouched_by_window_gate():
    """零回归红线:prime 链不经 start(),BeltController 恒 not-running
    -> 旧 settle 门控对 legacy 恒放行(两闸独立,pipeline 语义不变)。"""
    b = BeltController(stop_x=0.58, settle_delay=2.6, advance_speed=0.1)
    assert b.may_dispatch(now=999.0) is True          # 未 start 恒放行
    assert b.observe([0.9], 1.0) is None              # 未 start 不判停



def test_legacy_round_bypasses_pipeline_observe_line():
    """取证回归:legacy 出生帧喂 pipeline 判停线 0.58 必首帧触发回位
    (链串不动)——节点侧 _legacy_prime 轮因此旁路 _belt.observe,改由
    prime 定时链全权驱动。本例钉住"0.58 线对出生帧首帧即触发"这一
    旁路依据(BeltController 本身行为不变,pipeline 轮照常喂线)。"""
    b = BeltController(stop_x=0.58, settle_delay=2.6, advance_speed=0.1)
    b.start()
    assert b.observe([0.50, 0.58, 0.66], 0.1) == -0.25   # 首帧即触发=不能复用
