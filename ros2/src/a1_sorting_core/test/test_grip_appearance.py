"""观感修复①② 单测(2026-10-08):力保持 + 脱件小开闸。

背景(隔离实验定罪,cell_bringup 单会话无任务):
 ① 夹爪纯 effort 控制,无指令时两指**不受力**,被重力/接触推向相反
    限位且两臂镜像(arm1 L=-0.003/R=+0.040,arm2 反之)——"一闭一
    张"出生态;且**单发指令会丢**(新 publisher 首发布撞 DDS 订阅
    发现竞态,单发 +8N 三秒后指位不动,持续发布则两指对称响应)。
    修法 = 10Hz 重发最近一次力值(同值幂等,零物理差)+ 初值全开
    兼作启动归位。
 ② 旧释放序列在**箱底**一路开到 +0.04 全张限位:外格格心 y 偏
    ±0.05(锚表定值,红线),全开指外缘到箱心 0.128 > 壁内缘 0.085,
    插进壁带 43mm,随后张着满爪抬臂穿 95mm 壁高 = "顶住盒沿"。
    修法 = 位置闸小开到刚脱件 → 保持小开出箱 → 箱沿之上才全开。

本文件钉:判据纯函数数学 + 保持器三态(常值/位置保持/新命令接管)
+ 释放闸软闸契约(键查不到/中途消失/超时均不抛,按旧行为兜底)。
桩 self 直调未绑定方法(同 test_p2_settle 手法,不实例化节点)。
"""

import time
import types

import pytest

pytest.importorskip('rclpy', reason='executor 模块顶层依赖 rclpy')

from a1_sorting_core.pick_place_executor_node import (  # noqa: E402
    GraspError, PickPlaceExecutorNode as N, fingers_clamped,
    fingers_released)


# ------------------------------------ ② 判据:脱件(间隙口径 09 重写)
def test_released_both_past_gap():
    """间隙 = 0.052+ql+qr ≥ 52mm(>工件 50mm)即脱件。"""
    assert fingers_released(0.001, 0.001) is True      # 54mm
    assert fingers_released(0.040, 0.002) is True      # 94mm


def test_not_released_gap_still_tight():
    """双侧都还没让出间隙(闭爪持件位)不算脱件。"""
    assert fingers_released(-0.003, -0.003) is False   # 46mm 咬件
    assert fingers_released(-0.001, -0.001) is False   # 50mm 贴件
    assert fingers_released(-0.003, 0.001) is False    # 50mm 差 2mm


def test_released_single_side_free_counts():
    """共享命令下自由侧走到 +0.005、受阻侧钉在 -0.003:间隙 54mm,
    判脱件(旧口径永不满足→自由侧狂奔全张插壁 43mm 的病根)。"""
    assert fingers_released(0.005, -0.003) is True
    assert fingers_released(-0.003, 0.040) is True     # 已狂奔也算脱件


def test_released_gap_overridable():
    assert fingers_released(0.005, 0.005, release_gap=0.062) is False
    assert fingers_released(0.005, 0.005, release_gap=0.062 - 1e-9) is True


# ------------------------------------------------- ① 保持器三态
class _L:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


def _fake_self(positions=None, hold=8.0, target=None, pre_gap=0.0):
    s = types.SimpleNamespace()
    s._arm_name = 'arm2'
    s._joints_now = dict(positions or {})
    s._joints_alias = {}
    s._gripper_joints = 2
    s._gripper_open = 8.0
    s._gripper_closed = -8.0
    s._gripper_wait = 1.0
    s._pre_close_gap = pre_gap
    s._pre_close_force = -1.5
    s._pre_close_wait = 0.3
    s._descend_gap = 0.0
    s._grip_hold = hold
    s._grip_hold_target = target
    s._grip_release_force = 1.5
    s._pub_gripper = _Pub()
    s._pub_gripper_status = _Pub()
    s._logger = _L()
    s.get_logger = lambda: s._logger
    s._finger_joint_keys = lambda: N._finger_joint_keys(s)
    s._joint_lookup = lambda k: N._joint_lookup(s, k)
    s._pub_gripper_cmd = lambda v: N._pub_gripper_cmd(s, v)
    s._grip_done = lambda state, value: N._grip_done(s, state, value)
    s._grip = lambda **kw: N._grip(s, **kw)
    s._close_to_gap = lambda tag, gap=None: N._close_to_gap(s, tag, gap)
    s._wait_fingers_clamped = lambda lb, **kw: N._wait_fingers_clamped(
        s, lb, **kw)
    s._confirm_clamped = lambda tag: N._confirm_clamped(s, tag)
    return s


def test_hold_tick_republishes_last_value():
    """常值态:定时器原样重发 _grip_hold(同值幂等,零物理差)。"""
    s = _fake_self(hold=-1.5)
    N._grip_hold_tick(s)
    assert s._pub_gripper.sent == [[-1.5, -1.5]]


def test_hold_tick_position_hold_three_zones():
    """位置保持态:低于目标外推 / 带内 0N / 高于带 -force 内收。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.0005,
                    'arm2_gripper_finger_right_joint': 0.0005},
                   target=0.001)
    N._grip_hold_tick(s)
    assert s._pub_gripper.sent[-1] == [1.5, 1.5]      # 低于目标 → 外推

    s = _fake_self({'arm2_gripper_finger_left_joint': 0.0015,
                    'arm2_gripper_finger_right_joint': 0.0015},
                   target=0.001)
    N._grip_hold_tick(s)
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]      # 带内 → 卸荷

    s = _fake_self({'arm2_gripper_finger_left_joint': 0.020,
                    'arm2_gripper_finger_right_joint': 0.020},
                   target=0.001)
    N._grip_hold_tick(s)
    assert s._pub_gripper.sent[-1] == [-1.5, -1.5]    # 超带 → 内收


def test_hold_tick_missing_keys_falls_back_to_constant():
    """位置保持态但键查不到:退回常值重发,绝不抛。"""
    s = _fake_self({}, hold=8.0, target=0.001)
    N._grip_hold_tick(s)
    assert s._pub_gripper.sent[-1] == [8.0, 8.0]


def test_pub_gripper_cmd_records_hold_value():
    s = _fake_self()
    s._pub_gripper_cmd(-8.0)
    assert s._grip_hold == -8.0
    assert s._pub_gripper.sent[-1] == [-8.0, -8.0]


def test_new_grip_command_clears_position_hold():
    """任何新 _grip 命令接管夹爪:解除位置保持(否则定时器与命令打架)。"""
    s = _fake_self(target=0.001)
    s._grip(close=False, wait=0.0)
    assert s._grip_hold_target is None
    assert s._grip_hold == 8.0
    s._grip(close=True, force=-1.5, wait=0.0)
    assert s._grip_hold_target is None
    assert s._grip_hold == -1.5


# ------------------------------------------------- ② 释放闸软闸契约
def test_open_to_release_gap_missing_keys_soft_fallback():
    s = _fake_self()                      # 主表/别名表皆空 → 键查不到
    t0 = time.monotonic()
    ok = N._open_to_release_gap(s, timeout=0.3)
    dt = time.monotonic() - t0
    assert ok is False
    assert dt >= 1.1                      # 退回旧"1.5N 慢开 1.2s"
    assert any('键查不到' in m for _, m in s._logger.lines)
    assert s._grip_hold_target is None


def test_open_to_release_gap_reaches_and_unloads():
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.0015,
                    'arm2_gripper_finger_right_joint': 0.0015})
    t0 = time.monotonic()
    ok = N._open_to_release_gap(s, timeout=2.0)
    assert ok is True
    assert time.monotonic() - t0 < 0.5    # 已到位不傻等
    assert s._grip_hold_target is None    # 不交棒 bang-bang(共享命令
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]   # 到位即 0N 中立
    assert any('释放小开到位' in m for _, m in s._logger.lines)


def test_open_to_release_gap_pinned_side_passes_immediately():
    """旧版 6/8 超时的主角(一侧钉在 -0.003、自由侧已到 0.040):
    间隙口径下 89mm ≥ 52mm **即刻脱件**,卸荷出箱,不再等 2s 超时。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': -0.003,
                    'arm2_gripper_finger_right_joint': 0.040})
    t0 = time.monotonic()
    ok = N._open_to_release_gap(s, timeout=2.0)
    assert ok is True
    assert time.monotonic() - t0 < 0.5
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]   # 卸荷中立


def test_open_to_release_gap_both_pinned_holds_current_gap():
    """双侧都钉死(间隙 <52mm):超时**不抛**,卸荷保持当前开度出箱。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': -0.003,
                    'arm2_gripper_finger_right_joint': -0.003})
    t0 = time.monotonic()
    ok = N._open_to_release_gap(s, timeout=0.4)
    dt = time.monotonic() - t0
    assert ok is False
    assert 0.4 <= dt < 1.0                # 只等超时,不再补 1.2s 全开
    assert any('超时' in m and '卸荷保持' in m
               for _, m in s._logger.lines)
    assert s._grip_hold_target is None
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]   # 末条 = 卸荷,非 8N 全开


# -------------------------------- 夹持确认闸(2026-10-08 晚,§十六)
def test_clamped_true_on_part():
    """真咬住 50mm 工件:两指停在间隙≈50mm(-0.001/-0.001)或更紧。"""
    assert fingers_clamped(-0.001, -0.001) is True
    assert fingers_clamped(-0.003, 0.001) is True     # 间隙 50mm 压线过


def test_clamped_false_pinned():
    """钉死态(一指卡全开 0.04、一指到底 -0.003):间隙 89mm 必拒。
    预收口态 62mm 也拒(还没咬件)。"""
    assert fingers_clamped(0.040, -0.003) is False
    assert fingers_clamped(-0.003, 0.040) is False
    assert fingers_clamped(0.005, 0.005) is False     # 间隙 62mm
    assert fingers_clamped(0.002, 0.002) is False     # 间隙 56mm(>55)


def test_clamped_boundary_and_override():
    assert fingers_clamped(0.0, 0.003) is True        # 恰 55mm
    assert fingers_clamped(0.0, 0.004) is False       # 56mm
    assert fingers_clamped(0.005, 0.005, max_gap_m=0.062) is True


def test_close_to_gap_disabled_by_default():
    """pre_close_gap=0:直接返回,不碰夹爪(legacy 零行为差)。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.040,
                    'arm2_gripper_finger_right_joint': 0.040})
    N._close_to_gap(s, '预收口')
    assert s._pub_gripper.sent == []
    assert s._logger.lines == []


def test_close_to_gap_reaches_target():
    """两指已在目标间隙内:一条卸荷命令 + 到位读数,不再施力。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.005,
                    'arm2_gripper_finger_right_joint': 0.005},
                   pre_gap=0.005)
    N._close_to_gap(s, '预收口')
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]
    assert any('到位' in m for _, m in s._logger.lines)


def test_close_to_gap_pushes_then_unloads():
    """两指全开:循环发轻闭力,到位后卸荷收束。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.040,
                    'arm2_gripper_finger_right_joint': 0.040},
                   pre_gap=0.005)
    orig = s._joint_lookup

    def moving(k):
        v = orig(k)
        if v is None:
            return None
        s._joints_now[k] = max(-0.003, v - 0.008)   # 每 poll 走 8mm
        return v
    s._joint_lookup = moving
    s._pre_close_wait = 0.8
    N._close_to_gap(s, '预收口')
    assert any(m == [-1.5, -1.5] for m in s._pub_gripper.sent)
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]
    assert any('到位' in m for _, m in s._logger.lines)


def test_close_to_gap_timeout_soft():
    """被钉住到不了位:超时软放行(不抛),末条卸荷。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.040,
                    'arm2_gripper_finger_right_joint': 0.040},
                   pre_gap=0.005)
    s._pre_close_wait = 0.15
    t0 = time.monotonic()
    N._close_to_gap(s, '预收口')
    assert 0.15 <= time.monotonic() - t0 < 1.0
    assert any('超时' in m for _, m in s._logger.lines)
    assert s._pub_gripper.sent[-1] == [0.0, 0.0]


def test_wait_clamped_true_and_none():
    s = _fake_self({'arm2_gripper_finger_left_joint': -0.002,
                    'arm2_gripper_finger_right_joint': -0.002})
    assert N._wait_fingers_clamped(s, '闸') is True
    assert any('夹持确认' in m for _, m in s._logger.lines)
    s2 = _fake_self()                                  # 键查不到
    assert N._wait_fingers_clamped(s2, '闸') is None
    assert any('键查不到' in m for _, m in s2._logger.lines)


def test_confirm_clamped_retries_then_raises():
    """钉死态:补锁紧(全开解楔→重收口→重锁)两次仍不合拢 → GraspError
    (拒绝抬臂走重试链)。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.040,
                    'arm2_gripper_finger_right_joint': -0.003})
    s._wait_fingers_clamped = lambda lb, **kw: N._wait_fingers_clamped(
        s, lb, timeout=0.1)
    t0 = time.monotonic()
    with pytest.raises(GraspError):
        s._confirm_clamped('锁紧段')
    assert time.monotonic() - t0 >= 2 * 0.6          # 两次补锁紧各等 0.6s
    assert any('第 2 次重夹' in m for _, m in s._logger.lines)


def test_confirm_clamped_recovers_on_retry():
    """补锁紧后合拢:第二次读数达标即放行,不抛。"""
    s = _fake_self({'arm2_gripper_finger_left_joint': 0.040,
                    'arm2_gripper_finger_right_joint': -0.003})
    calls = {'n': 0}

    def fake(k):
        v = N._joint_lookup(s, k)
        if v is None:
            return None
        if k.endswith('left_joint') and calls['n'] >= 2:
            return -0.002                              # 补锁紧后脱钉
        return v
    orig_tick = N._wait_fingers_clamped

    def counted(self_, label, timeout=2.5, poll=0.05, max_gap_m=0.055):
        calls['n'] += 1
        return orig_tick(self_, label, timeout=0.1, poll=0.05,
                         max_gap_m=max_gap_m)
    s._wait_fingers_clamped = lambda lb, **kw: counted(s, lb, **kw)
    s._joint_lookup = fake
    s._pub_gripper_cmd = lambda v: N._pub_gripper_cmd(s, v)
    N._confirm_clamped(s, '锁紧段')                     # 不抛即通过


# -------------------------------- 实测对中(§十六终案)
class _Cache:
    def __init__(self, xy):
        self._xy = xy

    def get(self, pid):
        return self._xy


class _Part:
    part_id = 'part_1'


def _recenter_self(tol=0.008, true_xy=None, base_xy=None, plans=None):
    plans = [] if plans is None else plans
    s = _fake_self()
    s._center_tol = tol
    s._center_max = 0.05
    s._cam_cache = _Cache(true_xy)
    s._gripper_center_xy = lambda: base_xy
    s._tcp_z = 0.204
    s._pre_h = 0.08
    s._descend_gap = 0.015
    s._pick_descend_linear = True
    s._down_pose = lambda c, z: (c, z)
    s._plan_execute = lambda p, label, linear=False: plans.append(label)
    s._close_to_gap = lambda tag, gap=None: plans.append(tag)
    s._descend = lambda c, z, linear=True: plans.append('descend2')
    s._wait_tool_near = lambda xy, z, label, **kw: plans.append(label)
    s._plans = plans
    return s


def test_recenter_disabled_returns_original():
    s = _recenter_self(tol=0.0)
    c = (0.40, 0.0, 0.805)
    assert N._recenter_over_part(s, _Part(), c) == c
    assert s._logger.lines == []


def test_recenter_within_tol_no_move():
    s = _recenter_self(true_xy=(0.401, 0.002), base_xy=(0.400, 0.000))
    plans = []
    c = (0.40, 0.0, 0.805)
    assert N._recenter_over_part(s, _Part(), c) == c
    assert plans == []
    assert any('无需修正' in m for _, m in s._logger.lines)


def test_recenter_corrects_and_re_returns_new_center():
    """偏心 26mm:平移重下探,返回值 = center+off 且保持三元组。"""
    s = _recenter_self(true_xy=(0.426, 0.004), base_xy=(0.400, 0.000))
    c = (0.40, 0.0, 0.805)
    out = N._recenter_over_part(s, _Part(), c)
    assert out == (0.426, 0.004, 0.805)
    assert s._plans == ['recenter', '重下探收窄', 'descend2', 'pick_settle2']
    assert any('实测偏心 26mm' in m for _, m in s._logger.lines)


def test_recenter_far_off_fails():
    """偏心 > 上限(带面漂移超窗):判可重试失败,不硬追。"""
    s = _recenter_self(true_xy=(0.460, 0.0), base_xy=(0.400, 0.0))
    with pytest.raises(GraspError):
        N._recenter_over_part(s, _Part(), (0.40, 0.0, 0.805))


def test_recenter_missing_data_soft_skip():
    for ty, bx in ((None, (0.4, 0.0)), ((0.4, 0.0), None)):
        s = _recenter_self(true_xy=ty, base_xy=bx)
        c = (0.40, 0.0, 0.805)
        assert N._recenter_over_part(s, _Part(), c) == c
        assert any('缺失' in m for _, m in s._logger.lines)
