"""P2-2a 单测:指关节停稳闸(判据纯函数 + 软闸行为)。

时序修正本体(抬臂前插 _wait_fingers_settled)在仿真闸取证(dual 轮
日志"两指停稳"读数);本文件钉两层:
(a) fingers_settled 判据数学:0.1s 窗位移双侧均 |Δ|<2mm ⇒ 停稳;
(b) _wait_fingers_settled 软闸契约(零新增失败路径):键查不到/轮询
    中键消失/超时均**不抛异常**只告警返回 False;静止时连续两次安静
    采样返回 True。
桩 self 直调未绑定方法(同 test_grip_sync_cap 手法,不实例化节点)。
"""

import time
import types

import pytest

pytest.importorskip('rclpy', reason='executor 模块顶层依赖 rclpy')

from a1_sorting_core.pick_place_executor_node import (  # noqa: E402
    PickPlaceExecutorNode as N, fingers_settled)


# ---------------------------------------------------------------- (a) 判据
def test_settled_both_quiet():
    assert fingers_settled(0.000, 0.000) is True
    assert fingers_settled(0.0019, -0.0019) is True   # 1.9mm 仍算停


def test_not_settled_one_finger_moving():
    """一侧还在走(≥2mm)不停稳——旧固定 wait 漏掉的正是这种相位。"""
    assert fingers_settled(0.002, 0.0) is False       # 恰 2mm:< 为假
    assert fingers_settled(0.015, 0.001) is False     # sync_cap 同相位
    assert fingers_settled(-0.004, 0.0) is False      # 张开方向同计 |Δ|


def test_settled_threshold_overridable():
    assert fingers_settled(0.005, 0.005, tol_m=0.006) is True
    assert fingers_settled(0.007, 0.0, tol_m=0.006) is False


# ---------------------------------------------------------------- (b) 软闸
class _L:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))


def _fake_self(positions=None):
    """positions=None → 键查不到;dict 则主表存之,lookup 直读。"""
    s = types.SimpleNamespace()
    s._arm_name = 'arm2'
    s._joints_now = dict(positions or {})
    s._joints_alias = {}
    s._logger = _L()
    s.get_logger = lambda: s._logger
    s._finger_joint_keys = lambda: N._finger_joint_keys(s)
    s._joint_lookup = lambda k: N._joint_lookup(s, k)
    return s


def test_soft_gate_missing_keys_returns_false_no_raise():
    s = _fake_self()
    assert N._wait_fingers_settled(s, '锁紧段', timeout=0.3) is False
    assert any('键查不到' in m for _, m in s._logger.lines)


def test_soft_gate_still_fingers_passes_quickly():
    pos = {'arm2_gripper_finger_left_joint': 0.001,
           'arm2_gripper_finger_right_joint': 0.002}
    s = _fake_self(pos)
    t0 = time.monotonic()
    ok = N._wait_fingers_settled(s, '锁紧段', timeout=2.0)
    dt = time.monotonic() - t0
    assert ok is True                       # 连续 2 个安静采样即放行
    assert dt < 1.0                         # 不傻等满 timeout
    assert any('两指停稳' in m for _, m in s._logger.lines)


def test_soft_gate_moving_fingers_times_out_softly():
    """指一直在走:超时**不抛**,告警放行(夹持真伪仍由探针/验证把关)。"""
    pos = {'arm2_gripper_finger_left_joint': 0.03,
           'arm2_gripper_finger_right_joint': 0.03}
    s = _fake_self(pos)

    class MovingClock:
        pass

    t0 = time.monotonic()
    # 每轮 lookup 都读到移动值:用可变桩模拟关节推进
    def bump():
        while True:
            s._joints_now['arm2_gripper_finger_left_joint'] -= 0.004
            time.sleep(0.05)
    import threading
    th = threading.Thread(target=bump, daemon=True)
    th.start()
    try:
        ok = N._wait_fingers_settled(s, '锁紧段', timeout=0.5, poll=0.1)
    finally:
        th = None  # daemon 线程随测试结束
    dt = time.monotonic() - t0
    assert ok is False and dt >= 0.45
    assert any('超时' in m and '放行' in m for _, m in s._logger.lines)
