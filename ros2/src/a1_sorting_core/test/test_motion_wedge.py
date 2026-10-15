"""R1-P1 楔死判死闸单元测试(motion_client.MotionClient 记账状态机)。

纯 Python:mock node/fake future,monkeypatch 掉 ActionClient 与 sleep,
不连接任何 ROS 运行时。判据语义(与实装一致,Lead 2026-10-04):
session_dead = 曾出现 dangling(结果未回传被早退放行)之后,同一客户端
连续 ≥3 次 CONTROL_FAILED(-4);任意一次完整正常成功清 dangling/连击。
forensics §2.7 的"纯 -4 连击 3 次判死"(其 test 3)**不采用**——双臂
瞬态互挤(2026-10-01 实证常态)也产生 -4,无 dangling 前置会假判死,
这正是该报告 G-C 警告的病灶。
"""
import time

import pytest

from a1_sorting_core import motion_client as mc


class FakeLogger:
    def __init__(self):
        self.errors, self.warns, self.infos = [], [], []

    def error(self, m):
        self.errors.append(str(m))

    def warn(self, m):
        self.warns.append(str(m))

    def info(self, m):
        self.infos.append(str(m))


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeClient:
    """替掉 ActionClient:wait_for_server 恒真,send_goal_async 计调用数。"""

    calls = 0

    def __init__(self, node, msg, name):
        pass

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        FakeClient.calls += 1
        raise AssertionError('判死后不应再发送目标')


def _mk_client(wedge_detect=True, monkeypatch=None):
    monkeypatch.setattr(mc, 'ActionClient', FakeClient)
    c = mc.MotionClient(FakeNode(), 'g', 'tool0', 'world',
                        wedge_detect=wedge_detect)
    return c


def test_wedge_flags_default_off_no_behavior_change(monkeypatch):
    """回归锚:开关关 ⇒ 记账与判死全部沉默,session_dead 恒 False。"""
    c = _mk_client(wedge_detect=False, monkeypatch=monkeypatch)
    c._note_outcome(True, dangling=True)
    for _ in range(5):
        c._note_outcome(False, code=-4)
    assert c.session_dead is False
    assert c._refuse_if_dead('x') is False
    assert not c._node.get_logger().errors


def test_dangling_then_three_m4_kills_session(monkeypatch):
    """到位闸早退(dangling)→ 连续 3 次 -4 ⇒ 判死 + 错误日志锚。"""
    c = _mk_client(monkeypatch=monkeypatch)
    c._note_outcome(True, dangling=True)
    assert c.session_dead is False
    c._note_outcome(False, code=-4)
    c._note_outcome(False, code=-4)
    assert c.session_dead is False          # 2 连不杀(阈值 N=3)
    c._note_outcome(False, code=-4)
    assert c.session_dead is True
    assert any('MOTION_SESSION_WEDGED' in e for e in c._node.get_logger().errors)


def test_m4_without_dangling_does_not_kill(monkeypatch):
    """双臂瞬态互挤的 -4(无悬置前置)不判死——G-C 假判死防线。"""
    c = _mk_client(monkeypatch=monkeypatch)
    for _ in range(6):
        c._note_outcome(False, code=-4)
    assert c.session_dead is False


def test_normal_result_resets_dangling_and_streak(monkeypatch):
    """完整正常成功清 dangling 与连击:判死需"悬置之后"的 3 连。"""
    c = _mk_client(monkeypatch=monkeypatch)
    c._note_outcome(True, dangling=True)
    c._note_outcome(False, code=-4)
    c._note_outcome(False, code=-4)         # 2 连,未达阈值
    c._note_outcome(True)                   # 一次完整成功:全清
    assert c._dangling is False and c._consec_m4 == 0
    for _ in range(4):                      # 之后 4 连 -4:无 dangling 前置
        c._note_outcome(False, code=-4)
    assert c.session_dead is False
def test_success_between_m4_resets_streak(monkeypatch):
    """-4, -4, 成功, -4, -4, -4 只在最后 3 连后才可能判死(且需 dangling)。"""
    c = _mk_client(monkeypatch=monkeypatch)
    c._note_outcome(True, dangling=True)
    c._note_outcome(False, code=-4)
    c._note_outcome(False, code=-4)
    c._note_outcome(True)                   # 成功:清 dangling+连击
    for _ in range(4):
        c._note_outcome(False, code=-4)
    assert c.session_dead is False          # 无 dangling 前置,不判死


def test_session_dead_blocks_new_goals(monkeypatch):
    """判死后 move_to_joints/move_to_pose 拒发:send_goal_async 零调用。"""
    c = _mk_client(monkeypatch=monkeypatch)
    c._note_outcome(True, dangling=True)
    for _ in range(3):
        c._note_outcome(False, code=-4)
    assert c.session_dead is True
    FakeClient.calls = 0
    assert c.move_to_joints(['a'], [0.0], 'x') is False
    assert c.move_to_pose(object(), 'y') is False
    assert FakeClient.calls == 0            # 完全未触碰发送
