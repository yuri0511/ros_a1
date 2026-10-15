"""attach 跟随提质单测(2026-10-09,搬运失真修复)。

定罪(同步遥测):旧 _send_pose "上一条在途就丢整拍",大机动时实际
更新率塌到 ~15-20Hz,工件一跳一跳落后夹爪 22-43mm。
修法:最多等一个周期,然后照发本拍最新位姿(不丢拍、不堆积)。
"""
import types

import pytest

pytest.importorskip('rclpy', reason='attach_helper 顶层依赖 rclpy')

from a1_sorting_core.attach_helper import GzKinematicAttach  # noqa: E402


class _Future:
    def __init__(self, done=True):
        self._done = done

    def done(self):
        return self._done

    def add_done_callback(self, cb):
        pass


class _Client:
    def __init__(self, pend_done):
        self.sent = []
        self._pend_done = pend_done
        self.cur = None

    def call_async(self, req):
        self.sent.append(req)
        f = _Future(self._pend_done)
        self.cur = f
        return f


def _fake_attach(period=0.0125, pend_done=True):
    a = GzKinematicAttach.__new__(GzKinematicAttach)
    a._period = period
    a._pending = None
    a._client = _Client(pend_done)
    a._MODEL_done = None
    import numpy as np
    a._mat = np.eye(4)
    return a


def test_send_pose_no_drop_when_inflight():
    """上一条在途:仍然发出本拍请求(旧版直接 return 丢拍)。"""
    a = _fake_attach(pend_done=False)
    a._pending = _Future(done=False)
    # 手动让 future 立刻完成(模拟等待窗口内落地)
    import threading
    threading.Timer(0.005, lambda: setattr(a._pending, '_done', True)).start()
    a._send_pose('part_1', a._mat)
    assert len(a._client.sent) == 1


def test_send_pose_fires_when_idle():
    a = _fake_attach()
    a._send_pose('part_1', a._mat)
    assert len(a._client.sent) == 1
    assert a._pending is not None


def test_send_pose_sends_latest_after_wait():
    """等待窗口耗尽仍未完成:照发最新值(请求覆盖,不丢节拍)。"""
    a = _fake_attach(period=0.01)
    a._pending = _Future(done=False)   # 永远不完成
    a._send_pose('part_1', a._mat)
    assert len(a._client.sent) == 1    # 旧版此处为 0(丢拍)
