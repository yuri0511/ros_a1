"""任务状态机单元测试。"""
import pytest

from a1_sorting_core.state_machine import (IllegalTransition, TaskState,
                                           TaskStateMachine)


def test_normal_path():
    sm = TaskStateMachine()
    sm.advance(TaskState.PICKING)
    sm.advance(TaskState.CARRYING)
    sm.advance(TaskState.PLACING)
    sm.advance(TaskState.COMPLETED)
    sm.advance(TaskState.IDLE)
    assert sm.state == TaskState.IDLE


def test_failed_from_each_working_state():
    """到达每个工作态(沿合法路径)后都能失败并回到 IDLE。"""
    paths = {
        TaskState.PICKING: [TaskState.PICKING],
        TaskState.CARRYING: [TaskState.PICKING, TaskState.CARRYING],
        TaskState.PLACING: [TaskState.PICKING, TaskState.CARRYING,
                            TaskState.PLACING],
    }
    for broken in (TaskState.PICKING, TaskState.CARRYING, TaskState.PLACING):
        sm = TaskStateMachine()
        for step in paths[broken]:
            sm.advance(step)
        sm.advance(TaskState.FAILED)
        sm.advance(TaskState.IDLE)
        assert sm.state == TaskState.IDLE


def test_failed_from_idle_allowed():
    """来料未知类别等场景下,任务未开始即可判失败。"""
    sm = TaskStateMachine()
    sm.advance(TaskState.FAILED)
    assert sm.state == TaskState.FAILED


def test_illegal_skip_rejected():
    sm = TaskStateMachine()
    with pytest.raises(IllegalTransition):
        sm.advance(TaskState.PLACING)          # 不能跳过 PICKING/CARRYING
    with pytest.raises(IllegalTransition):
        sm.advance(TaskState.COMPLETED)
    assert sm.state == TaskState.IDLE


def test_illegal_restart_rejected():
    sm = TaskStateMachine()
    sm.advance(TaskState.PICKING)
    with pytest.raises(IllegalTransition):
        sm.advance(TaskState.IDLE)  # 工作态不能直接回 IDLE


def test_idempotent_same_state():
    """重试期间可能重复上报同一工作态,应视为幂等。"""
    sm = TaskStateMachine()
    sm.advance(TaskState.PICKING)
    sm.advance(TaskState.PICKING)
    sm.advance(TaskState.PICKING)
    assert sm.state == TaskState.PICKING


def test_reset():
    sm = TaskStateMachine()
    sm.advance(TaskState.FAILED)
    sm.reset()
    assert sm.state == TaskState.IDLE
    sm.advance(TaskState.PICKING)
    assert sm.state == TaskState.PICKING
