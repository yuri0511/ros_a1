"""任务状态机(纯逻辑,与 ROS 解耦,便于单元测试)。

状态迁移:

    IDLE -> PICKING -> CARRYING -> PLACING -> COMPLETED
     ^         |          |           |
     |         v          v           v
     +------- FAILED <----------------+
"""
from __future__ import annotations  # ROS1 移植: Noetic 为 Python 3.8,需延迟求值 "str | None" 等新式注解
from enum import Enum


class TaskState(str, Enum):
    """一次搬运任务的状态。"""

    IDLE = 'IDLE'
    PICKING = 'PICKING'
    CARRYING = 'CARRYING'
    PLACING = 'PLACING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'


# 合法迁移表:任务从 IDLE 进入工作态,任一工作态都可失败,终态后回到 IDLE
_TRANSITIONS = {
    TaskState.IDLE: {TaskState.PICKING, TaskState.FAILED},
    TaskState.PICKING: {TaskState.CARRYING, TaskState.FAILED},
    TaskState.CARRYING: {TaskState.PLACING, TaskState.FAILED},
    TaskState.PLACING: {TaskState.COMPLETED, TaskState.FAILED},
    TaskState.COMPLETED: {TaskState.IDLE},
    TaskState.FAILED: {TaskState.IDLE},
}


class IllegalTransition(Exception):
    """非法状态迁移。"""


class TaskStateMachine:
    """单件任务的状态机,由执行节点持有。"""

    def __init__(self):
        self._state = TaskState.IDLE

    @property
    def state(self) -> TaskState:
        return self._state

    def advance(self, next_state: TaskState) -> None:
        """迁移到下一状态;同态重复上报视为幂等(重试期间会出现)。"""
        if next_state == self._state:
            return
        if next_state not in _TRANSITIONS[self._state]:
            raise IllegalTransition(
                f'非法迁移: {self._state.value} -> {next_state.value}')
        self._state = next_state

    def reset(self) -> None:
        """回到 IDLE,准备下一件任务。"""
        self._state = TaskState.IDLE
