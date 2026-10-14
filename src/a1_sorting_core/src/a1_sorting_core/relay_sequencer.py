"""接力调度纯逻辑(与 ROS 解耦,便于单元测试)。

单件流水线生命周期(relay_mode=true):

    空闲 --try_dispatch--> arm1 分拣 --COMPLETED--> 等待接力(on_relay_ready)
                                    |                    |
                                 FAILED               arm2 码放
                                    |                    |
                                    +--> 完成 <---------+
                                    (跳过接力)      COMPLETED/FAILED

- relay_mode=false 时退化为旧单臂逻辑:arm1 终态即该件结束,延迟后派发下一件;
- 并联分色(2026-10-04 Phase1):调度器对路由色直接 try_dispatch('arm2'),
  phase=arm2,arm2 终态即该件结束;arm1 对该件的(迟到)终态按阶段
  不匹配忽略。接力 RELAY 路径在并联模式下不构造(relay_mode 由节点
  在并联时置 false);
- 串行保证:同一时刻整条流水线只有一个任务在跑(_current 非空期间
  try_dispatch 恒返回 None);
- 状态过滤:只认 part_id == 当前件且阶段匹配的终态,arm2 迟到/重复消息
  一律忽略(arm 空串视为 arm1,兼容旧消息)。

节点(task_dispatcher_node)只做 ROS 桥接:把 /task/status 终态喂给
on_terminal,按返回的延迟动作起 timer,到期后调用 try_dispatch /
on_relay_ready 并发布返回的 part_id 对应命令。
"""
from __future__ import annotations  # ROS1 移植: Noetic 为 Python 3.8,需延迟求值 "str | None" 等新式注解
from collections import deque

DISPATCH = 'dispatch'   # 延迟 delay 秒后调用 try_dispatch 派发下一件
RELAY = 'relay'         # 延迟 delay 秒后调用 on_relay_ready 派发接力

TERMINAL_STATES = ('COMPLETED', 'FAILED')

# 当前件所处阶段:None=空闲, 'arm1'=arm1 执行中,
# 'wait_relay'=arm1 已完成等待接力派发, 'arm2'=arm2 执行中
PHASE_ARM1 = 'arm1'
PHASE_WAIT_RELAY = 'wait_relay'
PHASE_ARM2 = 'arm2'


class RelaySequencer:
    """串行任务队列 + 双臂接力推进器(纯逻辑,线程外使用需自行加锁)。"""

    def __init__(self, relay_mode: bool = False, relay_delay: float = 1.0,
                 dispatch_delay: float = 1.0):
        self._relay_mode = bool(relay_mode)
        self._relay_delay = float(relay_delay)
        self._dispatch_delay = float(dispatch_delay)
        self._queue: deque[str] = deque()
        self._queued: set[str] = set()
        self._done: set[str] = set()
        self._current: str | None = None
        self._phase: str | None = None

    # ------------------------------------------------------------- 只读视图
    @property
    def current(self) -> str | None:
        return self._current

    @property
    def phase(self) -> str | None:
        return self._phase

    @property
    def queue_size(self) -> int:
        return len(self._queue)

    def is_done(self, part_id: str) -> bool:
        return part_id in self._done

    # ---------------------------------------------------------------- 入队
    def enqueue(self, part_id: str) -> bool:
        """新件入队。正在执行、排队中、已完成的件都忽略。

        try_dispatch 会把当前件从排队集合里拿走。若这里不看 current，
        start_batch 会把正在抓的那一件再排到队尾。
        """
        if self.block_reason(part_id) is not None:
            return False
        self._queue.append(part_id)
        self._queued.add(part_id)
        return True

    def block_reason(self, part_id: str) -> str | None:
        """不能入队时说明原因: current 正在执行, queued 已在队里, done 本轮已结束。"""
        if part_id == self._current:
            return 'current'
        if part_id in self._queued:
            return 'queued'
        if part_id in self._done:
            return 'done'
        return None

    # ------------------------------------------------------------- 派发推进
    def peek(self) -> str | None:
        """返回队首件 id(不出队);空队/执行中返回 None。

        供调度节点在派发前按来料类别决定路由目标臂(并联分色)。
        """
        return self._queue[0] if self._queue else None

    def try_dispatch(self, arm: str = 'arm1') -> str | None:
        """空闲且有队列:弹出下一件并返回 part_id,进入执行阶段。

        arm='arm1'(默认):经典/接力路径,phase=arm1;
        arm='arm2':并联分色路由——该件由 arm2 从头执行(取件+放置),
        phase=arm2,arm2 终态即该件完全终态(不经 RELAY)。

        否则返回 None(忙/队空);重复调用幂等,timer 重复触发无害。
        """
        if self._current is not None or not self._queue:
            return None
        part_id = self._queue.popleft()
        self._queued.discard(part_id)
        self._current = part_id
        self._phase = PHASE_ARM2 if arm == 'arm2' else PHASE_ARM1
        return part_id

    def on_relay_ready(self) -> str | None:
        """relay_delay 到期:确认仍在等待接力,进入 arm2 阶段。

        返回 part_id(节点发布接力命令);阶段不符(重复/迟到触发)返回 None。
        """
        if self._phase != PHASE_WAIT_RELAY:
            return None
        self._phase = PHASE_ARM2
        return self._current

    def on_terminal(self, part_id: str, arm: str, state: str):
        """输入一条终态回报;返回 (是否受理, 应执行的延迟动作列表)。

        只受理 part_id == 当前件 且 阶段匹配的终态;其余(迟到/重复/
        非本阶段/非终态)返回 (False, [])。受理时:
        - arm1 COMPLETED 且 relay_mode -> (True, [(RELAY, relay_delay)]);
        - arm1 FAILED(或非接力模式终态)-> 该件结束,队列非空时
          (True, [(DISPATCH, dispatch_delay)])否则 (True, []);
        - arm2 终态 -> 该件结束,同上安排下一件。
        """
        if state not in TERMINAL_STATES:
            return False, []
        if self._current is None or part_id != self._current:
            return False, []      # 迟到的旧状态,忽略
        arm = arm or 'arm1'       # 空串视为 arm1(兼容旧消息)
        if self._phase == PHASE_ARM1 and arm == 'arm1':
            if state == 'COMPLETED' and self._relay_mode:
                self._phase = PHASE_WAIT_RELAY
                return True, [(RELAY, self._relay_delay)]
            self._finish()
            return True, self._after_finish()
        if self._phase == PHASE_ARM2 and arm == 'arm2':
            self._finish()
            return True, self._after_finish()
        return False, []          # 阶段不匹配(重复/迟到/异臂消息),忽略

    # ---------------------------------------------------------------- 内部
    def _finish(self):
        self._done.add(self._current)
        self._current = None
        self._phase = None

    def _after_finish(self):
        if self._queue:
            return [(DISPATCH, self._dispatch_delay)]
        return []
