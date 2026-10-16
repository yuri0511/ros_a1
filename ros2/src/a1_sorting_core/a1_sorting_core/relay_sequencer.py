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
- 串行保证(默认):同一时刻整条流水线只有一个任务在跑(_current
  非空期间 try_dispatch 恒返回 None);
- per_arm_slots=true(并联):每条臂各一个执行槽,可以同时各有一件。
  取件区仍是全局一把锁,try_dispatch 占用,release_zone 才放开。
  终态只释放本臂的槽,不释放取件区锁。CARRYING 不是释放事件;
- 状态过滤:只认 part_id == 当前件且阶段匹配的终态,arm2 迟到/重复消息
  一律忽略(arm 空串视为 arm1,兼容旧消息)。

节点(task_dispatcher_node)只做 ROS 桥接:把 /task/status 终态喂给
on_terminal,按返回的延迟动作起 timer,到期后调用 try_dispatch /
on_relay_ready 并发布返回的 part_id 对应命令。
"""
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
                 dispatch_delay: float = 1.0, per_arm_slots: bool = False, require_arm_ready: bool = False):
        self._require_arm_ready = require_arm_ready
        self._outcomes = {}
        self._faulted_arms = set()
        self._relay_mode = bool(relay_mode)
        self._relay_delay = float(relay_delay)
        self._dispatch_delay = float(dispatch_delay)
        self._per_arm_slots = bool(per_arm_slots)
        self._queue: deque[str] = deque()
        self._queued: set[str] = set()
        self._done: set[str] = set()
        self._current: str | None = None
        self._phase: str | None = None
        # 并联:每臂最多一件。取件区全局最多一台。
        self._slots: dict[str, str | None] = {'arm1': None, 'arm2': None}
        self._zone_arm: str | None = None
        self._zone_part: str | None = None

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

    @property
    def zone_arm(self) -> str | None:
        """正在占用取件区的臂;空表示锁开着。"""
        return self._zone_arm

    def slot(self, arm: str) -> str | None:
        return self._slots.get(arm or 'arm1')

    def idle_arms(self) -> list:
        """没有执行槽的臂。单槽模式忙时为空,闲时只有 arm1。"""
        if not self._per_arm_slots:
            return [] if self._current is not None else ['arm1']
        return [name for name in ('arm1', 'arm2')
                if self._slots.get(name) is None and name not in self._faulted_arms]

    def busy_parts(self) -> set:
        """在途件集合(排队中 + 执行中;2026-10-06 语音防撞车闸)。

        注意 enqueue 只对 _queued/_done 去重,对**正在执行**的件
        (_current,已出队)不去重——外部 /sort_task 若对执行中的件再
        入队,该件终态后会被第二次派发执行(2026-10-06 E2E 实证)。
        服务回调据本集合拒绝重复派发。
        """
        out = set(self._queued)
        if self._current is not None:
            out.add(self._current)
        for part_id in self._slots.values():
            if part_id is not None:
                out.add(part_id)
        return out

    # ---------------------------------------------------------------- 入队
    def enqueue(self, part_id: str) -> bool:
        """新件入队(排队中/已完成的件忽略);入队后可调用 try_dispatch。"""
        if part_id in self._queued or part_id in self._done:
            return False
        if part_id in self._slots.values():
            return False
        self._queue.append(part_id)
        self._queued.add(part_id)
        return True

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

        per_arm_slots 时只看这一条臂的槽,并同时占用取件区。
        取件区已被占用,或这条臂已有件,则不出队。
        """
        arm = 'arm2' if arm == 'arm2' else 'arm1'
        if arm in self._faulted_arms:return None
        if self._per_arm_slots:
            if self._slots.get(arm) is not None or not self._queue:
                return None
            if self._zone_arm is not None:
                return None
            part_id = self._queue.popleft()
            self._queued.discard(part_id)
            self._slots[arm] = part_id
            self._zone_arm = arm
            self._zone_part = part_id
            self._phase = arm
            return part_id
        if self._current is not None or not self._queue:
            return None
        part_id = self._queue.popleft()
        self._queued.discard(part_id)
        self._current = part_id
        self._phase = PHASE_ARM2 if arm == 'arm2' else PHASE_ARM1
        return part_id

    def release_zone(self, part_id: str, arm: str) -> bool:
        """安全退出取件区。只放锁,不放这条臂的执行槽。

        必须是当前占锁的那一件、那条臂。重复调用返回 False。
        终态和 CARRYING 都不走这里。
        """
        if not self._per_arm_slots:
            return False
        arm = 'arm2' if arm == 'arm2' else 'arm1'
        if self._zone_arm != arm or self._zone_part != part_id:
            return False
        self._zone_arm = None
        self._zone_part = None
        return True

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
        if self._require_arm_ready:
            return False, []  # Legacy terminal messages never free runtime slots.
        return self._accept_terminal(part_id, arm, state)

    def _accept_terminal(self, part_id, arm, state):
        if state not in TERMINAL_STATES:
            return False, []
        if self._per_arm_slots:
            return self._on_terminal_per_arm(part_id, arm, state)
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

    def _matches_slot(self, part_id, arm):
        if arm not in ('arm1','arm2'):return False
        return (self._slots.get(arm)==part_id if self._per_arm_slots else
                self._current==part_id and self._phase==arm)

    def note_outcome(self, part_id, arm, state):
        if state not in ('PART_PLACED','FAILED') or not self._matches_slot(part_id,arm):return False
        key=(part_id,arm)
        outcome='COMPLETED' if state=='PART_PLACED' else 'FAILED'
        if key in self._outcomes:return self._outcomes[key]==outcome
        self._outcomes[key]=outcome
        return True

    def arm_fault(self, part_id, arm):
        if not self._matches_slot(part_id,arm):return False
        self._faulted_arms.add(arm)
        return True  # Keep slot, queue/dedup and zone ownership unchanged.

    def arm_ready(self, part_id, arm):
        key=(part_id,arm)
        if arm in self._faulted_arms or key not in self._outcomes or not self._matches_slot(part_id,arm):return False, [], None
        state=self._outcomes.pop(key)
        accepted,actions=self._accept_terminal(part_id,arm,state)
        return accepted,actions,state

    def _on_terminal_per_arm(self, part_id: str, arm: str, state: str):
        """只结束这一条臂的槽。取件区锁不动。"""
        del state
        arm = 'arm2' if arm == 'arm2' else 'arm1'
        if self._slots.get(arm) != part_id:
            return False, []
        self._done.add(part_id)
        self._slots[arm] = None
        if self._phase == arm:
            self._phase = None
        if self._queue:
            return True, [(DISPATCH, self._dispatch_delay)]
        return True, []

    # ---------------------------------------------------------------- 内部
    def _finish(self):
        self._done.add(self._current)
        self._current = None
        self._phase = None

    def _after_finish(self):
        if self._queue:
            return [(DISPATCH, self._dispatch_delay)]
        return []
