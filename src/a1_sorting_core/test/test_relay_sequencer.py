"""接力调度纯逻辑(RelaySequencer)单元测试:不初始化 rclpy。"""
from a1_sorting_core.relay_sequencer import DISPATCH, RELAY, RelaySequencer


def make(relay_mode: bool = False) -> RelaySequencer:
    return RelaySequencer(relay_mode=relay_mode, relay_delay=0.5,
                          dispatch_delay=0.2)


def test_relay_full_pipeline():
    """arm1 完成 -> 发接力命令 -> arm2 终态 -> 才派发下一件(全程串行)。"""
    seq = make(relay_mode=True)
    seq.enqueue('part_1')
    seq.enqueue('part_2')
    assert seq.try_dispatch() == 'part_1'
    assert seq.phase == 'arm1'

    # 整条流水线被当前件占用:任何阶段都不派发下一件
    assert seq.try_dispatch() is None

    # arm1 完成 -> 不结束该件,要求延迟后向 arm2 派发
    accepted, actions = seq.on_terminal('part_1', 'arm1', 'COMPLETED')
    assert accepted
    assert actions == [(RELAY, 0.5)]
    assert seq.current == 'part_1'
    assert seq.phase == 'wait_relay'
    assert seq.try_dispatch() is None            # 等待接力期间不派发

    # relay_delay 到期 -> 派发 arm2
    assert seq.on_relay_ready() == 'part_1'
    assert seq.phase == 'arm2'
    assert seq.try_dispatch() is None            # arm2 执行期间不派发

    # arm2 终态 -> 该件结束,安排下一件
    accepted, actions = seq.on_terminal('part_1', 'arm2', 'COMPLETED')
    assert accepted
    assert actions == [(DISPATCH, 0.2)]
    assert seq.is_done('part_1')
    assert seq.current is None
    assert seq.try_dispatch() == 'part_2'


def test_arm1_failed_skips_relay():
    """arm1 FAILED -> 跳过接力直接下一件。"""
    seq = make(relay_mode=True)
    seq.enqueue('part_1')
    seq.enqueue('part_2')
    assert seq.try_dispatch() == 'part_1'
    accepted, actions = seq.on_terminal('part_1', 'arm1', 'FAILED')
    assert accepted
    assert actions == [(DISPATCH, 0.2)]          # 无 RELAY 动作
    assert seq.is_done('part_1')
    assert seq.phase is None
    assert seq.try_dispatch() == 'part_2'


def test_late_and_duplicate_ignored():
    """迟到/重复/阶段不匹配的消息一律忽略,不推进状态。"""
    seq = make(relay_mode=True)
    seq.enqueue('part_1')
    seq.enqueue('part_2')
    assert seq.try_dispatch() == 'part_1'

    assert seq.on_terminal('part_0', 'arm1', 'COMPLETED') == (False, [])  # 迟到旧件
    assert seq.on_terminal('part_1', 'arm1', 'PICKING') == (False, [])    # 非终态
    assert seq.on_terminal('part_1', 'arm2', 'COMPLETED') == (False, [])  # 阶段不匹配
    assert seq.current == 'part_1'

    assert seq.on_terminal('part_1', 'arm1', 'COMPLETED')[0]
    # 等待接力期间:重复的 arm1 终态不得提前结束该件
    assert seq.on_terminal('part_1', 'arm1', 'COMPLETED') == (False, [])
    assert seq.on_terminal('part_1', 'arm1', 'FAILED') == (False, [])
    # arm2 终态在接力派发前到达 -> 忽略
    assert seq.on_terminal('part_1', 'arm2', 'COMPLETED') == (False, [])
    assert seq.current == 'part_1'
    assert seq.phase == 'wait_relay'

    assert seq.on_relay_ready() == 'part_1'
    # relay timer 重复触发 -> 幂等无害
    assert seq.on_relay_ready() is None

    assert seq.on_terminal('part_1', 'arm2', 'COMPLETED')[0]
    # 该件已结束:后续任何迟到/重复消息忽略
    assert seq.on_terminal('part_1', 'arm2', 'COMPLETED') == (False, [])
    assert seq.on_terminal('part_1', 'arm2', 'FAILED') == (False, [])
    assert seq.on_terminal('part_1', '', 'COMPLETED') == (False, [])


def test_empty_arm_treated_as_arm1():
    """旧消息 arm 为空串,视为 arm1(兼容)。"""
    seq = make(relay_mode=True)
    seq.enqueue('p1')
    seq.try_dispatch()
    accepted, actions = seq.on_terminal('p1', '', 'COMPLETED')
    assert accepted and actions == [(RELAY, 0.5)]
    assert seq.phase == 'wait_relay'

    seq2 = make(relay_mode=False)
    seq2.enqueue('p1')
    seq2.try_dispatch()
    accepted, _ = seq2.on_terminal('p1', '', 'FAILED')
    assert accepted and seq2.is_done('p1')


def test_relay_mode_false_matches_legacy():
    """relay_mode=false:与旧单臂调度等价(终态即该件结束)。"""
    seq = make(relay_mode=False)
    seq.enqueue('part_1')
    assert seq.try_dispatch() == 'part_1'
    # 队列空:受理但无需安排下一件
    accepted, actions = seq.on_terminal('part_1', 'arm1', 'COMPLETED')
    assert accepted and actions == []
    assert seq.is_done('part_1') and seq.current is None

    seq.enqueue('part_2')
    assert seq.try_dispatch() == 'part_2'
    # 单臂模式下不存在 arm2 阶段,arm2 标记的消息忽略
    assert seq.on_terminal('part_2', 'arm2', 'COMPLETED') == (False, [])
    accepted, actions = seq.on_terminal('part_2', '', 'FAILED')
    assert accepted and actions == []
    assert seq.is_done('part_2')


def test_block_reason_names_why_enqueue_failed():
    """单件派发被拒绝时,能区分正在执行、已在队列和已经完成。"""
    seq = RelaySequencer(relay_mode=False, dispatch_delay=1.0)
    assert seq.block_reason('part_1') is None
    assert seq.enqueue('part_1') is True
    assert seq.block_reason('part_1') == 'queued'
    assert seq.try_dispatch('arm1') == 'part_1'
    assert seq.block_reason('part_1') == 'current'
    seq.on_terminal('part_1', 'arm1', 'COMPLETED')
    assert seq.block_reason('part_1') == 'done'


def test_enqueue_skips_part_being_executed():
    """正在执行的件不在排队集合里,start_batch 也不能把它再排到队尾。"""
    seq = RelaySequencer(relay_mode=False, dispatch_delay=1.0)
    for part_id in ('part_1', 'part_2', 'part_3'):
        assert seq.enqueue(part_id) is True
    assert seq.try_dispatch('arm1') == 'part_1'
    assert seq.enqueue('part_1') is False
    assert seq.queue_size == 2
    seq.on_terminal('part_1', 'arm1', 'COMPLETED')
    assert seq.try_dispatch('arm1') == 'part_2'
    assert seq.enqueue('part_1') is False        # 已经完成
    assert seq.queue_size == 1


def test_enqueue_dedup():
    """排队中的件去重;已完成的件不再入队(与旧逻辑一致)。"""
    seq = make()
    assert seq.enqueue('p1') is True
    assert seq.enqueue('p1') is False           # 已在队列
    assert seq.queue_size == 1
    assert seq.try_dispatch() == 'p1'
    seq.on_terminal('p1', 'arm1', 'COMPLETED')
    assert seq.enqueue('p1') is False           # 已完成
    assert seq.is_done('p1')


def test_dispatch_requires_queue_and_idle():
    """派发前置条件:空闲且有队列;重复调用幂等。"""
    seq = make()
    assert seq.try_dispatch() is None           # 空队列
    seq.enqueue('p1')
    assert seq.try_dispatch() == 'p1'
    assert seq.try_dispatch() is None           # 忙碌
    seq.on_terminal('p1', 'arm1', 'COMPLETED')
    assert seq.try_dispatch() is None           # 队列已空


def test_relay_then_arm2_failed_still_advances():
    """arm2 FAILED 同样结束该件并派发下一件(失败不阻塞流水线)。"""
    seq = make(relay_mode=True)
    seq.enqueue('part_1')
    seq.enqueue('part_2')
    assert seq.try_dispatch() == 'part_1'
    assert seq.on_terminal('part_1', 'arm1', 'COMPLETED')[0]
    assert seq.on_relay_ready() == 'part_1'
    accepted, actions = seq.on_terminal('part_1', 'arm2', 'FAILED')
    assert accepted and actions == [(DISPATCH, 0.2)]
    assert seq.is_done('part_1')
    assert seq.try_dispatch() == 'part_2'


# ------------------------------------------------ 并联分色路由(2026-10-04 Phase1)
def test_parallel_route_arm2_direct_terminal():
    """并联路由:try_dispatch('arm2') 直接进 arm2 阶段,arm2 终态即
    完全终态,不产生 RELAY 动作。"""
    seq = make(relay_mode=False)          # 并联激活时节点构造 relay off
    seq.enqueue('p1')
    assert seq.peek() == 'p1' and seq.queue_size == 1   # peek 不出队
    assert seq.try_dispatch('arm2') == 'p1'
    assert seq.phase == 'arm2'
    accepted, actions = seq.on_terminal('p1', 'arm2', 'COMPLETED')
    assert accepted and actions == []
    assert seq.is_done('p1')


def test_parallel_route_ignores_arm1_terminal():
    """arm2 路由件的 arm1 迟到终态按阶段不匹配忽略,不误结该件。"""
    seq = make(relay_mode=False)
    seq.enqueue('p1')
    assert seq.try_dispatch('arm2') == 'p1'
    assert seq.on_terminal('p1', 'arm1', 'COMPLETED') == (False, [])
    assert not seq.is_done('p1')
    assert seq.on_terminal('p1', 'arm2', 'FAILED')[0]   # 真终态仍推进
    assert seq.is_done('p1')


def test_parallel_arm1_route_unchanged():
    """非路由色走 arm1 路径:relay off 时 arm1 终态即完全终态(旧行为)。"""
    seq = make(relay_mode=False)
    seq.enqueue('p1')
    seq.enqueue('p2')
    assert seq.try_dispatch('arm1') == 'p1'
    assert seq.phase == 'arm1'
    accepted, actions = seq.on_terminal('p1', 'arm1', 'COMPLETED')
    assert accepted and actions == [(DISPATCH, 0.2)]
    assert seq.try_dispatch() == 'p2'                   # 默认 arm='arm1'


def test_peek_empty_and_busy():
    """peek:空队返回 None;执行中(队列空)也返回 None。"""
    seq = make()
    assert seq.peek() is None
    seq.enqueue('p1')
    assert seq.try_dispatch() == 'p1'
    assert seq.peek() is None
