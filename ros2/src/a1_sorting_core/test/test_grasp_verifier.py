"""夹取验证(GraspVerifier)单元测试:纯逻辑,不初始化 rclpy。

被测实现的窗口语义(见 grasp_verifier 模块注释,此处摘要):
- 开窗:从窗首起夹爪净位移首次"严格大于" 0.03 处截断开窗,恰 0.03
  不开窗;尾部残余不足一段不开窗;夹爪没动 -> 无窗;
- 窗内跟随:工件位移 >= 0.02(含边界)且夹爪/工件位移向量点积 > 0
  (垂直点积为 0,不算方向一致);
- 多窗以主导窗为准 = 夹爪位移模最大(并列取最早),整体验证结果
  只看主导窗。
"""
from a1_sorting_core.grasp_verifier import GraspVerifier


def _line(p0, p1, n):
    """从 p0 到 p1 的 n 个等距采样(含端点),模拟时间对齐轨迹。"""
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1)) for i in range(n)]


def test_full_follow_pass():
    """完全跟随:夹爪走 0.10,工件同向跟 0.08 -> 通过。"""
    v = GraspVerifier()
    ok, reason = v.verify(_line((0.0, 0.0), (0.10, 0.0), 11),
                          _line((0.0, 0.0), (0.08, 0.0), 11))
    assert ok is True
    assert reason == 'following'


def test_part_static_while_gripper_moves_fails():
    """未跟随:夹爪走 0.10,工件原地不动 -> 不通过。"""
    v = GraspVerifier()
    ok, reason = v.verify(_line((0.0, 0.0), (0.10, 0.0), 11),
                          [(0.44, 0.0)] * 11)
    assert ok is False
    assert reason == 'part_move_below_threshold'


def test_gripper_static_fails():
    """夹爪没动 -> 无窗不通过;工件自己动/全静止都一样。"""
    v = GraspVerifier()
    ok, reason = v.verify([(0.44, 0.0)] * 5,
                          _line((0.0, 0.0), (0.05, 0.0), 5))
    assert ok is False and reason == 'gripper_not_moved'
    ok2, reason2 = v.verify([(0.44, 0.0)] * 5, [(0.44, 0.0)] * 5)
    assert ok2 is False and reason2 == 'gripper_not_moved'


def test_too_few_samples_fails():
    """空序列或长度 1(不足一个位移段)-> 不通过。"""
    v = GraspVerifier()
    assert v.verify([], []) == (False, 'too_few_samples')
    assert v.verify([(0.0, 0.0)], [(0.0, 0.0)]) == (False, 'too_few_samples')


def test_threshold_boundaries():
    """阈值边界:夹爪恰 0.03 不开窗;工件恰 0.02(含边界)算跟随;
    位移垂直点积恰 0 不算方向一致。"""
    v = GraspVerifier()
    # 夹爪位移恰 0.03(严格大于才开窗)-> 无窗
    ok, reason = v.verify([(0.0, 0.0), (0.03, 0.0)],
                          [(0.0, 0.0), (0.02, 0.0)])
    assert ok is False and reason == 'gripper_not_moved'
    # 夹爪 0.031 开窗,工件恰 0.02(>= 含边界)-> 跟随成立
    ok2, reason2 = v.verify([(0.0, 0.0), (0.031, 0.0)],
                            [(0.0, 0.0), (0.02, 0.0)])
    assert ok2 is True and reason2 == 'following'
    # 工件位移 0.02 但方向与夹爪垂直(点积恰 0,不严格大于)-> 不通过
    ok3, reason3 = v.verify([(0.0, 0.0), (0.06, 0.0)],
                            [(0.0, 0.0), (0.0, 0.02)])
    assert ok3 is False and reason3 == 'direction_mismatch'


def test_dominant_window_decides():
    """部分窗口跟随部分不跟:以主导窗(夹爪位移模最大)为准。

    序列 [(0,0), A, B] 被切成相邻两窗 [0,A] 与 [A,B]:
    - 窗 1 夹爪 0.04 工件跟 0.03,窗 2 夹爪 0.12 工件原地 ->
      主导窗 = 窗 2,判不跟随;
    - 反过来窗 1 夹爪 0.12 工件跟 0.06,窗 2 夹爪 0.04 工件原地 ->
      主导窗 = 窗 1,判跟随。
    """
    v = GraspVerifier()
    ok, reason = v.verify([(0.0, 0.0), (0.04, 0.0), (0.16, 0.0)],
                          [(0.0, 0.0), (0.03, 0.0), (0.03, 0.0)])
    assert ok is False and reason == 'part_move_below_threshold'
    # 主导窗明细可供诊断复核
    assert [w['following'] for w in v.last_windows] == [True, False]

    ok2, reason2 = v.verify([(0.0, 0.0), (0.12, 0.0), (0.16, 0.0)],
                            [(0.0, 0.0), (0.06, 0.0), (0.06, 0.0)])
    assert ok2 is True and reason2 == 'following'
    assert [w['following'] for w in v.last_windows] == [True, False]


def test_direction_mismatch_fails():
    """工件动了 0.05 但方向与夹爪相反(点积为负)-> 不通过。"""
    v = GraspVerifier()
    ok, reason = v.verify([(0.0, 0.0), (0.10, 0.0)],
                          [(0.0, 0.0), (-0.05, 0.0)])
    assert ok is False and reason == 'direction_mismatch'


def test_length_mismatch_fails():
    """两序列长度不等(时间未对齐)-> 防御性不通过。"""
    v = GraspVerifier()
    ok, reason = v.verify([(0.0, 0.0), (0.05, 0.0), (0.10, 0.0)],
                          [(0.0, 0.0)])
    assert ok is False and reason == 'length_mismatch'


def test_diagonal_follow_custom_thresholds_and_determinism():
    """斜向跟随 + 构造阈值可覆盖 + 同输入同输出(确定性)。"""
    v = GraspVerifier(gripper_move_thresh=0.05, follow_move_thresh=0.01,
                      direction_dot_thresh=-1e-4)
    g = _line((0.0, 0.0), (0.05, 0.05), 7)
    p = _line((0.0, 0.0), (0.03, 0.03), 7)
    ok, reason = v.verify(g, p)
    assert ok is True and reason == 'following'
    assert v.verify(g, p) == (ok, reason)
