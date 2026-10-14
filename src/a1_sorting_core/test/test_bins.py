"""料区判定与结果台账单元测试(布局 v2 四色四箱口径,纯逻辑)。

v2 世界:bin_blue (0.30,+0.28) / bin_yellow (0.90,+0.28)(arm1 北排)、
bin_red (0.30,-0.28) / bin_green (0.90,-0.28)(arm2 南排);四箱几何与旧
bin_A/bin_B 逐字节同法(内口 120×170mm ⇒ 内口半宽 0.06/0.085,外廓
0.14×0.19 ⇒ 外沿半宽 0.07/0.095,壁顶 0.815)。
旧暖冷两箱 bin_A/bin_B 与 bin_out 接力区已随 v2 废止(docs/layout_v2_spec.md,
binC 世界变体同时删除,历史可考 64cd236/3f02c2b)。
⇒ Bin/judge_position/ResultBook 的**逻辑覆盖逐条保留**,只把钉值从旧世界
   几何换成 v2 真实几何(判定语义与阈值一字未改)。
"""
from a1_sorting_core.bins import (Bin, RESULT_FAILED, RESULT_MISPLACED,
                                  RESULT_NOT_FOUND, RESULT_OUTSIDE,
                                  RESULT_PLACED, ResultBook, judge_position)

# v2 北排两箱(台账判定用内口半宽 0.06/0.085,与 sorting_params*.yaml 的
# bins.*.half_x/half_y 同值 —— 若用外沿 0.07/0.095,蓝/红两箱判定区
# 会在 y=0 带上重叠,正确落南排的件会被先判北排 = MISPLACED 误报,
# 旧 0.12/0.10 定罪过的同型坑,口径不变)
BINS = [Bin('bin_blue', 0.30, 0.28, 0.06, 0.085),
        Bin('bin_yellow', 0.90, 0.28, 0.06, 0.085)]


def test_contains_boundaries():
    a = BINS[0]
    assert a.contains(0.30, 0.28)                  # 中心
    assert a.contains(0.24, 0.28)                  # x 内口沿(含边界,0.30-0.06)
    assert not a.contains(0.2399, 0.28)            # x 边界外
    assert a.contains(0.30, 0.20)                  # y 口内沿(dy=0.08<0.085)
    assert not a.contains(0.30, 0.19)              # y 越过南内口沿(dy=0.09)
    assert not a.contains(0.30, 0.37)              # y 越过北内口沿(dy=0.09)
    assert not a.contains(0.90, 0.28)              # 属黄箱
    # 说明:y 的"恰在沿上"取 0.28±0.085 会踩 float 表示误差
    # (0.28-0.085=0.19499999999999998 而 |Δy|=0.08500000000000002>0.085),
    # 属实现细节不是判据,故含边界断言放在 x 维(0.30-0.06=0.24 精确可表示)。


def test_judge_position():
    assert judge_position(0.30, 0.28, BINS) == 'bin_blue'
    assert judge_position(0.90 + 0.05, 0.28 + 0.08, BINS) == 'bin_yellow'
    assert judge_position(0.60, 0.00, BINS) is None    # 带尾停位(拉长带后 0.60),不在料区
    assert judge_position(0.36, 0.00, BINS) is None    # 旧停位同样不在料区
    assert judge_position(0.30, -0.28, BINS) is None   # 南排箱不在本表内


def test_result_book_csv(tmp_path):
    path = tmp_path / 'results.csv'
    book = ResultBook(str(path), ['bin_blue', 'bin_yellow'])

    book.record('part_1', 'blue', 'bin_blue', 'bin_blue', RESULT_PLACED)
    book.record('part_2', 'yellow', 'bin_yellow', 'bin_blue', RESULT_MISPLACED)
    book.record('part_3', 'blue', 'bin_blue', None, RESULT_FAILED)

    lines = path.read_text(encoding='utf-8').strip().splitlines()
    assert lines[0] == ('time,part_id,category,target_bin,final_bin,'
                        'result,arm,phase,batch_id,task_id,ground_truth,'
                        'predicted,commanded_bin,attempt,elapsed_sec')
    assert len(lines) == 4                       # 表头只写一次
    # 旧式调用不传 arm/phase 和批次列 -> 这些列为空串
    assert lines[1].endswith(',bin_blue,bin_blue,PLACED,,,,,,,,,')
    assert lines[2].endswith(',bin_yellow,bin_blue,MISPLACED,,,,,,,,,')
    assert lines[3].endswith(',bin_blue,,FAILED,,,,,,,,,')
    assert book.counts == {'bin_blue': 1, 'bin_yellow': 0,
                           'FAILED': 1, 'MISPLACED': 1}


def test_result_book_counts_by_result(tmp_path):
    book = ResultBook(str(tmp_path / 'r.csv'), ['bin_blue', 'bin_yellow'])
    book.record('p1', 'blue', 'bin_blue', 'bin_blue', RESULT_PLACED)
    book.record('p2', 'blue', 'bin_blue', None, RESULT_NOT_FOUND)
    book.record('p3', 'blue', 'bin_blue', 'table', RESULT_OUTSIDE)
    assert book.counts['bin_blue'] == 1
    assert book.counts['FAILED'] == 1
    assert book.counts['MISPLACED'] == 1         # OUTSIDE 并入放错统计


# --------------------------------------------------------- 双臂并联扩展
def test_result_book_relay_columns(tmp_path):
    """arm/phase 列按行写入;四箱计数各走同名键(由 bin_names 决定)。

    v2 并联分色:arm1 投北排(phase=sort)、arm2 投南排(phase=relay,
    recorder 的 phase='relay' if arm=='arm2' 语义不变)。
    """
    path = tmp_path / 'relay.csv'
    book = ResultBook(str(path), ['bin_blue', 'bin_yellow',
                                  'bin_red', 'bin_green'])
    book.record('part_1', 'blue', 'bin_blue', 'bin_blue', RESULT_PLACED,
                arm='arm1', phase='sort')
    book.record('part_2', 'red', 'bin_red', 'bin_red', RESULT_PLACED,
                arm='arm2', phase='relay')
    book.record('part_3', 'green', 'bin_green', 'bin_red', RESULT_MISPLACED,
                arm='arm2', phase='relay')       # 误投邻箱(不混色口径违反)
    lines = path.read_text(encoding='utf-8').strip().splitlines()
    assert lines[1].endswith(',bin_blue,bin_blue,PLACED,arm1,sort,,,,,,,')
    assert lines[2].endswith(',bin_red,bin_red,PLACED,arm2,relay,,,,,,,')
    assert lines[3].endswith(',bin_green,bin_red,MISPLACED,arm2,relay,,,,,,,')
    assert book.counts['bin_blue'] == 1
    assert book.counts['bin_red'] == 1
    assert book.counts['bin_green'] == 0
    assert book.counts['MISPLACED'] == 1


def test_round_counts_reset_keeps_cumulative(tmp_path):
    """新一轮只清 round_counts。累计 counts 和已写入的 CSV 都还在。"""
    book = ResultBook(str(tmp_path / 'rounds.csv'), ['bin_A', 'bin_B'])
    book.record('part_1', 'red', 'bin_A', 'bin_A', RESULT_PLACED)
    book.record('part_2', 'green', 'bin_B', 'bin_B', RESULT_PLACED)
    assert book.round_counts['bin_A'] == 1
    assert book.round_counts['bin_B'] == 1
    book.start_round()
    assert book.round_counts['bin_A'] == 0
    assert book.round_counts['bin_B'] == 0
    assert book.counts['bin_A'] == 1
    assert book.counts['bin_B'] == 1
    book.record('part_1', 'red', 'bin_A', 'bin_A', RESULT_PLACED,
                batch_id='batch_0002', ground_truth='red', predicted='red',
                commanded_bin='bin_A', attempt='1', elapsed_sec='1.5')
    assert book.round_counts['bin_A'] == 1
    assert book.counts['bin_A'] == 2
    body = (tmp_path / 'rounds.csv').read_text(encoding='utf-8')
    assert 'batch_0002' in body
    assert body.count('\n') == 4                 # 表头 + 3 行,没有因为换轮删历史


def test_judge_position_dual_arm_layout():
    """v2 四箱台账(dual 的 bin_names=四箱):南北排互不重叠、箱间/带尾判 None。"""
    four = [Bin('bin_blue', 0.30, 0.28, 0.06, 0.085),
            Bin('bin_yellow', 0.90, 0.28, 0.06, 0.085),
            Bin('bin_red', 0.30, -0.28, 0.06, 0.085),
            Bin('bin_green', 0.90, -0.28, 0.06, 0.085)]
    assert judge_position(0.90, -0.28, four) == 'bin_green'   # 正常投放
    assert judge_position(0.30, 0.28, four) == 'bin_blue'
    assert judge_position(0.60, 0.00, four) is None    # 带尾取件位(新停位)
    assert judge_position(0.60, 0.00, four) is None    # 两排之间(桌中线)
    assert judge_position(0.30, 0.19, four) is None    # 蓝箱南口沿外(Δy=0.09)
    assert judge_position(0.30, 0.20, four) == 'bin_blue'   # 口内(Δy=0.08)
    # 一箱一色:红件若落在蓝箱 => 判 bin_blue ≠ 目标 bin_red = MISPLACED
    assert judge_position(0.30 + 0.05, 0.28, four) == 'bin_blue'


# --------------------------------------------------------- 带壁箱子内口判定
def test_bin_inner_defaults_to_half():
    """不设内口(None):判定回落 half_x/half_y(平面料区旧行为)。"""
    b = Bin('bin_blue', 0.30, 0.28, 0.07, 0.095)
    assert b.inner_half_x is None and b.inner_half_y is None
    assert b.contains(0.30 + 0.069, 0.28 + 0.094)      # half 边界内
    assert b.contains(0.30 + 0.07, 0.28 + 0.095)       # half 边界(含)
    assert not b.contains(0.30 + 0.071, 0.28)          # half 边界外


def test_bin_inner_opening_contains():
    """内口(契约 §9:pipeline 0.06/0.085)判定:壁区不算入箱内。"""
    b = Bin('bin_yellow', 0.90, 0.28, 0.07, 0.095,
            inner_half_x=0.06, inner_half_y=0.085)
    assert b.contains(0.90, 0.28)                      # 中心
    assert b.contains(0.90 + 0.0599, 0.28 + 0.0849)    # 内口边界内
    assert not b.contains(0.90 + 0.0601, 0.28)         # 内口外 = 壁区
    assert not b.contains(0.90, 0.28 + 0.0851)         # 内口外 = 壁区
    assert not b.contains(0.90 + 0.095, 0.28)          # 箱外
    # 半宽语义:内口严格小于外沿时,原 half 边界处不再算入
    assert not b.contains(0.90 + 0.07, 0.28)           # half 边界(壁区)


def test_bin_inner_partial_fallback():
    """只设一维内口时,另一维独立回落 half(字段级回落)。"""
    b = Bin('bin_red', 0.30, -0.28, 0.07, 0.095, inner_half_x=0.06)
    assert not b.contains(0.30 + 0.065, -0.28)         # x 用内口
    assert b.contains(0.30, -0.28 - 0.09)              # y 回落 half


def test_judge_position_uses_inner_opening():
    """judge_position 用内口:落在壁区/两箱之间的件判不属于任何料区。"""
    bins_ = [Bin('bin_blue', 0.30, 0.28, 0.07, 0.095,
                 inner_half_x=0.06, inner_half_y=0.085),
             Bin('bin_yellow', 0.90, 0.28, 0.07, 0.095,
                 inner_half_x=0.06, inner_half_y=0.085)]
    assert judge_position(0.30, 0.28, bins_) == 'bin_blue'      # 箱内
    assert judge_position(0.90, 0.28, bins_) == 'bin_yellow'
    assert judge_position(0.30, 0.28 + 0.086, bins_) is None    # blue 壁区
    assert judge_position(0.60, 0.28, bins_) is None            # 两箱之间
    # 未设内口的箱子判定不变(混合配置:半程迁移)
    mixed = bins_ + [Bin('bin_green', 0.90, -0.28, 0.07, 0.095)]
    assert judge_position(0.90 + 0.06, -0.28, mixed) == 'bin_green'  # 仍用 half
