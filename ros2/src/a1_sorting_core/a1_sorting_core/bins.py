"""料区判定与结果记录(纯逻辑,与 ROS 解耦,便于单元测试)。"""
import csv
import time
from dataclasses import dataclass
from pathlib import Path

RESULT_PLACED = 'PLACED'      # 放进了目标料区
RESULT_MISPLACED = 'MISPLACED'  # 放进了别的料区
RESULT_OUTSIDE = 'OUTSIDE'    # 完成上报但工件不在任何料区内
RESULT_FAILED = 'FAILED'      # 任务失败,未放置
RESULT_NOT_FOUND = 'NOT_FOUND'  # 完成上报但查不到工件真值位姿


@dataclass
class Bin:
    """矩形料区(俯视 AABB),用于判定工件最终落在哪个料区。

    half_x/half_y:料区外沿半宽(RViz 可视框用);inner_half_x/
    inner_half_y:内口半宽(PLACED 判定用,带壁箱子场景,契约 §9),
    默认 None -> 回落 half_x/half_y(平面料区旧行为完全一致)。
    """

    name: str
    center_x: float
    center_y: float
    half_x: float
    half_y: float
    inner_half_x: float | None = None
    inner_half_y: float | None = None

    def contains(self, x: float, y: float) -> bool:
        """内口判定:工件中心是否落在箱内口 AABB 内(含边界)。"""
        hx = self.half_x if self.inner_half_x is None else self.inner_half_x
        hy = self.half_y if self.inner_half_y is None else self.inner_half_y
        return (abs(x - self.center_x) <= hx
                and abs(y - self.center_y) <= hy)


def judge_position(x: float, y: float, bins: list[Bin]) -> str | None:
    """返回工件所在的料区名;不在任何料区内时返回 None。"""
    for bin_ in bins:
        if bin_.contains(x, y):
            return bin_.name
    return None


class ResultBook:
    """分拣结果台账:逐条写 CSV,并维护各料区计数。

    arm/phase 为双臂接力扩展列(旧调用不传时为空串,列仍写入):
    arm∈{arm1,arm2},phase∈{sort,relay};counts 语义与单臂版一致,
    料区键名由 bin_names 决定(如 arm2 场景的 bin_out 走同名键)。
    """

    HEADER = ['time', 'part_id', 'category', 'target_bin', 'final_bin',
              'result', 'arm', 'phase']

    def __init__(self, csv_path: str, bin_names: list[str]):
        self._path = Path(csv_path).expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._bin_names = bin_names
        self.counts = {name: 0 for name in bin_names}
        self.counts['FAILED'] = 0
        self._write_header_once()

    def _write_header_once(self) -> None:
        if not self._path.exists() or self._path.stat().st_size == 0:
            with open(self._path, 'w', newline='', encoding='utf-8') as f:
                csv.writer(f).writerow(self.HEADER)

    def record(self, part_id: str, category: str, target_bin: str | None,
               final_bin: str | None, result: str, arm: str = '',
               phase: str = '') -> None:
        """追加一条结果并更新计数。

        target_bin/final_bin 可为 None(失败时);arm/phase 默认空串
        (兼容单臂旧调用)。
        """
        with open(self._path, 'a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow([
                time.strftime('%H:%M:%S'), part_id, category,
                target_bin or '', final_bin or '', result, arm, phase])
        if result == RESULT_PLACED and target_bin in self.counts:
            self.counts[target_bin] += 1
        elif result in (RESULT_FAILED, RESULT_NOT_FOUND):
            self.counts['FAILED'] += 1
        elif result in (RESULT_MISPLACED, RESULT_OUTSIDE):
            # 放错料区不计入任何目标料区计数,单独累计
            self.counts['MISPLACED'] = self.counts.get('MISPLACED', 0) + 1
