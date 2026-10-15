"""夹取验证(纯逻辑,与 ROS 解耦,便于单元测试)。

物理包夹是否成功的判定依据(契约 §8):闭爪抬升后,被夹住的工件应在
水平面内跟随夹爪运动。本模块输入时间对齐的夹爪 xy 采样序列与工件 xy
采样序列,输出 (是否跟随, 原因);executor 集成方(verified_only 模式)
在抬升完成后调用,通过才 attach 兜底,失败抛 GraspError(
'grasp_not_verified')(异常语义在 executor 侧,本模块不依赖 rclpy)。

判定语义(确定性,同输入同输出):
1. 预检:两序列长度不等 -> (False, 'length_mismatch');长度 < 2 ->
   (False, 'too_few_samples')。
2. 开窗:从窗起点起,夹爪净位移(窗首到当前采量的直线距离)首次
   严格大于 gripper_move_thresh 处截断开窗,并从该采样点起开下一窗
   (贪婪切块,不重叠;恰等于阈值不开窗;尾部残余不足一段不开窗)。
   夹爪没动/全静止 -> 无窗 -> (False, 'gripper_not_moved')。
3. 窗判定:窗内工件位移 |p| >= follow_move_thresh 且夹爪位移向量 g
   与工件位移向量 p 的点积 > direction_dot_thresh(位移方向一致),
   二者同时成立才算该窗"跟随"。
4. 多窗时以主导窗为准 = 夹爪位移模最大的窗(并列取最早出现的);
   主导窗跟随 -> (True, 'following');否则按主导窗的具体失败原因
   返回 (False, 'part_move_below_threshold' / 'direction_mismatch')。

诊断:verify() 后 self.last_windows 保留各窗的起止下标、夹爪/工件
位移与点积(采样点不带时间戳,窗只以采样下标记),集成方可用于打日志。
"""
from __future__ import annotations

import math
from typing import Sequence

# verify 失败原因常量(集成方按需比对,不要内联字符串)
REASON_LENGTH_MISMATCH = 'length_mismatch'
REASON_TOO_FEW_SAMPLES = 'too_few_samples'
REASON_GRIPPER_NOT_MOVED = 'gripper_not_moved'
REASON_PART_MOVE_BELOW = 'part_move_below_threshold'
REASON_DIRECTION_MISMATCH = 'direction_mismatch'
REASON_FOLLOWING = 'following'


class GraspVerifier:
    """工件是否跟随夹爪移动的判定器(阈值构造可覆盖,契约 §8)。"""

    def __init__(self, gripper_move_thresh: float = 0.03,
                 follow_move_thresh: float = 0.02,
                 direction_dot_thresh: float = 0.0):
        self.gripper_move_thresh = float(gripper_move_thresh)
        self.follow_move_thresh = float(follow_move_thresh)
        self.direction_dot_thresh = float(direction_dot_thresh)
        # 上次 verify 的分窗明细(诊断用,不参与判定)
        self.last_windows: list[dict] = []

    # ---------------------------------------------------------------- 内部
    @staticmethod
    def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def _split_windows(self, g: list[tuple[float, float]]
                       ) -> list[tuple[int, int, float]]:
        """贪婪切块(见模块注释第 2 条),返回 (start, end, 夹爪净位移)。"""
        windows: list[tuple[int, int, float]] = []
        start = 0
        for i in range(1, len(g)):
            move = self._dist(g[start], g[i])
            if move > self.gripper_move_thresh:
                windows.append((start, i, move))
                start = i          # 新窗从当前采样点起(相邻窗共享端点)
        return windows

    # ---------------------------------------------------------------- 接口
    def verify(self, gripper_xy: Sequence[Sequence[float]],
               part_xy: Sequence[Sequence[float]]) -> tuple[bool, str]:
        """判定工件是否跟随夹爪(语义见模块注释)。

        gripper_xy / part_xy:时间对齐、等长的 xy 采样序列(先行后列,
        每个采样是任意的 2 元素可索引序列;只看水平面,不含 z)。
        """
        self.last_windows = []
        if len(gripper_xy) != len(part_xy):
            return (False, REASON_LENGTH_MISMATCH)
        if len(gripper_xy) < 2:
            return (False, REASON_TOO_FEW_SAMPLES)
        g = [(float(s[0]), float(s[1])) for s in gripper_xy]
        p = [(float(s[0]), float(s[1])) for s in part_xy]

        spans = self._split_windows(g)
        if not spans:
            return (False, REASON_GRIPPER_NOT_MOVED)

        windows = []
        for start, end, gripper_move in spans:
            gv = (g[end][0] - g[start][0], g[end][1] - g[start][1])
            pv = (p[end][0] - p[start][0], p[end][1] - p[start][1])
            part_move = math.hypot(pv[0], pv[1])
            dot = gv[0] * pv[0] + gv[1] * pv[1]
            windows.append({
                'start': start, 'end': end,
                'gripper_move': gripper_move, 'part_move': part_move,
                'dot': dot,
                'following': (part_move >= self.follow_move_thresh
                              and dot > self.direction_dot_thresh),
            })
        self.last_windows = windows

        # 主导窗 = 夹爪位移模最大;max 对并列返回最先出现的(确定性)
        dominant = max(windows, key=lambda w: w['gripper_move'])
        if dominant['part_move'] < self.follow_move_thresh:
            return (False, REASON_PART_MOVE_BELOW)
        if dominant['dot'] <= self.direction_dot_thresh:
            return (False, REASON_DIRECTION_MISMATCH)
        return (True, REASON_FOLLOWING)
