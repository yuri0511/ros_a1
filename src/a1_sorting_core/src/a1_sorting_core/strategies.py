"""三种取放策略。

速度比例不同，预抓高度都是 0.10 米。高度保持不变，后面测到的耗时差
才能归到速度上。编号从 0 开始，Q 表的动作就是这个编号。
"""


class Strategy(object):

    def __init__(self, strategy_id, name, velocity_scaling, pre_grasp_height):
        self.id = int(strategy_id)
        self.name = str(name)
        self.velocity_scaling = float(velocity_scaling)
        self.pre_grasp_height = float(pre_grasp_height)


# 与 sorting_params.yaml 里原来的 0.5 不是同一档。0.5 仍是未指定策略时的默认值。
STRATEGIES = (
    Strategy(0, 'slow', 0.3, 0.10),
    Strategy(1, 'mid', 0.6, 0.10),
    Strategy(2, 'fast', 1.0, 0.10),
)


def strategy_by_id(strategy_id):
    """未知编号返回 None，调用方继续用节点启动时的默认速度。"""
    try:
        wanted = int(strategy_id)
    except (TypeError, ValueError):
        return None
    for item in STRATEGIES:
        if item.id == wanted:
            return item
    return None


def all_strategies():
    return list(STRATEGIES)
