"""用已经训好的 Q 表决定下一件推什么颜色。

表和计时配置只读，本模块不改那两个 JSON。
忙着的臂不会被提前送料：皮带上仍一次只有一件，而且这件只交给现在空闲的臂。
表里没有这个状态时，沿用 Q 表代码里的规则（先给空闲的臂，同臂再按颜色序号）。
"""
import copy


COLOR_INDEX = {'blue': 0, 'yellow': 1, 'red': 2, 'green': 3}
COLORS = ('blue', 'yellow', 'red', 'green')
ARM_NAMES = ('arm1', 'arm2')
WAIT = 4
DEFAULT_PACKAGE_DIR = '/home/yuri/lead_ws/ai-collaboration/codex_rl'


def _import_dual_arm(package_dir):
    import sys
    root = package_dir or DEFAULT_PACKAGE_DIR
    if root not in sys.path:
        sys.path.insert(0, root)
    from dual_arm_rl.model import Model
    from dual_arm_rl.qtable import choose, load
    from dual_arm_rl.model import load_config
    return Model, choose, load, load_config


def load_q_policy(package_dir, config_path, table_path):
    """读配置和表。两份文件里的 overlap_validated 必须一致，否则拒绝加载。"""
    _, _, load, load_config = _import_dual_arm(package_dir)
    config = load_config(config_path)
    table = load(table_path, config)
    return config, table


def decide_feed(config, table, remaining, arms, now, package_dir=''):
    """返回 (臂名, 颜色) 或 None。None 表示这步先等，不推新的一件。

    remaining 是蓝、黄、红、绿还没请求的件数。
    arms 是两条臂，每条有 phase、color、phase_start。
    phase 只用 READY 或 POST_PICK。POST_PICK 的 color 必须是 0 到 3。
    now 和 phase_start 都是仿真秒。

    这里把「可以给忙着的臂预送」关掉，只影响这一次判断，不改传入的配置。
    """
    Model, choose, _, _ = _import_dual_arm(package_dir)
    decision_config = copy.copy(config)
    decision_config['allow_prefeed_busy_arm'] = False
    counts = [max(0, int(n)) for n in remaining]
    model = Model(decision_config, counts)
    model.time = float(now)
    for index, obs in enumerate(arms):
        arm = model.arms[index]
        arm.phase = obs['phase']
        arm.color = int(obs['color'])
        arm.phase_start = float(obs['phase_start'])
        if arm.phase == 'POST_PICK' and arm.color not in (0, 1, 2, 3):
            raise ValueError('POST_PICK arm needs a color')
    # 还有臂没回到空闲时，模型才允许「等待」。两条都空闲就没有后续事件，不能干等。
    if any(arm.phase != 'READY' for arm in model.arms):
        model._schedule(1.0, 'ARM_READY', 0)
    stats = {}
    try:
        action = choose(table, model, stats)
    except ValueError:
        return None, stats
    if action == WAIT:
        return None, stats
    arm_index = int(decision_config['color_to_arm'][action])
    if model.arms[arm_index].phase != 'READY':
        return None, stats
    return (ARM_NAMES[arm_index], COLORS[action]), stats
