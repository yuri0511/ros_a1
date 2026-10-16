"""用已经训好的 Q 表决定下一件推什么颜色。

表和计时配置只读，本模块不改那两个 JSON。
忙着的臂不会被提前送料：皮带上仍一次只有一件，而且这件只交给现在空闲的臂。
表里没有这个状态时，沿用 Q 表代码里的规则（先给空闲的臂，同臂再按颜色序号）。
"""
import copy
from pathlib import Path


COLOR_INDEX = {'blue': 0, 'yellow': 1, 'red': 2, 'green': 3}
COLORS = ('blue', 'yellow', 'red', 'green')
ARM_NAMES = ('arm1', 'arm2')
WAIT = 4


def describe_q_load(loaded, detail):
    """启动日志用的两句固定说法。loaded 为真表示表已经读进来。"""
    if loaded:
        return f'已加载 Q 表: {detail}'
    return f'退回谁闲推谁: {detail}'


def resolve_q_paths(package_dir, config_path, table_path, anchors=None):
    """把相对路径变成绝对路径。配置或表有一个空着，就三个都返回空字符串。

    相对路径从 anchors、本文件和当前目录向上找，直到碰上仓库里的 codex_rl。
    绝对路径原样使用，但仍要确认文件或包在那里。
    """
    config_text = str(config_path or '').strip()
    table_text = str(table_path or '').strip()
    package_text = str(package_dir or '').strip()
    if not config_text or not table_text:
        return '', '', ''
    roots = _search_roots(anchors)
    config_abs = _resolve_one(config_text, roots, 'file')
    table_abs = _resolve_one(table_text, roots, 'file')
    if package_text:
        package_abs = _resolve_one(package_text, roots, 'package')
    else:
        package_abs = _package_from_config(config_abs)
    return package_abs, config_abs, table_abs


def _search_roots(anchors):
    roots = []
    seen = set()
    raw = list(anchors or [])
    raw.append(Path(__file__).resolve())
    raw.append(Path.cwd().resolve())
    for anchor in raw:
        path = Path(anchor).resolve()
        start = path if path.is_dir() else path.parent
        for parent in (start, *start.parents):
            if parent in seen:
                continue
            seen.add(parent)
            roots.append(parent)
    return roots


def _resolve_one(text, roots, expect):
    candidate = Path(text).expanduser()
    options = ([candidate] if candidate.is_absolute()
               else [root / candidate for root in roots])
    for option in options:
        if expect == 'file' and option.is_file():
            return str(option.resolve())
        if (expect == 'package'
                and (option / 'dual_arm_rl' / '__init__.py').is_file()):
            return str(option.resolve())
    kind = '文件' if expect == 'file' else '目录'
    raise FileNotFoundError(f'找不到 Q {kind}: {text}')


def _package_from_config(config_abs):
    for parent in Path(config_abs).resolve().parents:
        if (parent / 'dual_arm_rl' / '__init__.py').is_file():
            return str(parent)
    raise FileNotFoundError(f'配置旁边没有 dual_arm_rl 包: {config_abs}')


def _import_dual_arm(package_dir):
    import sys
    root = str(package_dir or '').strip()
    if root and root not in sys.path:
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
