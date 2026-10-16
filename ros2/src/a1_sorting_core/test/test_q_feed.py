"""Q 表来料选择。不启动 ROS，也不改计时 JSON 和 Q 表文件。"""
import copy
from pathlib import Path

import pytest

from a1_sorting_core.q_feed import (
    decide_feed, describe_q_load, load_q_policy, resolve_q_paths)


def _policy_paths():
    try:
        return resolve_q_paths(
            'codex_rl',
            'codex_rl/config/timing.gazebo-v1.json',
            'codex_rl/artifacts/gazebo-v1/q_table.json',
            anchors=[Path(__file__).resolve()])
    except FileNotFoundError:
        legacy = Path('/home/yuri/lead_ws/ai-collaboration/codex_rl')
        return resolve_q_paths(
            str(legacy),
            str(legacy / 'config' / 'timing.gazebo-v1.json'),
            str(legacy / 'artifacts' / 'gazebo-v1' / 'q_table.json'))


PACKAGE, CONFIG, TABLE = _policy_paths()


def _ready(now=0.0):
    return {'phase': 'READY', 'color': -1, 'phase_start': now}


def _post(color, start):
    return {'phase': 'POST_PICK', 'color': color, 'phase_start': start}


def test_load_messages_name_the_two_outcomes():
    assert describe_q_load(True, '1806 个状态').startswith('已加载 Q 表')
    assert describe_q_load(False, '没有填写 Q 表路径').startswith('退回谁闲推谁')


def test_empty_q_paths_mean_the_idle_rule():
    assert resolve_q_paths('codex_rl', '', '') == ('', '', '')
    assert resolve_q_paths('', 'codex_rl/config/timing.gazebo-v1.json', '') == (
        '', '', '')


def test_relative_paths_find_the_bundled_table_when_present():
    here = Path(__file__).resolve()
    bundled = None
    for parent in here.parents:
        candidate = parent / 'codex_rl' / 'artifacts' / 'gazebo-v1' / 'q_table.json'
        if candidate.is_file():
            bundled = parent
            break
    if bundled is None:
        pytest.skip('this tree does not carry ros2/codex_rl')
    package, config, table = resolve_q_paths(
        'codex_rl',
        'codex_rl/config/timing.gazebo-v1.json',
        'codex_rl/artifacts/gazebo-v1/q_table.json',
        anchors=[here])
    assert Path(package, 'dual_arm_rl', '__init__.py').is_file()
    assert Path(config) == bundled / 'codex_rl' / 'config' / 'timing.gazebo-v1.json'
    assert Path(table) == bundled / 'codex_rl' / 'artifacts' / 'gazebo-v1' / 'q_table.json'
    loaded_config, loaded_table = load_q_policy(package, config, table)
    assert loaded_config['overlap_validated'] is False
    assert loaded_table['overlap_validated'] is False


def test_real_table_initial_state_feeds_red_and_leaves_the_file_false():
    config, table = load_q_policy(PACKAGE, CONFIG, TABLE)
    assert config['overlap_validated'] is False
    assert table['overlap_validated'] is False
    assert table['deployment_ready'] is False
    before = copy.deepcopy(config)
    choice, stats = decide_feed(
        config, table, [2, 2, 2, 2], [_ready(), _ready()], 0.0)
    assert choice == ('arm2', 'red')
    assert stats.get('unknown_state_fallback_steps', 0) == 0
    assert config == before


def test_unknown_state_uses_the_ready_arm_rule():
    config, table = load_q_policy(PACKAGE, CONFIG, TABLE)
    empty = dict(table)
    empty['q_values'] = {}
    choice, stats = decide_feed(
        config, empty, [2, 2, 2, 2], [_ready(), _ready()], 0.0)
    assert choice == ('arm1', 'blue')
    assert stats.get('unknown_state_fallback_steps') == 1


def test_wait_is_kept_when_it_is_better_and_an_arm_is_busy():
    config, table = load_q_policy(PACKAGE, CONFIG, TABLE)
    # 蓝臂刚离区，黄/红/绿的估计让等待的分最高。键必须和 decide_feed 算出的一致。
    # 蓝的离区后估计是 10.85 秒，落在第 3 档（边界 5/10/20，档号从 1 起）。
    key = '[2,2,2,2,"POST_PICK","READY",3,0,0,-1]'
    edited = dict(table)
    edited['q_values'] = {key: [-1, -1, -1, -1, 5]}
    choice, _stats = decide_feed(
        config, edited, [2, 2, 2, 2],
        [_post(0, 0.0), _ready()], 0.0)
    assert choice is None


def test_busy_arm_color_is_not_prefed_even_if_its_value_is_highest():
    config, table = load_q_policy(PACKAGE, CONFIG, TABLE)
    key = '[2,2,2,2,"POST_PICK","READY",3,0,0,-1]'
    edited = dict(table)
    # 蓝和黄属于正忙的 arm1，分再高也不许预送。空闲的 arm2 里绿比红高。
    edited['q_values'] = {key: [100, 100, -5, -1, -20]}
    choice, stats = decide_feed(
        config, edited, [2, 2, 2, 2],
        [_post(0, 0.0), _ready()], 0.0)
    assert choice == ('arm2', 'green')
    assert stats.get('unknown_state_fallback_steps', 0) == 0
