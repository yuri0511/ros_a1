"""Q 表来料选择。不启动 ROS，也不改计时 JSON 和 Q 表文件。"""
import copy

from a1_sorting_core.q_feed import (
    DEFAULT_PACKAGE_DIR, decide_feed, load_q_policy)

CONFIG = (
    '/home/yuri/lead_ws/ai-collaboration/codex_rl/config/timing.gazebo-v1.json')
TABLE = (
    '/home/yuri/lead_ws/ai-collaboration/codex_rl/artifacts/gazebo-v1/q_table.json')


def _ready(now=0.0):
    return {'phase': 'READY', 'color': -1, 'phase_start': now}


def _post(color, start):
    return {'phase': 'POST_PICK', 'color': color, 'phase_start': start}


def test_real_table_initial_state_feeds_red_and_leaves_the_file_false():
    config, table = load_q_policy(DEFAULT_PACKAGE_DIR, CONFIG, TABLE)
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
    config, table = load_q_policy(DEFAULT_PACKAGE_DIR, CONFIG, TABLE)
    empty = dict(table)
    empty['q_values'] = {}
    choice, stats = decide_feed(
        config, empty, [2, 2, 2, 2], [_ready(), _ready()], 0.0)
    assert choice == ('arm1', 'blue')
    assert stats.get('unknown_state_fallback_steps') == 1


def test_wait_is_kept_when_it_is_better_and_an_arm_is_busy():
    config, table = load_q_policy(DEFAULT_PACKAGE_DIR, CONFIG, TABLE)
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
    config, table = load_q_policy(DEFAULT_PACKAGE_DIR, CONFIG, TABLE)
    key = '[2,2,2,2,"POST_PICK","READY",3,0,0,-1]'
    edited = dict(table)
    # 蓝和黄属于正忙的 arm1，分再高也不许预送。空闲的 arm2 里绿比红高。
    edited['q_values'] = {key: [100, 100, -5, -1, -20]}
    choice, stats = decide_feed(
        config, edited, [2, 2, 2, 2],
        [_post(0, 0.0), _ready()], 0.0)
    assert choice == ('arm2', 'green')
    assert stats.get('unknown_state_fallback_steps', 0) == 0
