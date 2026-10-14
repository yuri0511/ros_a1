"""策略编号、奖励和 Q 表更新。不需要 ROS。"""
import random
from pathlib import Path

from a1_sorting_core.batch_protocol import batch_id_from_spawn_message
from a1_sorting_core.q_learning import (TabularQ, placement_reward, state_key)
from a1_sorting_core.strategies import all_strategies, grasp_fails, strategy_by_id


def test_three_speeds_share_the_same_height():
    items = all_strategies()
    assert [item.velocity_scaling for item in items] == [0.3, 0.6, 1.0]
    assert [item.pre_grasp_height for item in items] == [0.10, 0.10, 0.10]
    assert strategy_by_id(1).name == 'mid'
    assert strategy_by_id(9) is None
    assert strategy_by_id('nope') is None


def test_grasp_failure_rates_stay_in_config_and_repeat():
    text = (Path(__file__).resolve().parents[1] / 'config' / 'sorting_params.yaml').read_text(
        encoding='utf-8')
    assert 'grasp_fail_seed: 7' in text
    assert 'grasp_fail_rate_slow: 0.02' in text
    assert 'grasp_fail_rate_mid: 0.05' in text
    assert 'grasp_fail_rate_fast: 0.15' in text

    def sequence():
        rng = random.Random(7)
        return [grasp_fails(rng, 0.15) for _ in range(30)]

    assert sequence() == sequence()
    assert any(sequence()) and not all(sequence())
    quiet = random.Random(7)
    assert grasp_fails(quiet, 0.0) is False
    fresh = random.Random(7)
    assert quiet.random() == fresh.random()
    assert grasp_fails(random.Random(1), 1.0) is True
    rng = random.Random(7)
    draws = [grasp_fails(rng, 0.15) for _ in range(4000)]
    ratio = sum(draws) / float(len(draws))
    assert 0.12 < ratio < 0.18


def test_state_uses_the_fixed_bin_rule():
    assert state_key('red') == 'red|bin_A'
    assert state_key('green') == 'green|bin_B'


def test_reward_pays_for_placement_and_charges_time():
    assert abs(placement_reward('PLACED', 10) - 8.0) < 1e-9
    assert abs(placement_reward('FAILED', 10) - (-12.0)) < 1e-9
    assert placement_reward('MISPLACED', 0) == -10.0


def test_update_moves_toward_reward_and_freeze_stops_it():
    learner = TabularQ([0, 1, 2], alpha=0.5, gamma=0.0, epsilon=0.0, seed=1)
    learner.update('red|bin_A', 2, 8.0)
    assert abs(learner.q['red|bin_A'][2] - 4.0) < 1e-9
    learner.freeze()
    assert learner.choose('red|bin_A') == 2
    try:
        learner.update('red|bin_A', 2, 8.0)
    except RuntimeError:
        return
    raise AssertionError('冻结后仍然更新了 Q 表')


def test_spawn_message_batch_id():
    text = '生成 3 件(batch_0002): part_1=red, part_2=green, part_3=red'
    assert batch_id_from_spawn_message(text) == 'batch_0002'
    assert batch_id_from_spawn_message('失败') == ''
