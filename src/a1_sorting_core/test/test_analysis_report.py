"""分析图使用实验表的列，不读取仿真。"""
import csv
from pathlib import Path

from a1_sorting_core.analysis_report import (mean_reward_from_results,
                                             strategy_summary, write_comparison,
                                             write_report)


def _write(path, rows):
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def test_summary_keeps_failures_in_the_rate(tmp_path):
    path = Path(tmp_path) / 'strategy.csv'
    rows = [
        {'strategy_name': 'fast', 'result': 'PLACED', 'detail': '', 'elapsed_sec': '10'},
        {'strategy_name': 'fast', 'result': 'FAILED', 'detail': 'plan_failed', 'elapsed_sec': '12'},
        {'strategy_name': 'slow', 'result': 'PLACED', 'detail': '', 'elapsed_sec': '20'},
    ]
    _write(path, rows)
    summary = {item['strategy_name']: item for item in strategy_summary(rows)}
    assert summary['fast']['count'] == 2
    assert abs(summary['fast']['success_rate'] - 0.5) < 1e-9
    assert summary['fast']['failures'] == {'plan_failed': 1}
    assert mean_reward_from_results([rows[2]]) > mean_reward_from_results([rows[1]])
    q_path = Path(tmp_path) / 'q.csv'
    _write(q_path, [
        {'phase': 'train', 'reward': '1', 'result': 'PLACED', 'detail': ''},
        {'phase': 'frozen', 'reward': '5', 'result': 'PLACED', 'detail': ''},
    ])
    text = write_report(str(path), str(q_path), Path(tmp_path) / 'out')
    assert 'fast' in text and 'frozen' in text
    assert (Path(tmp_path) / 'out' / 'reward_curve.svg').read_text(encoding='utf-8').startswith('<?xml')
    compare_path = Path(tmp_path) / 'compare.csv'
    _write(compare_path, [
        {'condition': 'frozen', 'result': 'PLACED', 'elapsed_sec': '16', 'reward': '6.8'},
        {'condition': 'mid', 'result': 'FAILED', 'elapsed_sec': '20', 'reward': '-14'},
        {'condition': 'slow', 'result': 'PLACED', 'elapsed_sec': '18', 'reward': '6.4'},
        {'condition': 'fast', 'result': 'PLACED', 'elapsed_sec': '15', 'reward': '7.0'},
    ])
    text = write_comparison(str(compare_path), Path(tmp_path) / 'compare_out')
    assert 'frozen' in text and 'mid' in text and '成功率 0.000' in text
    assert (Path(tmp_path) / 'compare_out' / 'compare_mean_reward.svg').is_file()
