#!/usr/bin/env python3
"""用记录器的真实结果训练表格 Q，然后冻结再跑几件。

仿真需要先启动，并且 auto_start 为 false。每次只生成一件，做完就更新。

    rosrun a1_sorting_core run_qlearn.py --train-parts 30 --eval-parts 6

Q 表写到 ~/.ros/a1_q_table.json。每件奖励写到 ~/.ros/a1_qlearn_steps.csv。
"""
from __future__ import annotations

import csv
import os

import rospy

from a1_sorting_core.batch_runner import BatchRunner
from a1_sorting_core.q_learning import (TabularQ, epsilon_schedule,
                                         placement_reward, state_key)
from a1_sorting_core.strategies import all_strategies, strategy_by_id

COLUMNS = ['phase', 'episode', 'state', 'action', 'strategy_name',
           'velocity_scaling', 'part_id', 'color', 'result', 'detail',
           'elapsed_sec', 'reward', 'batch_id', 'task_id', 'epsilon', 'condition']
COMPARE_CONDITIONS = ('frozen', 'slow', 'mid', 'fast')


def _strategy_named(name):
    for item in all_strategies():
        if item.name == name:
            return item
    raise KeyError(name)


def _one_part(runner, learner, color, timeout, phase, episode, update,
              epsilon, condition, forced_action=None):
    state = state_key(color)
    if forced_action is None:
        action = learner.choose(state, epsilon)
    else:
        action = int(forced_action)
    spec = strategy_by_id(action)
    runner.use_part_strategies({'part_1': action})
    batch_id, part_ids = runner.run_parts([color], timeout)
    msg = runner.results.get(part_ids[0])
    if msg is None:
        # 没有记录器耗时。不用整段等待时间冒充搬运耗时，失败分只记 -10。
        result, detail, elapsed, task_id = 'TIMEOUT', 'timeout', 0.0, ''
        elapsed_text = ''
    else:
        result = msg.result
        detail = msg.detail
        elapsed = float(msg.elapsed_sec or 0.0)
        elapsed_text = '%.3f' % elapsed
        task_id = msg.task_id
    reward = placement_reward(result, elapsed, learner.time_coef)
    if update:
        learner.update(state, action, reward)
    print('%s #%d %s 动作 %s -> %s 奖励 %.3f' % (
        phase, episode, state, spec.name, result, reward), flush=True)
    return [phase, episode, state, action, spec.name, '%.2f' % spec.velocity_scaling,
            part_ids[0], color, result, detail, elapsed_text, '%.3f' % reward,
            batch_id, task_id, '%.3f' % float(epsilon), condition]


def _write_header(path):
    directory = os.path.dirname(path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    handle = open(path, 'w', newline='', encoding='utf-8')
    writer = csv.writer(handle)
    writer.writerow(COLUMNS)
    handle.flush()
    return handle, writer


def run(train_parts, eval_parts, colors, timeout, table_path, csv_path,
        epsilon_start, epsilon_end, compare_parts, compare_path):
    runner = BatchRunner()
    learner = TabularQ([item.id for item in all_strategies()], epsilon=epsilon_start)
    learner.epsilon_start = float(epsilon_start)
    learner.epsilon_end = float(epsilon_end)
    rows = []
    handle, writer = _write_header(csv_path)
    try:
        for episode in range(1, train_parts + 1):
            color = colors[(episode - 1) % len(colors)]
            epsilon = epsilon_schedule(
                episode - 1, train_parts, epsilon_start, epsilon_end)
            learner.epsilon = epsilon
            row = _one_part(
                runner, learner, color, timeout, 'train', episode, True,
                epsilon, 'train')
            rows.append(row)
            writer.writerow(row)
            handle.flush()
            learner.save(table_path)
        learner.freeze()
        learner.save(table_path)
        for offset in range(1, eval_parts + 1):
            color = colors[(train_parts + offset - 1) % len(colors)]
            row = _one_part(
                runner, learner, color, timeout, 'frozen', train_parts + offset,
                False, 0.0, 'frozen')
            rows.append(row)
            writer.writerow(row)
            handle.flush()
    finally:
        handle.close()
    compare_rows = []
    if compare_parts > 0:
        compare_rows = compare_policies(
            runner, learner, colors, compare_parts, timeout, compare_path)
    return learner, rows, compare_rows


def compare_policies(runner, learner, colors, count, timeout, path):
    """冻结策略和三个固定档各跑同一串颜色。每一档开始前把抓取种子拨回配置值。"""
    rows = []
    handle, writer = _write_header(path)
    try:
        for condition in COMPARE_CONDITIONS:
            runner.reset_grasp_seed()
            forced = None
            if condition != 'frozen':
                forced = _strategy_named(condition).id
            print('对比 %s，抓取种子已重置，%d 件' % (condition, count), flush=True)
            for index in range(1, count + 1):
                color = colors[(index - 1) % len(colors)]
                row = _one_part(
                    runner, learner, color, timeout, 'compare', index, False,
                    0.0, condition, forced)
                rows.append(row)
                writer.writerow(row)
                handle.flush()
    finally:
        handle.close()
    return rows


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--train-parts', type=int, default=30)
    parser.add_argument('--eval-parts', type=int, default=6)
    parser.add_argument('--colors', default='red,green')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--epsilon-start', type=float, default=0.5)
    parser.add_argument('--epsilon-end', type=float, default=0.05)
    parser.add_argument('--compare-parts', type=int, default=0)
    parser.add_argument('--table', default=os.path.expanduser('~/.ros/a1_q_table.json'))
    parser.add_argument('--out', default=os.path.expanduser('~/.ros/a1_qlearn_steps.csv'))
    parser.add_argument('--compare-out',
                        default=os.path.expanduser('~/.ros/a1_qlearn_compare.csv'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('run_qlearn')
    colors = [item.strip() for item in args.colors.split(',') if item.strip()]
    _learner, rows, compare_rows = run(
        args.train_parts, args.eval_parts, colors, args.timeout,
        args.table, args.out, args.epsilon_start, args.epsilon_end,
        args.compare_parts, args.compare_out)
    print('Q 表 %s' % args.table, flush=True)
    print('奖励 %s，共 %d 行' % (args.out, len(rows)), flush=True)
    if compare_rows:
        print('对比 %s，共 %d 行' % (args.compare_out, len(compare_rows)), flush=True)


if __name__ == '__main__':
    main()
