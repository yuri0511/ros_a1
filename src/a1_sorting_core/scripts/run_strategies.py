#!/usr/bin/env python3
"""实测三种取放策略。仿真需要先启动，并且 auto_start 为 false。

    roslaunch a1_sorting_core sorting_demo.launch auto_start:=false use_rviz:=false
    rosrun a1_sorting_core run_strategies.py --rounds 2

每种策略跑若干轮，每轮三件 red,green,red。结果写到 ~/.ros/a1_strategy_compare.csv。
成功只认记录器的 PLACED。耗时用记录器的 elapsed_sec。
"""
from __future__ import annotations

import csv
import os

import rospy

from a1_sorting_core.batch_runner import BatchRunner
from a1_sorting_core.q_learning import state_key
from a1_sorting_core.strategies import all_strategies

COLUMNS = ['strategy_id', 'strategy_name', 'velocity_scaling', 'pre_grasp_height',
           'round', 'part_id', 'color', 'state', 'result', 'detail',
           'elapsed_sec', 'batch_id', 'task_id']


def run(rounds, colors, timeout, out_path):
    runner = BatchRunner()
    rows = []
    for spec in all_strategies():
        for round_index in range(1, rounds + 1):
            runner.use_one_strategy(spec.id)
            print('策略 %s 第 %d 轮' % (spec.name, round_index), flush=True)
            batch_id, part_ids = runner.run_parts(colors, timeout)
            for part_id, color in zip(part_ids, colors):
                msg = runner.results.get(part_id)
                if msg is None:
                    result, detail, elapsed, task_id = 'TIMEOUT', 'timeout', '', ''
                else:
                    result = msg.result
                    detail = msg.detail
                    elapsed = '%.3f' % float(msg.elapsed_sec or 0.0)
                    task_id = msg.task_id
                rows.append([
                    spec.id, spec.name, '%.2f' % spec.velocity_scaling,
                    '%.3f' % spec.pre_grasp_height, round_index, part_id, color,
                    state_key(color), result, detail, elapsed, batch_id, task_id])
                print('  %s %s %s %ss' % (part_id, color, result, elapsed or '?'), flush=True)
    directory = os.path.dirname(out_path)
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(out_path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    return rows


def summarize(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(row[1], []).append(row)
    lines = []
    for name in ('slow', 'mid', 'fast'):
        items = grouped.get(name, [])
        if not items:
            continue
        placed = sum(1 for item in items if item[8] == 'PLACED')
        elapsed = []
        for item in items:
            try:
                elapsed.append(float(item[10]))
            except ValueError:
                pass
        mean = sum(elapsed) / len(elapsed) if elapsed else 0.0
        lines.append('%s: %d/%d PLACED, 平均耗时 %.3f 秒' % (name, placed, len(items), mean))
    return lines


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--colors', default='red,green,red')
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--out', default=os.path.expanduser('~/.ros/a1_strategy_compare.csv'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('run_strategies')
    colors = [item.strip() for item in args.colors.split(',') if item.strip()]
    rows = run(args.rounds, colors, args.timeout, args.out)
    print('写入 %s' % args.out, flush=True)
    for line in summarize(rows):
        print(line, flush=True)


if __name__ == '__main__':
    main()
