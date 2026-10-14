#!/usr/bin/env python3
"""从策略实测和 Q 学习的 CSV 画图，并写 summary.txt。

不需要 ROS。默认读取 20.04 上实验写出的两个 CSV。

    python3 analysis.py --out-dir ~/.ros/a1_analysis
"""
from __future__ import annotations

import argparse
import os
import sys

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--strategy-csv',
                        default=os.path.expanduser('~/.ros/a1_strategy_compare.csv'))
    parser.add_argument('--q-csv',
                        default=os.path.expanduser('~/.ros/a1_qlearn_steps.csv'))
    parser.add_argument('--out-dir',
                        default=os.path.expanduser('~/.ros/a1_analysis'))
    args = parser.parse_args()
    # 开发机没有安装到 ROS 路径时，从源码包导入。
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
    if root not in sys.path:
        sys.path.insert(0, root)
    from a1_sorting_core.analysis_report import write_report
    strategy_csv = args.strategy_csv if os.path.exists(args.strategy_csv) else None
    q_csv = args.q_csv if os.path.exists(args.q_csv) else None
    text = write_report(strategy_csv, q_csv, args.out_dir)
    print(text, end='')
    print('图写在 %s' % args.out_dir)


if __name__ == '__main__':
    main()
