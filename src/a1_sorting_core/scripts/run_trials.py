#!/usr/bin/env python3
"""重复运行测试脚本:自动跑 N 轮分拣并统计成功率(ROS1 移植自组长 run_trials.py,支撑"测试与分析")。

用法(系统已经 roslaunch a1_sorting_core sorting_demo.launch 运行时):
    rosrun a1_sorting_core run_trials.py --trials 3
    rosrun a1_sorting_core run_trials.py --trials 2 --colors "red,green,red" "blue,green,yellow"

每轮流程:reset -> /parts/spawn -> start_batch -> 等待记录器的 /result/part。
成功只认 PLACED(按给定类别落到正确料区),不认执行器自己的 COMPLETED。
计时从 reset 之前开始,包含生成和搬运。超时也会写入 ~/.ros/a1_trials.csv。
"""
import argparse
import csv
import os
import time

import rospy

from a1_interfaces.msg import PartResult
from a1_interfaces.srv import SortTask, SpawnParts
from a1_sorting_core.batch_protocol import PART_RESULT_TOPIC


class TrialRunner(object):

    def __init__(self):
        rospy.Subscriber(PART_RESULT_TOPIC, PartResult, self._on_result)
        for name in ('/sort_task', '/parts/spawn'):
            rospy.wait_for_service(name, 30.0)
        self._sort = rospy.ServiceProxy('/sort_task', SortTask)
        self._spawn = rospy.ServiceProxy('/parts/spawn', SpawnParts)
        self.results = {}   # part_id -> PartResult

    def _on_result(self, msg):
        self.results[msg.part_id] = msg

    def wait_idle(self, timeout=120.0):
        """等调度器空闲才能 reset(上一轮最后一件还在收尾时 reset 会被拒绝)。"""
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self._sort('reset', '').accepted:
                return True
            rospy.sleep(1.0)
        return False

    def run_round(self, colors, timeout):
        parts = ['part_%d' % i for i in range(1, len(colors) + 1)]
        self.results.clear()
        t0 = time.time()
        if not self.wait_idle():
            raise RuntimeError('调度器一直忙,无法 reset')
        resp = self._spawn(','.join(colors))
        if not resp.success:
            raise RuntimeError('生成工件失败: %s' % resp.message)
        self._sort('start_batch', '')
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            if set(parts) <= self.results.keys():
                return time.time() - t0, parts
            rospy.sleep(0.5)
        return None, parts


def _all_done(text):
    """'3/3 完成' -> True;'2/3 完成' / 'TIMEOUT' -> False。"""
    head = text.split(' ')[0]
    if '/' not in head:
        return False
    a, b = head.split('/')
    return a == b


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--colors', nargs='*', default=['red,green,red'],
                        help='每轮的来料颜色序列(逗号分隔),轮数不足时循环使用')
    parser.add_argument('--timeout', type=float, default=240.0)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('run_trials')
    node = TrialRunner()
    color_seqs = [c.split(',') for c in args.colors]
    rows = []
    try:
        for trial in range(1, args.trials + 1):
            colors = color_seqs[(trial - 1) % len(color_seqs)]
            rospy.loginfo('== 第 %d 轮开始,来料 %s ==', trial, colors)
            elapsed, parts = node.run_round(colors, args.timeout)
            if elapsed is None:
                rospy.logerr('第 %d 轮超时,终止后续轮次', trial)
                rows.append((trial, ','.join(colors), 'TIMEOUT', '', ''))
                break
            ok = sum(1 for p in parts if node.results[p].result == 'PLACED')
            fails = ';'.join('%s:%s' % (p, node.results[p].result) for p in parts
                             if node.results[p].result != 'PLACED')
            rows.append((trial, ','.join(colors), '%d/%d 完成' % (ok, len(parts)), '%.1f' % elapsed, fails))
            rospy.loginfo('第 %d 轮完成: %d/%d,用时 %.1fs %s', trial, ok, len(parts), elapsed, fails)
            rospy.sleep(2.0)
    except (RuntimeError, rospy.ServiceException) as e:
        rospy.logerr(str(e))
    finally:
        path = os.path.expanduser('~/.ros/a1_trials.csv')
        if not os.path.isdir(os.path.dirname(path)):
            os.makedirs(os.path.dirname(path))
        new_file = not os.path.exists(path)
        with open(path, 'a') as f:
            writer = csv.writer(f)
            if new_file:
                writer.writerow(['trial', 'colors', 'result', 'seconds', 'failures'])
            writer.writerows(rows)
        completed = sum(1 for r in rows if _all_done(r[2]))
        rospy.loginfo('== 测试结束: %d/%d 轮全部完成,明细: %s,已追加到 %s ==',
                      completed, len(rows), rows, path)


if __name__ == '__main__':
    main()
