#!/usr/bin/env python3
"""重复运行测试脚本:自动跑 N 轮分拣并统计成功率(支撑"测试与分析")。

用法(系统已在仿真中运行时):
    ros2 run a1_sorting_core run_trials.py --trials 3
    ros2 run a1_sorting_core run_trials.py --trials 2 --colors "red,green,red" "blue,green,red"

每轮流程:清除旧工件 -> 按颜色序列重新生成 3 个工件 -> 等待视觉登记 ->
等待 3 件任务全部终态 -> 记录本轮计数。结束后输出成功率汇总并写
~/.ros/a1_trials.csv。
"""
import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from a1_interfaces.msg import PartInfo, TaskStatus
from a1_interfaces.srv import SortTask
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

# 与 a1_cell_sim/scripts/spawn_parts.py 保持一致
FEED_SLOTS = [(0.45, -0.15), (0.45, 0.0), (0.45, 0.15)]
FEED_Z = 0.745
PART_SIZE = 0.05
PART_MASS = 0.2
COLOR_RGB = {'red': (0.85, 0.10, 0.10), 'green': (0.10, 0.75, 0.15),
             'blue': (0.15, 0.35, 0.90)}
PARTS = ['part_1', 'part_2', 'part_3']
MODEL_ENTITY_TYPE = 2  # gz.msgs.Entity.Type MODEL


def part_sdf(index: int, color: str) -> str:
    r, g, b = COLOR_RGB[color]
    inertia = PART_MASS / 12 * (PART_SIZE ** 2 + PART_SIZE ** 2)
    # 生成位置由 EntityFactory.pose 给出(gz-sim 忽略 SDF 内嵌 pose)
    return f"""<?xml version="1.0" ?>
<sdf version="1.11">
  <model name="part_{index}">
    <link name="link">
      <inertial><mass>{PART_MASS}</mass>
        <inertia><ixx>{inertia}</ixx><iyy>{inertia}</iyy><izz>{inertia}</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>{PART_SIZE} {PART_SIZE} {PART_SIZE}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{PART_SIZE} {PART_SIZE} {PART_SIZE}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>"""


class TrialRunner(Node):

    def __init__(self, world: str):
        super().__init__('run_trials')
        self._create = self.create_client(SpawnEntity, f'/world/{world}/create')
        self._remove = self.create_client(DeleteEntity, f'/world/{world}/remove')
        self._sort = self.create_client(SortTask, '/sort_task')
        self.create_subscription(TaskStatus, '/task/status', self._on_status, 10)
        self.create_subscription(PartInfo, '/parts/arrival', self._on_arrival, 10)
        self.terminal: set[str] = set()
        self.arrived: set[str] = set()

    def _wait_service(self, client, name):
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'服务不可用: {name}')

    def _on_status(self, msg: TaskStatus):
        if msg.state in ('COMPLETED', 'FAILED'):
            self.terminal.add(msg.part_id)

    def _on_arrival(self, msg: PartInfo):
        self.arrived.add(msg.part_id)

    def clear_parts(self):
        self._wait_service(self._remove, self._remove.srv_name)
        for part_id in PARTS:
            req = DeleteEntity.Request()
            req.entity.name = part_id
            req.entity.type = MODEL_ENTITY_TYPE
            future = self._remove.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        time.sleep(1.0)  # 等物理场景清理完成

    def spawn_parts(self, colors):
        self._wait_service(self._create, self._create.srv_name)
        for i, color in enumerate(colors, start=1):
            req = SpawnEntity.Request()
            req.entity_factory.name = f'part_{i}'
            req.entity_factory.sdf = part_sdf(i, color)
            slot = FEED_SLOTS[i - 1]
            req.entity_factory.pose.position.x = slot[0]
            req.entity_factory.pose.position.y = slot[1]
            req.entity_factory.pose.position.z = FEED_Z
            future = self._create.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is None or not future.result().success:
                raise RuntimeError(f'生成 part_{i} 失败')
            time.sleep(0.3)

    def start_batch(self):
        self._wait_service(self._sort, '/sort_task')
        future = self._sort.call_async(SortTask.Request(command='start_batch'))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def wait_round(self, timeout_sec: float = 240.0):
        """等待 3 件全部来料登记并全部到达终态。"""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if PARTS <= self.arrived and PARTS <= self.terminal:
                return True
        return False

    def reset_round_state(self):
        self.terminal.clear()
        self.arrived.clear()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--world', default='sorting_cell')
    parser.add_argument('--colors', nargs='*', default=['red,green,red'],
                        help='每轮的来料颜色序列(逗号分隔),轮数不足时循环使用')
    parser.add_argument('--timeout', type=float, default=240.0)
    args = parser.parse_args()

    color_seqs = [c.split(',') for c in args.colors]
    rclpy.init()
    node = TrialRunner(args.world)
    rows = []
    try:
        for trial in range(1, args.trials + 1):
            colors = color_seqs[(trial - 1) % len(color_seqs)]
            node.reset_round_state()
            node.get_logger().info(f'== 第 {trial} 轮开始,来料 {colors} ==')
            node.clear_parts()
            node.spawn_parts(colors)
            ok = node.wait_round(args.timeout)
            if not ok:
                node.get_logger().error(f'第 {trial} 轮超时,终止后续轮次')
                rows.append((trial, ','.join(colors), 'TIMEOUT'))
                break
            time.sleep(1.0)
            rows.append((trial, ','.join(colors),
                         f'{len(node.terminal)}/3 终态'))
            node.get_logger().info(f'第 {trial} 轮完成: {sorted(node.terminal)}')
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        path = Path('~/.ros/a1_trials.csv').expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not path.exists()
        with open(path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if new_file:
                writer.writerow(['trial', 'colors', 'result'])
            writer.writerows(rows)
        completed = sum(1 for r in rows if '3/3' in r[2])
        node.get_logger().info(
            f'== 测试结束: {completed}/{len(rows)} 轮全部完成,明细: {rows},'
            f'已追加到 {path} ==')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
