#!/usr/bin/env python3
"""Deliberate contact check for the arm mesh sensors.

Spawns one temporary static box on the shoulder mesh, then dips the upper
arm, forearm, and wrist into the existing table collision. Removes the box
before the dip. Does not change the sorting world file.
"""
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from trajectory_msgs.msg import JointTrajectoryPoint

CSV = Path(sys.argv[1] if len(sys.argv) > 1 else 'contacts_mesh_probe.csv')
RESULT = Path('/tmp/mesh_probe_result.txt')
BOX = 'shoulder_probe_box'
SDF = """<?xml version="1.0"?>
<sdf version="1.11">
  <model name="shoulder_probe_box">
    <static>true</static>
    <pose>0.600 0.410 0.602 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>0.04 0.04 0.04</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
MESH_NEEDLES = (
    'shoulder_link_collision',
    'upper_arm_link_collision',
    'forearm_link_collision',
    'wrist_1_link_collision',
    'wrist_2_link_collision',
)


def seen(needle: str) -> bool:
    if not CSV.exists():
        return False
    text = CSV.read_text(errors='replace')
    return needle in text


class Probe(Node):
    def __init__(self) -> None:
        super().__init__('mesh_probe')
        self.spawn = self.create_client(SpawnEntity, '/world/sorting_cell/create')
        self.remove = self.create_client(DeleteEntity, '/world/sorting_cell/remove')
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

    def wait_ready(self, client, label: str, seconds: float) -> bool:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'{label} ready')
                return True
        self.get_logger().error(f'{label} not ready')
        return False

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        return future.result()


def wait_rows(needles, seconds: float) -> set:
    deadline = time.monotonic() + seconds
    found = set()
    while time.monotonic() < deadline:
        found = {name for name in needles if seen(name)}
        if found == set(needles):
            return found
        time.sleep(2.0)
    return found


def main() -> int:
    rclpy.init()
    node = Probe()
    missing = []
    try:
        if not node.wait_ready(node.spawn, 'create', 600.0):
            missing.append('create_service')
            raise SystemExit(2)
        if node.wait_ready(node.remove, 'remove', 30.0):
            stale = DeleteEntity.Request()
            stale.entity.name = BOX
            stale.entity.type = Entity.MODEL
            node.call(node.remove, stale)
        request = SpawnEntity.Request()
        request.entity_factory.name = BOX
        request.entity_factory.sdf = SDF
        # The bridge pose replaces the pose inside the SDF. Leave it at
        # zero and the box sits on the world origin, missing the shoulder.
        request.entity_factory.pose.position.x = 0.600
        request.entity_factory.pose.position.y = 0.410
        request.entity_factory.pose.position.z = 0.602
        request.entity_factory.pose.orientation.w = 1.0
        spawned = False
        for _ in range(30):
            result = node.call(node.spawn, request)
            if result is not None and result.success:
                spawned = True
                node.get_logger().info('shoulder probe box spawned')
                break
            time.sleep(2.0)
        if not spawned:
            missing.append('spawn_box')
            raise SystemExit(2)
        found = wait_rows(('shoulder_link_collision',), 180.0)
        if 'shoulder_link_collision' not in found:
            missing.append('shoulder_link_collision')
        if node.wait_ready(node.remove, 'remove', 30.0):
            entity = Entity()
            entity.name = BOX
            entity.type = Entity.MODEL
            delete = DeleteEntity.Request()
            delete.entity = entity
            node.call(node.remove, delete)
            node.get_logger().info('shoulder probe box removed')
        if not node.arm.wait_for_server(timeout_sec=180.0):
            missing.append('joint_trajectory_controller')
            raise SystemExit(2)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        start = JointTrajectoryPoint()
        start.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        start.time_from_start = Duration(sec=1)
        end = JointTrajectoryPoint()
        # Vertices of these four meshes sit inside the table brick here.
        end.positions = [0.0, 1.125, 0.0, 0.0, 0.0, 0.0]
        end.time_from_start = Duration(sec=10)
        goal.trajectory.points = [start, end]
        send = node.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send, timeout_sec=30.0)
        accepted = send.result()
        if accepted is None or not accepted.accepted:
            missing.append('trajectory_rejected')
            raise SystemExit(2)
        node.get_logger().info('dip trajectory accepted')
        found = wait_rows((
            'upper_arm_link_collision',
            'forearm_link_collision',
            'wrist_1_link_collision',
            'wrist_2_link_collision',
        ), 240.0)
        for name in (
            'upper_arm_link_collision',
            'forearm_link_collision',
            'wrist_1_link_collision',
            'wrist_2_link_collision',
        ):
            if name not in found:
                missing.append(name)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if missing:
        RESULT.write_text('MISSING ' + ','.join(missing) + '\n')
        print('PROBE_MISSING ' + ','.join(missing), flush=True)
        return 2
    RESULT.write_text('OK\n')
    print('PROBE_OK', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
