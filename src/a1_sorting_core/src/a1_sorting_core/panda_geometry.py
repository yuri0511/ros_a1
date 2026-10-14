"""Panda 竖直向下抓取的几何计算(纯逻辑,ROS1 移植新增,可单测)。

对应组长 executor 里的 _down_pose / tcp_z_offset / pre_grasp_height / bin_grid_offsets:
组长目标是 UR5e 的 tool0,这里是 Panda 的 panda_link8。
"""
from __future__ import annotations

import math

# panda_link8 竖直向下、手指沿世界 x 方向张合的姿态(panda_moveit_config 'ready' 位姿同族)
DOWN_RPY = (math.pi, 0.0, -math.pi / 4)


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    """与 tf.transformations.quaternion_from_euler(r, p, y) / tf2 setRPY 相同,返回 (x, y, z, w)。"""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


DOWN_Q = quat_from_rpy(*DOWN_RPY)


def down_pose(center_xyz, tcp_z_offset: float, height: float = 0.0):
    """工件中心 -> panda_link8 目标位置(竖直向下,高出 height)。返回 ((x,y,z), q)。"""
    x, y, z = center_xyz
    return (x, y, z + tcp_z_offset + height), DOWN_Q


def grid_slot(bin_xy, index: int, offsets_x, offsets_y):
    """料区内第 index 个格位(循环使用),与组长 bin_grid_offsets 同语义。"""
    n = min(len(offsets_x), len(offsets_y))
    if n == 0:
        return float(bin_xy[0]), float(bin_xy[1])
    i = index % n
    return float(bin_xy[0]) + offsets_x[i], float(bin_xy[1]) + offsets_y[i]


def parse_colors(text: str):
    """'red, green,red' -> ['red', 'green', 'red'](去空格、去空项)。"""
    return [c.strip() for c in str(text).split(',') if c.strip()]


# MoveIt 1.1.16 的 compute_cartesian_path 在 fraction<1 时仍返回部分轨迹。
# 低于这个比例就不用那段轨迹，改走普通规划。
CARTESIAN_MIN_FRACTION = 0.999


def cartesian_reached(fraction, minimum=CARTESIAN_MIN_FRACTION):
    """直线规划是否覆盖到目标。fraction 不是数字时视为没到。"""
    try:
        value = float(fraction)
    except (TypeError, ValueError):
        return False
    return value + 1e-9 >= float(minimum)
