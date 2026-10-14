"""ROS1 移植新增:Panda 抓取几何单测(不需要 ROS)。"""
import math

from a1_sorting_core.panda_geometry import (DOWN_Q, cartesian_reached,
                                            down_pose, grid_slot,
                                            parse_colors, quat_from_rpy)


def test_down_pose_height():
    (x, y, z), q = down_pose((0.45, -0.15, 0.025), 0.105, 0.10)
    assert (x, y) == (0.45, -0.15)
    assert abs(z - 0.23) < 1e-9
    assert q == DOWN_Q


def test_down_quat_points_down():
    # 旋转后 z 轴应指向 -z:R[:,2] 的 z 分量 = 1 - 2(x^2 + y^2)
    x, y, z, w = DOWN_Q
    assert abs((1 - 2 * (x * x + y * y)) + 1.0) < 1e-9


def test_quat_identity():
    assert quat_from_rpy(0, 0, 0) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = quat_from_rpy(0, 0, math.pi / 2)
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9


def test_grid_slot_cycles():
    ox, oy = [-0.05, 0.05, -0.05, 0.05], [-0.035, -0.035, 0.035, 0.035]
    sx, sy = grid_slot((0.35, 0.35), 0, ox, oy)
    assert abs(sx - 0.30) < 1e-9 and abs(sy - 0.315) < 1e-9
    assert grid_slot((0.35, 0.35), 5, ox, oy) == grid_slot((0.35, 0.35), 1, ox, oy)
    assert grid_slot((0.1, 0.2), 3, [], []) == (0.1, 0.2)


def test_parse_colors():
    assert parse_colors(' red, green,,red ') == ['red', 'green', 'red']


def test_cartesian_fraction_must_cover_the_goal():
    assert cartesian_reached(1.0) is True
    assert cartesian_reached(0.999) is True
    assert cartesian_reached(0.9) is False
    assert cartesian_reached(None) is False
