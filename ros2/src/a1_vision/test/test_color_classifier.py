"""颜色分类纯逻辑单元测试。"""
import numpy as np
import pytest

from a1_vision.color_classifier import (classify_region, rpy_to_matrix,
                                        world_to_pixel)

H, W = 480, 640


def _image_with(color_at):
    """黑底图像,在给定像素中心画 20x20 色块。color_at: {px: bgr}"""
    img = np.zeros((H, W, 3), dtype=np.uint8)
    for (u, v), bgr in color_at.items():
        img[v - 10:v + 10, u - 10:u + 10] = bgr
    return img


def test_classify_red_and_green():
    # 纯红 BGR=(0,0,255),纯绿 BGR=(0,255,0)
    img = _image_with({(150, 120): (0, 0, 255), (450, 300): (0, 255, 0)})
    assert classify_region(img, (150, 120), 12) == 'red'
    assert classify_region(img, (450, 300), 12) == 'green'


def test_classify_none_on_dark():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    assert classify_region(img, (320, 240), 12) is None


def test_classify_empty_window():
    img = np.zeros((H, W, 3), dtype=np.uint8)
    assert classify_region(img, (2000, 2000), 12) is None  # 窗口完全出界


def test_world_to_pixel_top_down():
    """俯视相机投影:rpy=(0, pi/2, 0),相机 (0,0,2),工件 (0.1,-0.2,0)。

    手算:深度 2;u 偏移 = fx * (0.2)/2,v 偏移 = fx * (-0.1)/2。
    """
    rot = rpy_to_matrix(0.0, np.pi / 2, 0.0)
    intrinsics = (300.0, 300.0, 320.0, 240.0)
    u, v = world_to_pixel((0.1, -0.2, 0.0), (0.0, 0.0, 2.0), rot, intrinsics)
    assert u == pytest.approx(320.0 + 300.0 * 0.2 / 2)
    assert v == pytest.approx(240.0 - 300.0 * 0.1 / 2)


def test_world_to_pixel_behind_camera():
    rot = rpy_to_matrix(0.0, np.pi / 2, 0.0)
    with pytest.raises(ValueError):
        world_to_pixel((0.0, 0.0, 3.0), (0.0, 0.0, 2.0), rot, (300, 300, 320, 240))
