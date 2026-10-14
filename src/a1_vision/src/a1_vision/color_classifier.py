"""颜色分类纯逻辑(与 ROS 解耦,便于单元测试)。

两部分:
1. HSV 分割分类:在图像指定像素窗口内,判定占比最高的颜色类别;
2. 世界坐标 -> 像素投影:俯视相机的针孔投影(用于定位采样窗口)。

简化声明:工件位置取自仿真真值并投影到图像,视觉只负责"看颜色"定类别;
颜色到类别的判定是真实感知,位姿未做像素级估计(README 有说明)。
"""
from __future__ import annotations  # ROS1 移植: 兼容 Python 3.8
import cv2
import numpy as np

# 默认 HSV 阈值(OpenCV H:0-179, S/V:0-255)。
# 红色横跨 0°/180° 两侧,需要两个区间。
# yellow 区间 18-32:H=24.6(生成色 0.95/0.80/0.10 实测换算),与 red
# (0-10/170-179)、green(35-85)双向留隙(2026-10-03 加黄时核定)。
DEFAULT_RANGES = {
    'red':   [((0, 90, 60), (10, 255, 255)),
              ((170, 90, 60), (179, 255, 255))],
    'green': [((35, 90, 60), (85, 255, 255))],
    'blue':  [((95, 90, 60), (135, 255, 255))],
    'yellow': [((18, 90, 60), (32, 255, 255))],
}


def classify_region(image_bgr, center_px, radius: int,
                    ranges=None, min_ratio: float = 0.25):
    """在 (center_px, radius) 正方形窗口内做 HSV 分割分类。

    返回占比最高且不低于 min_ratio 的类别名;都不达标返回 None。
    """
    if ranges is None:
        ranges = DEFAULT_RANGES
    h, w = image_bgr.shape[:2]
    cx, cy = int(center_px[0]), int(center_px[1])
    x0, x1 = max(0, cx - radius), min(w, cx + radius)
    y0, y1 = max(0, cy - radius), min(h, cy + radius)
    if x1 <= x0 or y1 <= y0:
        return None
    crop = image_bgr[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).astype(np.int16)

    best_cat, best_ratio = None, 0.0
    for category, spans in ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=bool)
        for lo, hi in spans:
            mask |= ((hsv[..., 0] >= lo[0]) & (hsv[..., 0] <= hi[0])
                     & (hsv[..., 1] >= lo[1]) & (hsv[..., 1] <= hi[1])
                     & (hsv[..., 2] >= lo[2]) & (hsv[..., 2] <= hi[2]))
        ratio = float(mask.mean())
        if ratio >= min_ratio and ratio > best_ratio:
            best_cat, best_ratio = category, ratio
    return best_cat


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    """ZYX 欧拉角 -> 旋转矩阵(传感器系 -> 世界系)。"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def world_to_pixel(point_world, cam_pos, cam_rot, intrinsics) -> tuple:
    """世界坐标 -> 像素坐标(俯视针孔投影)。

    坐标约定(Gazebo 相机传感器系):
      传感器 +X 为视线方向,rpy=(0, pi/2, 0) 时视线竖直向下;
      图像 u 方向沿 -Y_s,v 方向沿 -Z_s(与 Gazebo 渲染一致)。
    intrinsics: (fx, fy, cx, cy)。
    """
    fx, fy, cx, cy = intrinsics
    p = np.asarray(point_world, dtype=float) - np.asarray(cam_pos, dtype=float)
    depth = float(p @ cam_rot[:, 0])            # 沿视线的深度
    if depth <= 1e-6:
        raise ValueError('目标在相机后方,无法投影')
    u = cx + fx * float(p @ (-cam_rot[:, 1])) / depth
    v = cy + fy * float(p @ (-cam_rot[:, 2])) / depth
    return (u, v)
