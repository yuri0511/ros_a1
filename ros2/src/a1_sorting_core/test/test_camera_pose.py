"""逻辑相机真值换算与缓存(camera_pose)单元测试:不初始化 rclpy。

换算式与 attach_helper.GzKinematicAttach._on_logical 同款
(俯视安装 rpy=(0,90°,0)):world_x = cam_x + rel_z,world_y = cam_y + rel_y;
rel_x 为深度分量(有系统偏差,弃用)。

**布局 v2 终档(2026-10-05 G1 收口,Lead 定档)**:相机立柱
**(-0.10,-0.48)** 桌外落地、头高 2.10,图像相机 h_fov 1.2→**1.35**
(带缘三点走 480px 窄轴,h1.2 全灭;详见 SDF camera_pole 注与
G0_g0full.json pole_trials;2026-10-04 曾短暂定档 -0.05)。换算用例
按终档重推,并新增 **世界↔参数一致性闸**:实读
sorting_cell.sdf 的 overhead_camera_model <pose>,与 sorting_params.yaml
的 camera_xy / recorder camera.* 逐值对齐 —— 2026-10-03 "立柱挪位漏改
一段 ⇒ y 偏 0.32、取件去够空地" 的同型坑,静态拦住。
(代码默认值 (0.65,0.0) 属构造默认、派单明令不动,不在本文件口径内。)
"""
import re
from pathlib import Path

import pytest
import yaml

from a1_sorting_core.camera_pose import CameraPoseCache, world_xy_from_rel

# v2 终档相机安装(与 worlds/sorting_cell.sdf 的 overhead_camera_model 同值)
CAMERA_XY = (-0.10, -0.48)
CAMERA_Z = 2.10
# 图像相机 h_fov 终档(SDF overhead_camera sensor;逻辑相机仍 2.2 不动)
IMG_H_FOV = 1.35
LOGICAL_H_FOV = 2.2

_WS = Path(__file__).resolve().parents[3]          # ros2_ws 根
_SDF = _WS / 'src/a1_cell_sim/worlds/sorting_cell.sdf'
_PARAMS = _WS / 'src/a1_sorting_core/config/sorting_params.yaml'
_PIPELINE = _WS / 'src/a1_sorting_core/config/sorting_params_pipeline.yaml'


def test_world_xy_from_rel_overhead():
    # rel_z=-0.05 -> world_x=-0.10+(-0.05)=-0.15;rel_y=0.28 -> -0.48+0.28=-0.20
    assert world_xy_from_rel(CAMERA_XY, 1.2, 0.28, -0.05) == pytest.approx(
        (-0.15, -0.20))
    # rel_x(深度)不参与换算:取极端值结果不变
    assert world_xy_from_rel(CAMERA_XY, 9.9, 0.28, -0.05) == pytest.approx(
        (-0.15, -0.20))
    # 带尾停位 (0.60, 0.0)(2026-10-04 晚拉长带后的新停位 = 锚表
    # meta.belt_stop)对应的相对读数:rel_z=0.70、rel_y=0.48
    assert world_xy_from_rel(CAMERA_XY, 0.0, 0.48, 0.70) == pytest.approx(
        (0.60, 0.0))
    # 生成点 (0.18, 0.0) 仍在带首:rel_z=0.28
    assert world_xy_from_rel(CAMERA_XY, 0.0, 0.48, 0.28) == pytest.approx(
        (0.18, 0.0))


def test_cache_updates_only_part_models():
    cache = CameraPoseCache(CAMERA_XY)
    cache.update([
        ('part_1', 0.0, 0.28, 0.35),           # -> (0.25, -0.20)
        ('gripper_base', 0.0, 0.0, 0.0),       # 非工件模型,忽略
        ('bin_blue', 0.0, 0.0, 0.0),
        ('bin_green', 0.0, 0.0, 0.0),
        ('sorting_cell', 0.0, 0.0, 0.0),
    ])
    assert cache.get('part_1') == pytest.approx((0.25, -0.20))
    assert cache.get('gripper_base') is None
    assert cache.known_parts() == {'part_1'}


def test_cache_latest_observation_wins():
    """工件被搬动后,最新观测覆盖旧值(执行节点取件前刷新)。"""
    cache = CameraPoseCache(CAMERA_XY)
    cache.update([('part_1', 0.0, 0.48, 0.28)])        # 带尾生成位 (0.18,0)
    assert cache.get('part_1') == pytest.approx((0.18, 0.0))
    cache.update([('part_1', 0.0, 0.76, 0.40)])        # 北排蓝箱 (0.30,0.28)
    assert cache.get('part_1') == pytest.approx((0.30, 0.28))


def test_cache_missing_returns_none():
    """无缓存 -> None(执行节点据此抛可重试的 GraspError('no_camera_pose'))。"""
    cache = CameraPoseCache(CAMERA_XY)
    cache.update([('part_1', 0.0, 0.48, 0.28)])
    assert cache.get('part_2') is None


def test_cache_custom_camera_xy():
    """参数化本身:任何 camera_xy 都按同一换算式生效(此处用旧 v1 柱位)。"""
    cache = CameraPoseCache((0.65, -0.32))
    assert cache.camera_xy == (0.65, -0.32)
    cache.update([('part_2', 0.0, 0.52, -0.25)])
    assert cache.get('part_2') == pytest.approx((0.40, 0.20))


# ------------------------------------------------- 世界 ↔ 参数一致性闸(v2)
def _sdf_camera_pose() -> tuple[float, float, float]:
    """从 sorting_cell.sdf 实读 overhead_camera_model 的 <pose>。"""
    txt = _SDF.read_text(encoding='utf-8')
    m = re.search(r'<model name="overhead_camera_model">.*?<pose>'
                  r'([^<]+)</pose>', txt, re.S)
    assert m, 'sdf 里找不到 overhead_camera_model 的 pose'
    vals = [float(v) for v in m.group(1).split()]
    assert len(vals) == 6, vals
    # rpy 必须是 (0, 90°, 0) —— 换算式成立的前提
    assert abs(vals[3]) < 1e-6 and abs(vals[4] - 1.5707963) < 1e-5 \
        and abs(vals[5]) < 1e-6, vals
    return vals[0], vals[1], vals[2]


def test_world_image_camera_h_fov_final_135():
    """图像相机 h_fov 终档 1.35 + 逻辑相机保持 2.2(SDF 实读)。

    Lead 2026-10-05 定档:h1.2 六档柱位全灭(带缘三点走 480px 窄轴,
    半程=0.75·tan(h/2)·d),h1.35 @柱 -0.10 余量 +52.4mm。逻辑相机
    (识别真值源)口径未动 —— 误把图像相机当识别相机收窄会砍掉带缘。
    """
    txt = _SDF.read_text(encoding='utf-8')
    fovs = re.findall(r'<horizontal_fov>([\d.]+)</horizontal_fov>', txt)
    assert len(fovs) == 2, fovs                      # 恰好图像+逻辑两处
    # overhead_camera(图像)在 logical_camera 之前,顺序即文件顺序
    assert float(fovs[0]) == pytest.approx(IMG_H_FOV)
    assert float(fovs[1]) == pytest.approx(LOGICAL_H_FOV)


def test_world_camera_pose_matches_params():
    """sdf 相机 pose = 参数文件 camera_xy / recorder camera.*(缺一处即炸)。"""
    sx, sy, sz = _sdf_camera_pose()
    assert (sx, sy) == pytest.approx(CAMERA_XY)
    assert sz == pytest.approx(CAMERA_Z)
    doc = yaml.safe_load(_PARAMS.read_text(encoding='utf-8'))
    for section in ('/pick_place_executor', '/pick_place_relay'):
        p = doc[section]['ros__parameters']
        assert [float(v) for v in p['camera_xy']] == pytest.approx(
            [sx, sy]), section
    rec = doc['/result_recorder']['ros__parameters']
    assert (float(rec['camera.x']), float(rec['camera.y']),
            float(rec['camera.z'])) == pytest.approx((sx, sy, sz))
    # 覆盖文件里的 camera_xy(若写了)也必须同值(加载顺序 = 基表在前)
    pdoc = yaml.safe_load(_PIPELINE.read_text(encoding='utf-8'))
    for section in ('/pick_place_executor', '/pick_place_relay'):
        p = pdoc[section]['ros__parameters']
        if 'camera_xy' in p:
            assert [float(v) for v in p['camera_xy']] == pytest.approx(
                [sx, sy]), (_PIPELINE.name, section)


def test_world_bin_poses_match_recorder_bins():
    """sdf 四箱 <pose> 与 recorder bins.* 箱心逐值对齐(台账判定几何)。

    64cd236 教训的静态版:箱心只改世界不改参数(或反之)会让正确投放被记
    MISPLACED/OUTSIDE = 配置错配假信号。四箱必须齐、旧两箱不得残留模型。
    """
    txt = _SDF.read_text(encoding='utf-8')
    doc = yaml.safe_load(_PARAMS.read_text(encoding='utf-8'))
    rec = doc['/result_recorder']['ros__parameters']
    found = dict(re.findall(r'<model name="(bin_\w+)">\s*'
                            r'<static>true</static>\s*'
                            r'<pose>([^<]+)</pose>', txt))
    assert set(found) == {'bin_blue', 'bin_yellow', 'bin_red', 'bin_green'}, \
        sorted(found)
    for name, pose in found.items():
        x, y = [float(v) for v in pose.split()[:2]]
        assert (float(rec[f'bins.{name}.x']),
                float(rec[f'bins.{name}.y'])) == pytest.approx((x, y)), name
        # 内口半宽(判定口径)与执行器段一致,五键齐全
        for suffix in ('half_x', 'half_y', 'inner_half_x', 'inner_half_y'):
            assert f'bins.{name}.{suffix}' in rec, (name, suffix)
