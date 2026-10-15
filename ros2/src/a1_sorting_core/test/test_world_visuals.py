"""世界 visual 互穿闸(2026-10-09 穿模修复回归)。

定罪(截图 + AABB 全扫):table.slab_top(深色板 visual 顶 0.705)与
tabletop.top(浅色板 0.70-0.72)互穿 5mm + 西缘 100mm 台阶 = 桌面
"补丁感"元凶。修法:浅色大面 visual 删除(四块碰撞补丁一字未动)、
深色板顶抬到 0.72 共面。滑轨曾收窄 68mm,用户拍板回退 124mm 原长
(见 test_gripper_rail_original_length_locked)。

本闸钉死不变式:
- 跨模型 visual×visual 不得互穿(共面/贴合允许;belt 滚纹为设计内凸出,
  与带体同模型不属跨模型对);
- 桌面唯一大面 = table.slab_top,顶面恰 0.72(承托面/带底/盒底共面);
- tabletop 只剩四块碰撞补丁,无大面 visual。
collision 逐字节由 tools/check_collision_locked.py + 锚表把关,不在本闸。
"""
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

WORLD = Path(__file__).resolve().parents[2] / 'a1_cell_sim' / 'worlds' / 'sorting_cell.sdf'
GRIPPER = Path(__file__).resolve().parents[2] / 'a1_cell_sim' / 'urdf' / 'gripper.urdf.xacro'


def _pose_of(el):
    p = el.find('pose')
    v = [float(x) for x in p.text.split()] if p is not None and p.text else []
    while len(v) < 6:
        v.append(0.0)
    return v


def _box_items(path):
    """model 列表:(model, kind, name, (minx,maxx,miny,maxy,minz,maxz))"""
    root = ET.parse(path).getroot().find('world')
    out = []
    for m in root.findall('model'):
        mn = m.get('name')
        mp = _pose_of(m)
        for link in m.findall('link'):
            for kind in ('visual', 'collision'):
                for v in link.findall(kind):
                    g = v.find('geometry')
                    b = g.find('box') if g is not None else None
                    if b is None or b.find('size') is None:
                        continue
                    size = [float(x) for x in b.find('size').text.split()]
                    vp = _pose_of(v)
                    cx, cy, cz = mp[0] + vp[0], mp[1] + vp[1], mp[2] + vp[2]
                    out.append((mn, kind, v.get('name') or '',
                                (cx - size[0] / 2, cx + size[0] / 2,
                                 cy - size[1] / 2, cy + size[1] / 2,
                                 cz - size[2] / 2, cz + size[2] / 2)))
    return out


def test_no_cross_model_visual_overlap():
    items = _box_items(WORLD)
    bad = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            A, B = items[i], items[j]
            if A[0] == B[0]:
                continue                      # 同模型互穿不属本闸(夹爪自洽)
            if A[1] != 'visual' or B[1] != 'visual':
                # 只闸渲染件互穿;碰撞体不可见(补丁埋进桌板正是要它
                # 隐形承托),由锚表/物理定档管
                continue
            # 设计内:belt_side_rails 滚纹(rib)凸出带面 5mm 且横向探入
            # 带体(骑跨造型);collision 版滚纹不存在,仅 visual×visual
            pair = {A[0], B[0]}
            if pair == {'belt', 'belt_side_rails'} \
                    and ('rib' in A[2] or 'rib' in B[2]):
                continue
            bb = [min(A[3][1 + 2 * k], B[3][1 + 2 * k])
                  - max(A[3][2 * k], B[3][2 * k]) for k in range(3)]
            if all(o > 0.0015 for o in bb):
                bad.append((A[0], A[2], B[0], B[2], [round(o * 1000) for o in bb]))
    assert not bad, f'跨模型 visual 互穿: {bad}'


def test_table_surface_coplanar_and_single():
    items = _box_items(WORLD)
    slab = [it for it in items if it[0] == 'table' and it[2] == 'slab_top']
    assert len(slab) == 1
    (_, _, _, (minx, maxx, miny, maxy, minz, maxz)) = slab[0]
    assert abs(maxz - 0.72) < 1e-6, f'slab_top 顶面应为 0.72,实为 {maxz}'
    # tabletop 大面 visual 必须已删除,只剩 4 块碰撞补丁
    tt = [it for it in items if it[0] == 'tabletop']
    assert all(k == 'collision' for _, k, _, _ in tt), 'tabletop 不得再有 visual'
    assert len(tt) == 4
    for it in tt:
        assert abs(it[3][5] - 0.72) < 1e-6    # 补丁顶共面(承托面不变)


def test_gripper_rail_original_length_locked():
    """滑轨 124mm 原长**定档**(用户拍板 2026-10-09 上午)。

    凌晨版曾收窄到 68mm 消"下探瞬间与盒壁 ~27mm 视觉重叠",用户看后
    否决:横杠短于指距,变成悬空小断条,观感更差——回退原长。瞬时重叠
    仅 settle 触底一两秒、纯 visual 无碰撞,可接受。本闸钉死 124mm,
    防止后续批次再"好心"收窄。"""
    s = GRIPPER.read_text()
    import re
    m = re.search(r'<visual name="rail">.*?<geometry><box size="([\d. ]+)"/>',
                  s, re.S)
    assert m, '滑轨 visual 块未找到'
    assert m.group(1).split()[1] == '0.124', \
        f'滑轨宽必须保持 124mm 定档(用户拍板),实为 {m.group(1)}'
