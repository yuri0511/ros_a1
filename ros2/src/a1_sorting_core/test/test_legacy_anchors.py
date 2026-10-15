"""放置锚定参数回归(纯 Python:yaml.safe_load,不依赖 ROS/rclpy)。

**布局 v2 口径(2026-10-04 G1)**:校验两份参数文件
  - src/a1_sorting_core/config/sorting_params.yaml       (legacy 单臂基表)
  - src/a1_sorting_core/config/sorting_params_pipeline.yaml(pipeline 覆盖表)

覆盖(键名/结构校验全部保留,判据阈值一字未放宽):
(a) 6 组 carry_anchor xy/joints 与 12 组 carry_settle_top/bottom{N}_joints
    均存在、长度正确(xy=2/关节=6)、无 NaN 且非 1e9 哨兵 —— executor
    装载时 xy[0]>=1e8 或关节含 NaN 的槽位会被静默跳过
    (pick_place_executor_node.__init__ 的 _carry_anchors/_settle_pairs);
(b) 全部 18 组关节角属弯肘族:lift(j1)∈(-2.8,-0.2) 且
    |wrist_2(j4)+π/2|<0.35 —— 近直肘族在满伸展下 P 控制举不动
    (pipeline 联调实证)。**v2 实测族**:G0 定档锚 lift ∈ -1.70~-2.06、
    wrist_2 全 = -1.5708,原包络直接通过,故包络常数不改(不为变绿而放宽);
(c) settle 槽位与 anchor 槽位一一对应:executor 运行时按"放置格 xy
    就近选锚"选 carry_anchor,再以该锚 xy(三位舍入)为键查
    _settle_pairs —— 本测试静态复刻该选择:每个网格格的最近锚槽位号
    必须与该格所属箱的 settle 槽位段一致(北/南排第一箱格->1..3,
    第二箱格->4..6),且 6 个锚 xy 三位舍入后互异(否则 _settle_pairs
    字典键冲突会静默丢掉一对);
(d) **四色四箱**:executor/relay 段箱心与 /result_recorder 段一致,
    每箱五键齐全(x/y/half_x/half_y/inner_half_x/inner_half_y —— 64cd236
    漏 x/y ⇒ recorder 把料区判到世界原点的假 MISPLACED 教训),
    place_joints_<箱> 保持显式全 NaN(节点内置默认值非 NaN,
    carry_place_joints 分支优先于 carry_anchor,不覆盖会把锚定架空);
(e) camera_xy / recorder camera 三处 v2 终档 (-0.10,-0.48,z 2.10);
(f) 基表与 pipeline 表的 arm1 锚值逐值一致(legacy 与 pipeline 同臂同族,
    差异只允许出现在 tcp_z_offset / carry_safe_z 等有注明的模式项)。

不校验 settle 梯子的运行时启用:executor 的 set-down 分支还要求
place_drop_clearance>0(两份 yaml 均 0.09,已在本文件断言)。

✅ 锚值接线状态(2026-10-05 01:0x 解冻完成):本文件钉值与两份 yaml
   均以终版权威源 **docs/evidence/G0_anchors_v3.json**(5 位发布)为准,
   (g) 项 test_wired_anchors_match_G0_anchors_v3_json 把 4 段×75 组锚与
   JSON 逐值核对 ⇒ 锚表再重发即刻抓出漂移,不靠肉眼)。20:2x 那次"框架级 bug /
   偏 561mm"已被 Lead 00:5x 撤案(根因 = builder 改 xacro 中间态的双重变换
   瞬态,非锚值错)。
   仍有效的结构面:键名/槽位对应/弯肘族包络常数(新表 lift 最深 -2.0644,
   wrist_2 全 = -1.5708,原包络直接通过,**未放宽**)/四箱五键/camera/belt
   几何闸。待 Lead 单发的两项(arm2 standby 新值、camera_xy 终值)到位后,
   只需改 yaml + STANDBY_ARM2/CAMERA_XY 常量,本文件逻辑不动。
   另:xacro 的 initial_positions、tools/e0_probe_v2.py 的 PROBE_ANCHORS
   与两 yaml 同源,已一并同步(e0 探针保留双源互验机制)。
   """
import json
import math
import re
from pathlib import Path

import yaml

# ../../config/…(相对本测试文件)
_CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
_PKG_ROOT = Path(__file__).resolve().parents[3]   # ros2_ws 根
_LEGACY_FILE = _CONFIG_DIR / 'sorting_params.yaml'
_PIPELINE_FILE = _CONFIG_DIR / 'sorting_params_pipeline.yaml'
ANCHOR_SLOTS = range(1, 7)          # executor 槽位 1-6
LIFT_LO, LIFT_HI = -2.8, -0.2       # 弯肘族 lift(shoulder_lift)包络
W2_TOL = 0.35                       # |wrist_2 + π/2| 容差
SENTINEL = 1e8                      # executor 的"槽位禁用"哨兵阈值

# ---- 布局 v2 定档值(docs/layout_v2_spec.md + docs/notes_kinematics_v2.md)----
CAMERA_XY = [-0.10, -0.48]          # 相机立柱 v2 **终档**(2026-10-05 G1
#   收口:Lead 定档 -0.10 + 图像相机 h_fov 1.35;2026-10-04 曾定档 -0.05)
CAMERA_Z = 2.10                     # 相机头高(杆落地跨 0..2.10)
# 带尾取件目标(2026-10-04 晚用户拍板拉长带 = 锚表 meta.belt_stop)
BELT_STOP_XY = [0.60, 0.0]
# 终版锚表(唯一数据源):单测直接读它做逐值交叉核对,锚表再重发即自动抓漂移
_ANCHOR_JSON = Path(__file__).resolve().parents[3] / \
    'docs' / 'evidence' / 'G0_anchors_v3.json'
SLOT_COLOR = {'arm1': ['blue1', 'blue2', 'blue3', 'yellow1', 'yellow2',
                       'yellow3'],
              'arm2': ['red1', 'red2', 'red3', 'green1', 'green2', 'green3']}
# 四箱箱心(与 sorting_cell.sdf 的 <pose> 逐字一致)
BIN_XY = {'bin_blue': (0.30, 0.28), 'bin_yellow': (0.90, 0.28),
          'bin_red': (0.30, -0.28), 'bin_green': (0.90, -0.28)}
BIN_Z = 0.745                       # 箱内静置中心(内底 0.72 + 半边 0.025)
INNER_HALF = (0.06, 0.085)          # 内口 120(x)×170(y)mm 的半宽
NORTH = ('bin_blue', 'bin_yellow')  # arm1 北排
SOUTH = ('bin_red', 'bin_green')    # arm2 南排
GRID_X = [-0.03, 0.0, -0.03]        # 箱内 3 格对角布局(与 6 锚一一对应)
GRID_Y = [-0.05, 0.0, 0.05]
# G0 定档 standby(notes §1,两臂**不对称**)
STANDBY_ARM1 = [1.2593, -1.6371, 1.1284, -1.0621, -1.5708, -1.8822]
STANDBY_ARM2 = [1.1104, -1.8345, 1.0582, -0.7945, -1.5708, 1.1104]


def _load(path: Path) -> dict:
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _params(section: str = '/pick_place_executor',
            file: Path = _LEGACY_FILE) -> dict:
    return _load(file)[section]['ros__parameters']


def _num(v) -> float:
    """标量取数:容忍 str。哨兵 1e9 在 YAML 1.1(PyYAML)不带小数点/符号
    指数时解析成字符串,而运行时 rcl_yaml_param_parser(C)当 double —— 两种
    读法都必须能过(HEAD 起 yaml 里就写 [1e9, 1e9] 且实测可跑)。"""
    return float(v)


def _finite(v) -> bool:
    try:
        return math.isfinite(_num(v))
    except (TypeError, ValueError):
        return False


def _joint_vectors(p: dict) -> list[tuple[str, list]]:
    """(名字, 关节角) 对:6 锚 + 12 settle,共 18 组。"""
    out = []
    for slot in ANCHOR_SLOTS:
        out.append((f'anchor{slot}', p[f'carry_anchor{slot}_joints']))
        for end in ('top', 'bottom'):
            out.append((f'settle_{end}{slot}',
                        p[f'carry_settle_{end}{slot}_joints']))
    return out


# 两份文件 × 两臂段的参数表清单(legacy 基表 relay 段 = 旧接力历史依据,
# 其锚值不属 v2 定档,只校验 v2 同步项 camera_xy/standby_joints)
_SECTION_TARGETS = [
    ('legacy/pick_place_executor(arm1 北排)',
     _LEGACY_FILE, '/pick_place_executor', NORTH),
    ('legacy/pick_place_relay(arm2 南排)',
     _LEGACY_FILE, '/pick_place_relay', SOUTH),
    ('pipeline/pick_place_executor(arm1 北排)',
     _PIPELINE_FILE, '/pick_place_executor', NORTH),
    ('pipeline/pick_place_relay(arm2 南排)',
     _PIPELINE_FILE, '/pick_place_relay', SOUTH),
]


# ------------------------------------------------ (a) 完整性 / 无 NaN
def test_anchor_and_settle_slots_complete_and_finite():
    for label, file, section, _bins in _SECTION_TARGETS:
        p = _params(section, file)
        for slot in ANCHOR_SLOTS:
            xy = p[f'carry_anchor{slot}_xy']
            assert isinstance(xy, list) and len(xy) == 2, (label, slot)
            assert all(_finite(v) for v in xy), (label, slot)
            assert _num(xy[0]) < SENTINEL, \
                f'{label} anchor{slot} xy 为 1e9 哨兵(槽位禁用)'
            joints = p[f'carry_anchor{slot}_joints']
            assert isinstance(joints, list) and len(joints) == 6, (label, slot)
            assert all(_finite(v) for v in joints), \
                f'{label} anchor{slot} 关节含 NaN/缺省(executor 会静默跳过该槽)'
            for end in ('top', 'bottom'):
                settle = p[f'carry_settle_{end}{slot}_joints']
                assert isinstance(settle, list) and len(settle) == 6, (label, end, slot)
                assert all(_finite(v) for v in settle), (label, end, slot)
        assert _finite(p['settle_touch_z']), label


# ------------------------------------------------ (b) 弯肘族包络
def test_all_anchor_and_settle_joints_in_bent_elbow_family():
    """v2 新锚仍在弯肘族包络内(G0 notes 实测 lift -1.70~-2.06、
    wrist_2 = -1.5708)。包络常数按原判据取严,未放宽。"""
    for label, file, section, _bins in _SECTION_TARGETS:
        for name, joints in _joint_vectors(_params(section, file)):
            assert len(joints) == 6, (label, name)
            lift, w2 = float(joints[1]), float(joints[4])
            assert LIFT_LO < lift < LIFT_HI, \
                (label, name, 'lift 超出弯肘族包络', lift)
            assert abs(w2 + math.pi / 2) < W2_TOL, \
                (label, name, 'w2 偏离 -π/2', w2)


# ------------------------------------------------ (c) settle/anchor 槽位对应
def test_settle_slots_pair_with_anchor_slots_via_nearest_cell():
    """静态复刻 executor 选锚(min 平方距离):每格最近锚的槽位号必须落在
    该箱的 settle 槽位段;锚 xy 三位舍入互异(_settle_pairs 键)。"""
    for label, file, section, bins in _SECTION_TARGETS:
        p = _params(section, file)
        anchors = {slot: (_num(p[f'carry_anchor{slot}_xy'][0]),
                          _num(p[f'carry_anchor{slot}_xy'][1]))
                   for slot in ANCHOR_SLOTS}
        keys = [(round(x, 3), round(y, 3)) for x, y in anchors.values()]
        assert len(set(keys)) == 6, \
            f'{label} 锚 xy 三位舍入后冲突,_settle_pairs 会静默丢对: {keys}'
        gx = [_num(v) for v in p['bin_grid_offsets.x']]
        gy = [_num(v) for v in p['bin_grid_offsets.y']]
        assert (gx, gy) == (GRID_X, GRID_Y), f'{label} 网格偏移与 G0 定档不一致'
        assert len(gx) == len(gy) == 3, f'{label} 箱内 3 格网格(与 6 锚一一对应)'
        for first_slot, bin_name in zip((1, 4), bins):
            bx, by = _num(p[f'{bin_name}.x']), _num(p[f'{bin_name}.y'])
            assert (bx, by) == BIN_XY[bin_name], (label, bin_name)
            for i, (ox, oy) in enumerate(zip(gx, gy)):
                cx, cy = bx + ox, by + oy
                slot = min(anchors,
                           key=lambda s: (anchors[s][0] - cx) ** 2
                           + (anchors[s][1] - cy) ** 2)
                assert slot == first_slot + i, \
                    (label, bin_name, f'格{i + 1}({cx:.3f},{cy:.3f}) 就近锚为槽位{slot}',
                     f'预期槽位{first_slot + i}')
                # 锚 xy 必须等于名义格心(settle 索引键 = 格心,不能漂)
                assert (round(anchors[slot][0], 3), round(anchors[slot][1], 3)) \
                    == (round(cx, 3), round(cy, 3)), (label, bin_name, slot)


# ------------------------------------------------ (d) 四箱 / 五键 / NaN 覆盖
def test_bins_are_v2_world_positions_with_executor_section():
    for label, file, section, bins in _SECTION_TARGETS:
        p = _params(section, file)
        assert [str(b) for b in p['bin_names']] == list(bins), \
            f'{label} bin_names 必须只含本臂路由箱(防南排暴露给 arm1)'
        for name in bins:
            assert (_num(p[f'{name}.x']), _num(p[f'{name}.y']),
                    _num(p[f'{name}.z'])) == (*BIN_XY[name], BIN_Z), \
                (label, name)
            joints = p[f'place_joints_{name}']
            assert isinstance(joints, list) and len(joints) == 6, (label, name)
            assert all(isinstance(v, float) and math.isnan(v) for v in joints), \
                f'{label} place_joints_{name} 必须显式全 NaN(否则架空 carry_anchor)'
        # 路由表:一箱一色,不混色
        expect = dict(zip(bins, bins))
        for color, target in zip([b.replace('bin_', '') for b in bins], bins):
            assert str(p[f'bin_for_{color}']) == target, (label, color)
        del expect


def test_recorder_bins_five_keys_and_consistent_with_executor():
    """四箱五键全给(x/y/half_x/half_y/inner_half_x/inner_half_y)+
    与 executor/relay 段箱心一致。64cd236 漏 x/y ⇒ 假 MISPLACED。"""
    for file in (_LEGACY_FILE, _PIPELINE_FILE):
        rec = _params('/result_recorder', file)
        exec_p = _params('/pick_place_executor', file)
        for name in BIN_XY:
            for suffix in ('x', 'y', 'half_x', 'half_y',
                           'inner_half_x', 'inner_half_y'):
                key = f'bins.{name}.{suffix}'
                assert key in rec, f'{file.name} recorder 缺五键之一: {key}'
            assert (_num(rec[f'bins.{name}.x']),
                    _num(rec[f'bins.{name}.y'])) == BIN_XY[name], (file.name, name)
            assert (_num(rec[f'bins.{name}.inner_half_x']),
                    _num(rec[f'bins.{name}.inner_half_y'])) == INNER_HALF, \
                (file.name, name)
            # half_* 不得大于内口(否则邻箱判定区重叠,旧 0.12/0.10 定罪)
            assert _num(rec[f'bins.{name}.half_x']) <= INNER_HALF[0] + 1e-9
            assert _num(rec[f'bins.{name}.half_y']) <= INNER_HALF[1] + 1e-9
        # 与执行器段箱心一致(两排各自)
        for bins, section in ((NORTH, '/pick_place_executor'),
                              (SOUTH, '/pick_place_relay')):
            p = _params(section, file)
            if not all(f'{b}.x' in p for b in bins):
                continue   # legacy 基表 relay 段是旧接力(bin_out),不含南排
            for name in bins:
                assert (_num(p[f'{name}.x']), _num(p[f'{name}.y']),
                        _num(p[f'{name}.z'])) == (*BIN_XY[name], BIN_Z)
        del exec_p


# ------------------------------------------------ (e) camera v2 定档
def test_camera_xy_and_recorder_camera_v2():
    for label, file, section in (
            ('legacy executor', _LEGACY_FILE, '/pick_place_executor'),
            ('legacy relay(历史段)', _LEGACY_FILE, '/pick_place_relay'),
            ('pipeline executor', _PIPELINE_FILE, '/pick_place_executor'),
            ('pipeline relay', _PIPELINE_FILE, '/pick_place_relay')):
        p = _params(section, file)
        assert [_num(v) for v in p['camera_xy']] == CAMERA_XY, \
            f'{label} camera_xy 必须 v2 定档 {CAMERA_XY}'
    # 相机安装位姿在基表给全(pipeline 表只覆盖差值,加载顺序 = 基表在前)
    rec = _params('/result_recorder', _LEGACY_FILE)
    assert (_num(rec['camera.x']), _num(rec['camera.y']),
            _num(rec['camera.z'])) == (*CAMERA_XY, CAMERA_Z)
    assert _num(rec['camera.pitch_deg']) == 90.0
    for file in (_LEGACY_FILE, _PIPELINE_FILE):
        rec = _params('/result_recorder', file)
        for key in ('camera.x', 'camera.y', 'camera.z'):
            if key in rec:   # pipeline 表若写了相机键,必须与 v2 定档一致
                expect = {"camera.x": CAMERA_XY[0], "camera.y": CAMERA_XY[1],
                          "camera.z": CAMERA_Z}[key]
                assert _num(rec[key]) == expect, (file.name, key)


# ------------------------------------------------ (f) 两表 arm1 锚逐值一致
def test_arm1_anchors_identical_between_legacy_and_pipeline():
    """legacy 与 pipeline 的 arm1 必须同族同值(同一条物理臂、同一套 G0
    A 档锚)。允许差异的模式项显式列出,其余键凡两表都有的必须相等。"""
    a = _params('/pick_place_executor', _LEGACY_FILE)
    b = _params('/pick_place_executor', _PIPELINE_FILE)
    for slot in ANCHOR_SLOTS:
        assert a[f'carry_anchor{slot}_xy'] == b[f'carry_anchor{slot}_xy']
        assert a[f'carry_anchor{slot}_joints'] == b[f'carry_anchor{slot}_joints']
        for end in ('top', 'bottom'):
            assert a[f'carry_settle_{end}{slot}_joints'] == \
                b[f'carry_settle_{end}{slot}_joints']
    assert [float(v) for v in a['standby_joints']] == STANDBY_ARM1
    if 'standby_joints' in b:   # pipeline 表未写该键时由基表提供(覆盖语义)
        assert [float(v) for v in b['standby_joints']] == STANDBY_ARM1
    # 模式项(有注明的差异):tcp 0.211/0.204、carry_safe_z 1.05/1.08、取件锚
    assert _num(a['tcp_z_offset']) == 0.211     # legacy 定值不动
    assert _num(b['tcp_z_offset']) == 0.204     # pipeline Phase0 定档
    assert _num(a['carry_safe_z']) == 1.05
    assert _num(b['carry_safe_z']) == 1.08      # v2 两臂统一
    assert _num(a['settle_touch_z']) == _num(b['settle_touch_z']) == 0.958
    assert _num(a['place_drop_clearance']) == _num(b['place_drop_clearance']) == 0.09


def _same(x, y) -> bool:
    """值比较:容忍 NaN 与 1e9 的 str/double 两种解析口径。"""
    if isinstance(x, (list, tuple)) or isinstance(y, (list, tuple)):
        if not (isinstance(x, (list, tuple)) and isinstance(y, (list, tuple))) \
                or len(x) != len(y):
            return False
        return all(_same(i, j) for i, j in zip(x, y))
    if isinstance(x, float) and math.isnan(x):
        return isinstance(y, float) and math.isnan(y)
    try:
        return _num(x) == _num(y)
    except (TypeError, ValueError):
        return str(x) == str(y)


def test_relay_section_identical_between_both_tables():
    """基表与 pipeline 表的 /pick_place_relay 段必须**逐值一致**。

    沿革:旧基表 relay 段是"经典接力(arm2 高肩立柱投 bin_out)"的历史标定,
    v2 已随 pedestal2 高立柱与 bin_out 模型一并废止。若基表留旧值而只改
    pipeline 表,`dual_arm:=true pipeline:=false` 会读到已废止的高肩锚 +
    不存在的 bin_out = 静默错配(64cd236 型假信号)。两表同源 ⇒ 任何加载
    组合都自洽;本闸锁死两源不漂移。
    """
    ra = _params('/pick_place_relay', _LEGACY_FILE)
    rb = _params('/pick_place_relay', _PIPELINE_FILE)
    assert set(ra) == set(rb), sorted(set(ra) ^ set(rb))
    for key in sorted(set(ra) & set(rb)):
        assert _same(ra[key], rb[key]), (key, ra[key], rb[key])


def test_relay_standby_and_belt_pick_anchor_v2():
    """arm2 relay 段 v2 定档:standby = G0 值(与 dual xacro 同值)、
    带尾取件锚 xy = **[0.60, 0.0]**(2026-10-04 晚用户拍板拉长带后的新停位,
    = 锚表 meta.belt_stop)、箱内取件 6 槽显式清空。"""
    r = _params('/pick_place_relay', _PIPELINE_FILE)
    assert [float(v) for v in r['standby_joints']] == STANDBY_ARM2
    assert (_num(r['standby_x']), _num(r['standby_y']),
            _num(r['standby_z'])) == (0.300, -0.470, 1.150)
    assert [_num(v) for v in r['pick_anchor1_xy']] == BELT_STOP_XY
    assert all(_finite(v) for v in r['pick_anchor1_joints'])
    # 第二(带外兜底)槽必须禁用:终版表只解 y=0,服务带契约 |y|<=0.019
    assert _num(r['pick_anchor2_xy'][0]) >= SENTINEL, \
        '带外兜底槽已随终版表删除,必须置哨兵禁用'
    assert all(isinstance(v, float) and math.isnan(v)
               for v in r['pick_anchor2_joints'])
    for slot in ANCHOR_SLOTS:
        xy = r[f'pick_grasp_anchor{slot}_xy']
        assert _num(xy[0]) >= SENTINEL, \
            f'pick_grasp_anchor{slot} 必须清空(并联不经箱内取件)'
        assert all(isinstance(v, float) and math.isnan(v)
                   for v in r[f'pick_grasp_anchor{slot}_joints'])
    assert all(isinstance(v, float) and math.isnan(v)
               for v in r['place_anchor_joints'])
    # v2 定档口径(Lead 派单)
    assert _num(r['feed_z']) == 0.805
    assert _num(r['tcp_z_offset']) == 0.204
    assert _num(r['verify_probe_offset']) == 0.04
    assert str(r['attach_follow']) == 'verified_only'
    assert _num(r['place_drop_clearance']) == 0.09
    assert _num(r['settle_touch_z']) == 0.958
    assert bool(r['pick_descend_linear']) is False
    assert _num(r['vel_scale']) == _num(r['acc_scale']) == 0.2   # G3 定罪
    # (2026-10-05):0.25 时带尾构型 verify_probe 短 LIN 的 IK 采样在
    # t≈0.54s 踩 Pilz 关节加速度限(r1b 实录 PLANNING_FAILED×3 耗尽),
    # 两臂 scale 同步降 0.2;非判据红线(τ/FK/间隙/gain/tcp/锚位不动)。
    assert _num(r['carry_safe_z']) == 1.08
    assert str(r['arm_name']) == 'arm2'
    assert str(r['command_topic']) == '/task/command_arm2'
    assert str(r['group_name']) == 'arm2_ur_manipulator'
    assert str(r['tool_frame']) == 'arm2_tool0'
    assert [str(j) for j in r['joint_names']] == [
        f'arm2_{n}_joint' for n in ('shoulder_pan', 'shoulder_lift', 'elbow',
                                    'wrist_1', 'wrist_2', 'wrist_3')]


def test_legacy_standby_matches_yaml_xyz():
    """基表 arm1 standby:G0 定档关节值 + xyz = notes §1 的 tool0 位姿。"""
    a = _params('/pick_place_executor', _LEGACY_FILE)
    assert [float(v) for v in a['standby_joints']] == STANDBY_ARM1
    assert (_num(a['standby_x']), _num(a['standby_y']),
            _num(a['standby_z'])) == (1.035, 0.470, 1.080)


# ------------------------------------------------ (g) 终版锚表逐值交叉核对
def _anchor_table():
    assert _ANCHOR_JSON.exists(), \
        f'缺终版锚表 {_ANCHOR_JSON}(Lead 2026-10-05 收口指令指定唯一数据源)'
    return json.loads(_ANCHOR_JSON.read_text(encoding='utf-8'))


_V2_JSON = _ANCHOR_JSON.with_name('G0_anchors_v2.json')


def _anchor_v2_meta():
    # v3 无 belt_stop/res_from v2 ⇒ 带尾口径从 v2 meta 读(文件随表在库)
    return json.loads(_V2_JSON.read_text(encoding='utf-8'))['meta']


def test_wired_anchors_match_G0_anchors_v3_json():
    """两份 yaml 里所有已接线的关节锚必须与 docs/evidence/G0_anchors_v3.json
    **逐字一致**(v3 起按 meta.publish_decimals 发布,其中 3 锚提精到 5 位小数;
    yaml 数值与 JSON 存储值 float 相等,不再回绕到 4 位比辂)。锚表若再次
    重发,本闸立刻抓出漂移,而不是让人
    靠肉眼比对(2026-10-04 夜"手抄/同源同错"的直接教训)。

    覆盖 = 每臂 6 carry + 12 settle + 1 带尾 rise(pick 已并入"rise +
    笛卡尔下探",不作生效键,只核 JSON.pick 的 z 与 tcp 定档自洽)。
    """
    T = _anchor_table()
    A = T['anchors']
    checked = 0
    for file, section, arm in (
            (_LEGACY_FILE, '/pick_place_executor', 'arm1'),
            (_LEGACY_FILE, '/pick_place_relay', 'arm2'),
            (_PIPELINE_FILE, '/pick_place_executor', 'arm1'),
            (_PIPELINE_FILE, '/pick_place_relay', 'arm2')):
        p = _params(section, file)
        for i, color in enumerate(SLOT_COLOR[arm], 1):
            pairs = ((f'carry_anchor{i}_joints', f'{color}.carryA'),
                     (f'carry_settle_top{i}_joints', f'{color}.setTopA'),
                     (f'carry_settle_bottom{i}_joints', f'{color}.setBotA'))
            for key, jkey in pairs:
                assert key in p, (file.name, section, key)
                got = [float(v) for v in p[key]]
                exp = [float(v) for v in A[arm][jkey]['q']]
                assert got == exp, (file.name, section, key, got, exp)
                checked += 1
            # 名义格心 = JSON target 的 xy(= 箱心 + 网格偏移)
            t = A[arm][f'{color}.carryA']['target']
            xy = [round(_num(v), 2) for v in p[f'carry_anchor{i}_xy']]
            assert xy == [round(t[0], 2), round(t[1], 2)], \
                (file.name, section, i, xy, t)
        # 带尾锚:生效键 = rise(上空 PTP),其 xy 必须是新停位
        if 'pick_anchor1_joints' in p:
            got = [float(v) for v in p['pick_anchor1_joints']]
            exp = [float(v) for v in A[arm]['rise']['q']]
            assert got == exp, (file.name, section, 'pick_anchor1', got, exp)
            assert [round(_num(v), 2)
                    for v in p['pick_anchor1_xy']] == [round(c, 2)
                                                       for c in
                                                       A[arm]['rise']['target'][:2]]
            checked += 1
    # 段×锚:4 段各 18 关节锚 + 3 段有带尾 rise 键(基表 executor 无取件
    # 锚键,走笛卡尔 pre_grasp,属 legacy 原口径)⇒ 72 + 3
    assert checked == 75, f'交叉核对项数 {checked} != 75(72 关节锚 + 3 带尾)'


def test_anchor_table_gates_and_belt_stop_are_pinned():
    """锚表自身的判据口径(meta)与带尾停位必须与接线一致:
    fk 闸 40µm / τ 闸 30 / 净空闸 8mm(未放宽),belt_stop = (0.60, 0)。
    另核对:执行器 rise 锚的 tool0 z 与 settle 触底档、tcp_z_offset 0.204
    下 JSON.pick 的深度档 1.009 自洽(0.805+0.204=1.009)。"""
    T = _anchor_table()
    meta = T['meta']
    gate = meta['gate']       # v3 布局:gate{fk_um,tau,clr_mm}
    assert float(gate['fk_um']) == 40.0
    assert float(gate['tau']) == 30.0
    assert float(gate['clr_mm']) == 8.0
    # v3 belt_stop 沿 res_from(v2)口径;w3_rule 锁 OLD standby 归一
    assert [round(float(v), 2) for v in _anchor_v2_meta()['belt_stop']] == \
        [round(c, 2) for c in BELT_STOP_XY]
    assert float(meta['publish_decimals']) == 5
    assert float(meta['publish_res_max_um']) <= float(gate['fk_um'])
    assert float(meta['pole_final']) == -0.10
    p = _params('/pick_place_executor', _PIPELINE_FILE)
    feed_z, tcp = _num(p['feed_z']), _num(p['tcp_z_offset'])
    pick_z = T['anchors']['arm1']['pick']['target'][2]
    assert abs(feed_z + tcp - pick_z) <= 1e-3, \
        f'带尾深度档不自洽:feed_z {feed_z} + tcp {tcp} != JSON.pick z {pick_z}'
    disp = _params('/task_dispatcher', _PIPELINE_FILE)
    # 停位线 0.58 + 越线偏置 ~0.02 ⇒ 停位 0.60(与 belt_stop 锚一致)
    assert abs(_num(disp['belt_stop_x']) - 0.58) < 1e-9
    for arm in ('arm1', 'arm2'):
        for key, v in T['anchors'][arm].items():
            assert float(v['fk_um']) <= float(gate['fk_um']), (arm, key, v['fk_um'])
            if v.get('tau') is not None:
                assert float(v['tau']) < float(gate['tau']), (arm, key, v['tau'])
    # R4 w3 归一(消回绕):全表任一锚的 wrist_3 与 OLD standby 数值差
    # 不得超过 π(±2π 等价分支取最短行程;v2 曾把 arm1 蓝格/带尾解放在
    # +2π 分支 ⇒ standby→锚 PTP 行程 4.74rad,已废)。
    sb = T['meta']['w3_ref_standby']
    for arm in ('arm1', 'arm2'):
        w3_ref = float(sb[arm][5])
        for key, v in T['anchors'][arm].items():
            d = abs(float(v['q'][5]) - w3_ref)
            assert d <= math.pi + 1e-6, (arm, key, round(d, 3))
    # meta 里的 PTP 行程记录同步(实测值,防表被换口径重发)
    assert float(T['meta']['ptp_max']['arm1'][0]) <= math.pi
    assert float(T['meta']['ptp_max']['arm2'][0]) <= math.pi
    # standby 与 xacro/yaml 同值(OLD 保留裁定)也归本闸核
    assert sb['arm1'] == STANDBY_ARM1 and sb['arm2'] == STANDBY_ARM2


# ------------------------------------------------ (h) xacro ↔ yaml 同步闸
def _xacro_initial_positions(path: Path) -> dict[str, list[float]]:
    """从 xacro 文本抽各 <xacro:ur_ros2_control> 块的 initial_positions
    (纯 regex,不跑 ROS)。返回 {块标识: 6 个 double},块标识取该块的
    tf_prefix('' -> 'arm1'、'arm2_' -> 'arm2');两臂的关节名相同(前缀由
    官方宏内部加),故**不能**按关节名建字典,只能按块序/tf_prefix 分。"""
    txt = path.read_text(encoding='utf-8')
    out: dict[str, list[float]] = {}
    for m in re.finditer(
            r'<xacro:ur_ros2_control\b(.*?)initial_positions="\$\{dict\('
            r'(.*?)\)\}"', txt, re.S):
        head, body = m.group(1), m.group(2)
        pairs = re.findall(r'([A-Za-z0-9_]+joint)=(-?[\d.]+)', body)
        assert len(pairs) == 6, (head[:60], pairs)
        tp = (re.search(r'tf_prefix="([^"]*)"', head) or
              re.search(r'name="([^"]*)"', head))
        assert tp, head[:80]
        tag = tp.group(1)
        # 单臂文件的块写的是 tf_prefix="$(arg tf_prefix)"/name="$(arg name)"
        # (运行时才定),它只能是 arm1(双臂版另有独立文件);双臂版按
        # 显式 name/tf_prefix 区分 arm1 / arm2。
        if 'arm2' in tag:
            key = 'arm2'
        else:
            key = 'arm1'
        assert key not in out, f'重复块 {key}(tag={tag})'
        out[key] = [float(v) for _, v in pairs]
    return out


def test_xacro_spawn_pose_matches_yaml_standby():
    """出生位形(xacro initial_positions)必须与 yaml standby_joints 同值。

    机理:机器人一出生就应已在待机位,首段才不是"跨构型大转移"(Pilz PTP
    的关节路径会因此间歇性撞自碰撞被拒解,实证坑)。Lead 单发新 standby
    (arm2 重验中 / camera 终值)时,本闸保证**不会只改一处**:
      单臂 xacro == 基表 executor == pipeline executor(arm1)
      双臂 xacro arm1 == arm1 值;双臂 xacro arm2 == 两表 relay 段值
    """
    urdf_dir = _PKG_ROOT / 'src/a1_cell_sim/urdf'
    single = _xacro_initial_positions(urdf_dir / 'ur5e_cell.urdf.xacro')
    dual = _xacro_initial_positions(urdf_dir / 'ur5e_cell_dual.urdf.xacro')
    assert set(single) == {'arm1'}, single
    assert set(dual) == {'arm1', 'arm2'}, dual
    s_vals = single['arm1']

    a_exec = _params('/pick_place_executor', _LEGACY_FILE)
    a_relay = _params('/pick_place_relay', _LEGACY_FILE)
    p_relay = _params('/pick_place_relay', _PIPELINE_FILE)
    assert [float(v) for v in a_exec['standby_joints']] == s_vals
    assert dual['arm1'] == s_vals, '单臂/双臂的 arm1 出生位形必须同值'
    for tag, p in (('基表 relay', a_relay), ('pipeline relay', p_relay)):
        assert [float(v) for v in p['standby_joints']] == dual['arm2'], tag
    assert dual['arm2'] == STANDBY_ARM2, 'arm2 出生位应 = 单测钉的 G0 定档值'
