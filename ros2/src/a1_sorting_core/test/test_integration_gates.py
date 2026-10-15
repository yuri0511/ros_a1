"""集成闸门(pipeline 模式组长集成新增逻辑)纯逻辑回归测试:不初始化 rclpy。

覆盖(2026-10-01 收尾测试拉齐,任务书 A1-C3):
(a) place_descend_target_z:place_drop_clearance 0 / 0.09(pipeline 实配值,
    sorting_params_pipeline.yaml)的放置下探终点;
(b) verify_probe_targets:探针方向回归 —— 去程为 **-x 方向**(去程
    x = center_x - offset,+x 侧紧邻挡停板会把贴指工件怼进板里挤脱,
    联调定论改 -x/带尾方向),以及 offset<=0 禁用返回 None;
(c) BeltController 推板三命令时序回归:按节点发布次序模拟整轮
    前进(+0.1) -> 回位(-0.25,仅一次) -> 推板回到起点附近才停板(0.0);
    墙钟 settle 不再放行。单元级用例在 test_pipeline_logic.py;
(d) 在场闸(part_not_under_gripper,3cm):该判定内联于
    pick_place_executor_node._run_task_impl(夹爪 xy 与最新真值 xy 的
    math.hypot 距离 >0.03 抛 GraspError),模块顶层 import rclpy,本
    测试环境(PYTHONPATH 方式、不 source ROS)无法导入 —— 按任务书
    约定跳过用例并在测试报告注明,不为测试改源码。
(e) 拉长带几何↔参数一致性闸 + 静置槽 FEED_SLOTS↔带面闸(2026-10-04 用户拍板拉长带,结构件,不涉锚值):
    实读 sorting_cell.sdf 的带面/挡停板/推板限位 + pipeline yaml 的
    belt_stop_x,校验"停位→板内缘"余量与旧定案态等价(见函数内注释)。
时间戳由测试注入(流水线按仿真时间理解),与 pipeline_logic 模块约定一致。
"""
import re
from pathlib import Path

import pytest
import yaml

from a1_sorting_core.pipeline_logic import (BeltController,
                                            place_descend_target_z,
                                            verify_probe_targets)

# pipeline 实配常数(sorting_params_pipeline.yaml,联调修订值)
TCP_Z = 0.204          # /pick_place_executor.tcp_z_offset(2026-10-04 Phase0
                        # 定档:原 0.208,减小 4mm 加深包覆,根治扫帚首败;
                        # URDF 合法域 [0.201,0.231])
PLACE_Z = 0.745        # 箱内静置中心(布局 v2:bin_blue/bin_yellow/
                         # bin_red/bin_green 四箱同值 = 内底 0.72 + 半边 0.025)
CLEARANCE = 0.09       # place_drop_clearance(0.06 时指尖低壁顶 3mm 犁壁)
LIFT_H = 0.06          # lift_height


# -------------------------------------------------- (a) 放置下探终点 z
def test_gate_place_descend_clearance_zero_hits_place_plane():
    """clearance=0(legacy 默认/关闭落入):终点 = place_z + tcp_z,
    下探到放置面。pipeline 实配与 legacy 实配各验一组。"""
    assert place_descend_target_z(PLACE_Z, TCP_Z, 0.0) == pytest.approx(0.949)
    # legacy 实配(0.756 + 0.211)不回归
    assert place_descend_target_z(0.756, 0.211, 0.0) == pytest.approx(0.967)


def test_gate_place_descend_clearance_009_stops_above_wall():
    """clearance=0.09(pipeline 实配):终点 = place_z + tcp_z + 0.09 =
    1.039;若下探到该终点,松爪时工件中心 = place_z + 0.09 = 0.835,
    底面 0.810——箱口 0.815(壁高 0.095,2026-10-02 加高)下 5mm,
    块底已入口、短落距无弹跳。"""
    z = place_descend_target_z(PLACE_Z, TCP_Z, CLEARANCE)
    assert z == pytest.approx(1.039)
    assert z - TCP_Z == pytest.approx(PLACE_Z + CLEARANCE)   # 工件中心 0.835
    assert z - TCP_Z - 0.025 == pytest.approx(0.810)         # 工件底
    assert place_descend_target_z(PLACE_Z, TCP_Z, 0.09) - \
        place_descend_target_z(PLACE_Z, TCP_Z, 0.0) == pytest.approx(0.09)


# -------------------------------------------------- (b) 水平验证探针
def test_gate_probe_outbound_is_minus_x_direction():
    """探针去程 = -x 方向(带尾方向):去程 x = center_x - offset,
    回程回取件中心;高度 = 抬升终点(center_z + tcp_z + lift_h)。
    以 arm1 带上取件实配(center 0.805)与 arm2 箱内取件(feed_z 0.745)
    各验一组。"""
    out, back = verify_probe_targets((0.44, 0.0, 0.805), TCP_Z, LIFT_H, 0.04)
    assert out[0] == pytest.approx(0.44 - 0.04)     # 去程 x = center_x - offset
    assert out[0] < back[0]                          # 方向回归:-x 去程
    assert out[1:] == back[1:] == pytest.approx(
        (0.0, 0.805 + TCP_Z + LIFT_H))
    out2, back2 = verify_probe_targets((0.62, 0.20, 0.745), TCP_Z, LIFT_H,
                                       0.04)
    assert out2[0] == pytest.approx(0.58)
    assert out2[0] < back2[0]
    assert back2 == pytest.approx((0.62, 0.20, 0.745 + TCP_Z + LIFT_H))


def test_gate_probe_disabled_returns_none():
    """offset=0 或负值 -> None(禁用探针,不产生任何运动)。"""
    assert verify_probe_targets((0.44, 0.0, 0.805), TCP_Z, LIFT_H,
                                0.0) is None
    assert verify_probe_targets((0.44, 0.0, 0.805), TCP_Z, LIFT_H,
                                -0.04) is None


# -------------------------------------------------- (c) 推板三命令时序
def make_belt(stop_x: float = 0.40, settle_delay: float = 2.6
              ) -> BeltController:
    return BeltController(stop_x=stop_x, settle_delay=settle_delay)


def test_gate_pusher_full_cycle_command_timeline():
    """整轮回归:前进 +0.1 -> 越线回位 -0.25(仅一次) -> 推板世界坐标
    回到起点附近才停板 0.0。墙钟 settle 不再放行。三命令值互不相同。"""
    b = make_belt()
    cmds = []
    b.start()
    cmds.append(b.advance_speed)                    # +0.1 前进(推工件)
    frames = [(1.0, [0.30]), (1.2, [0.36]), (1.4, [0.40]), (1.6, [0.43])]
    retract_at = None
    for now, xs in frames:
        cmd = b.observe(xs, now)
        if cmd is not None:
            cmds.append(cmd)
            retract_at = now
    assert cmds == [0.1, -0.25]
    assert retract_at == 1.4
    assert b.observe([0.44], 1.7) is None           # 回位命令只发一次
    assert b.may_dispatch(retract_at + 2.6 + 0.01) is False  # 墙钟到期仍挂起
    assert b.observe_pusher(0.50, retract_at + 1.0) is None
    assert b.observe_pusher(0.14, retract_at + 1.1) == 0.0
    assert b.may_dispatch(retract_at + 1.1) is True
    cmds.append(b.stop_speed)                       # 回到起点后节点发 0.0
    assert cmds == [0.1, -0.25, 0.0]
    assert len(set(cmds)) == 3


def test_gate_pusher_second_part_repeats_timeline():
    """第二件 start() 复位后整条时序可重复(逐件循环回归):
    门控重新关闭 -> 越线再回位 -> 再次回到起点才放行;命令值不变。"""
    b = make_belt()
    b.start()
    assert b.observe([0.41], 1.0) == -0.25
    assert b.observe_pusher(0.12, 2.0) == 0.0
    assert b.may_dispatch(2.0) is True
    cmds = [b.advance_speed]
    b.start()                                       # 第二件:推板再前进
    assert b.may_dispatch(3.7) is False             # 门控重新关闭
    assert b.observe([0.35], 3.8) is None           # 新件未越线不回位
    assert b.observe([0.40], 4.0) == -0.25
    assert b.may_dispatch(4.0 + 2.6 + 0.01) is False
    assert b.observe_pusher(0.14, 5.0) == 0.0
    assert b.may_dispatch(5.0) is True
    cmds.append(b.stop_speed)
    assert len(set(cmds + [b.retract_speed])) == 3


# -------------------------------------------------- (d) 在场闸
# 跳过说明:在场闸(最新真值 xy 与夹爪 xy 距离 >0.03 -> 抛
# GraspError('part_not_under_gripper'))内联在
# a1_sorting_core/pick_place_executor_node.py 的 _run_task_impl 中,
# 该模块顶层 import rclpy,本测试环境不 source ROS 无法导入;
# 按任务书约定不为测试改源码,故此处不写复刻断言(复刻逻辑与源码
# 无耦合,绿了也证明不了源码行为)。待该闸抽到 pipeline_logic 后
# 可直接补:同点 <0.03 放行 / 恰 0.03 放行(严格大于才拒)/
# 5cm 拒绝 / 一侧为 None 拒绝。


# -------------------------------------------------- (e) 拉长带 世界↔参数一致性
_WS = Path(__file__).resolve().parents[3]
_SDF = _WS / 'src/a1_cell_sim/worlds/sorting_cell.sdf'
_PIPELINE_YAML = _WS / 'src/a1_sorting_core/config/sorting_params_pipeline.yaml'
PART_HALF = 0.025          # 工件边长 0.05 的一半


def _model_block(txt: str, name: str) -> str:
    m = re.search(r'<model name="%s">.*?</model>' % name, txt, re.S)
    assert m, 'sdf 里找不到 model ' + name
    return m.group(0)


def test_belt_geometry_v2_lengthened_and_params_agree():
    """拉长带(用户拍板 2026-10-04)三件套 + 停位线的结构一致性闸。

    钉死内容(**只钉几何关系,不钉任何关节锚值**——锚表待 Lead 重发):
      - 带面 size x 0.60、pose x 0.45 ⇒ 覆盖 x 0.15..0.75(旧 0.15..0.50);
      - 带顶高度 0.78 未动(pose z 0.75 + size z 0.06);
      - 挡停板 pose x = 0.71、内缘 0.705;推板 joint upper = 0.59;
      - belt_stop_x = 0.58 ⇒ 停位 world_x ≈ 0.60,工件前缘→板内缘余量
        **与旧定案态逐值等价**(旧:停位 0.36 → 板内缘 0.465 = 8cm;
        新:停位 0.60 → 板内缘 0.705 = 8cm),即"平移 +0.24、相对几何不变"。
        这条余量是探针(+x 0.04)不撞板的物理成因,不许被后续改动吃掉。
      - friction mu/mu2/fdir1 与推板话题名未动(记忆坑:话题名由模型/链接
        名派生,改名即断链)。
    """
    txt = _SDF.read_text(encoding='utf-8')
    belt = _model_block(txt, 'belt')
    sizes = re.findall(r'<size>([\d.]+ [\d.]+ [\d.]+)</size>', belt)
    poses = re.findall(r'<pose>([\d.\-]+ [\d.\-]+ [\d.\-]+ 0 0 0)</pose>', belt)
    assert sizes and all(s == '0.60 0.16 0.06' for s in sizes), sizes
    assert poses and all(p == '0.45 0 0.75 0 0 0' for p in poses), poses
    # 带顶 = 0.75 + 0.06/2 = 0.78(高度未动)
    assert abs((0.75 + 0.06 / 2) - 0.78) < 1e-9
    # 摩擦与主方向未动
    assert '<mu>1.0</mu>' in belt and '<mu2>2.0</mu2>' in belt
    assert '<fdir1>0 1 0</fdir1>' in belt
    # 挡停板
    plate = _model_block(txt, 'stop_plate')
    pp = re.search(r'<pose>([\d.\-]+) 0 ([\d.\-]+) 0 0 0</pose>', plate)
    assert pp and float(pp.group(1)) == pytest.approx(0.71), plate[:120]
    assert float(pp.group(2)) == pytest.approx(0.79), '板底贴带顶 0.78 的旧写法未动'
    ps = re.search(r'<size>0.01 ([\d.]+) ([\d.]+)</size>', plate)
    assert ps, '挡停板厚 0.01 不应改动'
    plate_inner = 0.71 - 0.01 / 2          # 内缘 = pose x - 半板厚
    assert plate_inner == pytest.approx(0.705)
    # 推板行程上限
    pusher = _model_block(txt, 'pusher')
    lim = re.search(r'<lower>([\d.\-]+)</lower><upper>([\d.\-]+)</upper>', pusher)
    assert lim and float(lim.group(1)) == 0.0 and float(lim.group(2)) == pytest.approx(0.59), lim.groups()
    # 话题名由模型名/链接名派生(记忆坑):模型名、关节名、阻尼不得改
    assert 'pusher_slide' in pusher and '<damping>5.0</damping>' in pusher
    assert '<child>pusher_link</child>' in pusher

    # 参数侧:pipeline yaml 的停位线
    doc = yaml.safe_load(_PIPELINE_YAML.read_text(encoding='utf-8'))
    disp = doc['/task_dispatcher']['ros__parameters']
    stop_x = float(disp['belt_stop_x'])
    assert stop_x == pytest.approx(0.58)
    # 越线即停 ⇒ 停位 ≈ stop_x + 0.02(旧定案态 0.34→0.36 的同款偏置)
    rest_x = stop_x + 0.02
    margin = plate_inner - (rest_x + PART_HALF)
    assert margin == pytest.approx(0.08, abs=1e-6), \
        f'探针余量漂移到 {margin*1000:.1f}mm(旧定案态 80mm,不许吃进)'
    # 推板回位/生成点仍在带面内
    assert 0.15 <= 0.18 <= 0.75, '带尾生成位须落在带面覆盖内'
    assert rest_x + PART_HALF < plate_inner


def test_feed_slots_v2_consistent_with_belt_world():
    """Lead G1 收口亲手改的静置槽 FEED_SLOTS=(0.50/0.58/0.66, 0)(新基座下
    r 0.4805/0.4704/0.4738,G0 带尾域内,8cm 间距)。本闸校验几何:
      - 三槽全落在带面覆盖 [0.15, 0.75] 内且含件半宽不越界;
      - 间距 8cm >= 工件边长 5cm,相邻件不相碰;
      - 最远槽 + 工件半宽 距挡停板内缘 = 20mm(Lead 派单口径:
        0.66+0.025=0.685 对板内缘 0.705)——生成不被板挤住的物理成因;
      - 停位线 belt_stop_x 恰为中间槽 x(首件推至中槽停位语义保持)。
    不 import spawn_parts(a1_cell_sim 包在测试环境不可解析),
    regex 直读源文件字面量 ⇒ 改槽位必须同步过本闸。
    """
    src = (_WS / 'src/a1_cell_sim/scripts/spawn_parts.py').read_text(
        encoding='utf-8')
    m = re.search(r'FEED_SLOTS\s*=\s*\[([^\]]+)\]', src)
    assert m, 'spawn_parts 里找不到 FEED_SLOTS'
    slots = [(float(a), float(b))
             for a, b in re.findall(r'\(([\d.]+),\s*([\d.\-]+)\)', m.group(1))]
    assert len(slots) == 3, slots
    xs = [x for x, _ in slots]
    assert [y for _, y in slots] == [0.0, 0.0, 0.0]        # 全在带中线
    assert xs == sorted(xs) and len(set(xs)) == 3
    for x in xs:
        assert 0.15 + PART_HALF <= x <= 0.75 - PART_HALF, x
    assert xs[1] - xs[0] == pytest.approx(0.08)
    assert xs[2] - xs[1] == pytest.approx(0.08)
    txt = _SDF.read_text(encoding='utf-8')
    plate = _model_block(txt, 'stop_plate')
    px = float(re.search(r'<pose>([\d.\-]+) 0 ', plate).group(1))
    plate_inner = px - 0.01 / 2
    assert plate_inner - (xs[-1] + PART_HALF) == pytest.approx(0.020, abs=1e-9), \
        '最远槽距板 20mm 是 Lead 定档值,漂移即报错'
    doc = yaml.safe_load(_PIPELINE_YAML.read_text(encoding='utf-8'))
    stop_x = float(doc['/task_dispatcher']['ros__parameters']['belt_stop_x'])
    assert stop_x == pytest.approx(xs[1]), '停位线应恰为中间静置槽'

