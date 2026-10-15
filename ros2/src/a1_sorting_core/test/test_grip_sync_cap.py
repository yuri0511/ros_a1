"""P1 sync_cap 单测(2026-10-08):封顶判据纯函数 + 关节名双存键取数。

背景(P1 根因,HANDOFF-20261007 §九):sync_cap 在双臂下静默失效——
`_grip` 按 `arm2_gripper_finger_*` 查 `_joints_now`,而该表键名来自
`_on_joint_state` 原样存的 msg.name;若 joint_states 发布者发**裸关节
名**(双臂同名一条消息各发一份),arm2 的键永远查不到 → 读恒为基准 →
dl/dr 恒 0 → 单侧先行条件永不成立 → 轮询白跑;旧收尾 `if not (capped
or polled)` 又在 polled=True 时跳过 sleep(死分支),轻触段退化为"发
命令即返回"。本文件钉住修复后的两层:

(a) should_cap(dl, dr) 纯函数判据(触发边界/对称性/双侧同步不误触);
(b) _store_joint_states + _joint_lookup 取数层:裸名双份世界第二份
    存 arm2_ 别名(不覆盖第一份 ⇒ arm1 数据不被跨臂误读,10-06 §二
    定罪的候选形态),前缀世界逐键原样存(零回归)。

取数层用桩 self 直调未绑定方法(不实例化节点、不碰 rclpy 生命周期);
判据层是模块级纯函数。executor 模块顶层 import rclpy(既有约定),
无 ROS 环境按 importorskip 跳过。
"""

import pytest

pytest.importorskip('rclpy', reason='executor 模块顶层依赖 rclpy')

from a1_sorting_core.pick_place_executor_node import (  # noqa: E402
    _JOINT_ALT_PREFIXES, should_cap)


class _FakeMsg:
    """sensor_msgs/JointState 桩(只喂 name/position/velocity 三字段)。"""

    def __init__(self, names, positions, velocities=None):
        self.name = names
        self.position = positions
        self.velocity = velocities if velocities is not None else (
            [0.0] * len(names))


class _FakeSelf:
    """_store_joint_states/_joint_lookup 所需属性的最小桩。"""

    def __init__(self):
        self._joints_now = {}
        self._joint_vels = {}
        self._joints_alias = {}
        self._joint_vels_alias = {}


def _store(msg):
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode
    s = _FakeSelf()
    PickPlaceExecutorNode._store_joint_states(s, msg)
    return s


def _lookup(s, name):
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode
    return PickPlaceExecutorNode._joint_lookup(s, name)


# ---------------------------------------------------------------- (a) 判据
def test_should_cap_triggers_on_left_lead():
    """左指先行 ≥15mm 且右指 <5mm ⇒ 封顶(10-06 pipeline 实录形态)。"""
    assert should_cap(0.015, 0.0) is True
    assert should_cap(0.043, 0.001) is True   # r9 空扫 43mm 定罪样本


def test_should_cap_triggers_on_right_lead_symmetric():
    """右指先行同样触发(10-06 dual 实录 L=0 R=15mm 形态)——判据对称。"""
    assert should_cap(0.0, 0.015) is True
    assert should_cap(0.002, 0.02) is True


def test_should_cap_no_trigger_below_lead_threshold():
    """先行侧未过 15mm 不触发;恰在 14.9mm 也不触发(边界严格 ≥)。"""
    assert should_cap(0.0149, 0.0) is False
    assert should_cap(0.0, 0.0149) is False
    assert should_cap(0.0, 0.0) is False      # 键失效世界的全 0 恒不触发


def test_should_cap_no_trigger_when_both_move_together():
    """双侧同步推进(滞后侧 ≥5mm)不触发——正常夹取不误封顶。"""
    assert should_cap(0.015, 0.005) is False  # 滞后侧恰 5mm:< lag 为假
    assert should_cap(0.02, 0.018) is False   # 同步到半程
    assert should_cap(0.03, 0.03) is False


def test_should_cap_thresholds_overridable():
    """阈值参数化(默认为线上口径 15mm/5mm)。"""
    assert should_cap(0.010, 0.0, lead_m=0.008, lag_m=0.005) is True
    assert should_cap(0.010, 0.006, lead_m=0.008, lag_m=0.005) is False


# ---------------------------------------------------------------- (b) 取数
def test_bare_duplicate_names_store_both_arms():
    """裸关节名双份世界:同名的第 1 份(arm1)存主表,第 2 份存 arm2_
    别名表;两臂都查得到且各自读到**自己的**指(修复 arm2 查不到的根因,
    且不把 arm2 误接到 arm1 数据上(10-06 §二 跨臂误读定罪形态))。"""
    msg = _FakeMsg(
        ['gripper_finger_left_joint', 'gripper_finger_right_joint',
         'gripper_finger_left_joint', 'gripper_finger_right_joint'],
        [0.030, 0.031, 0.020, 0.021])
    s = _store(msg)
    # arm1(主表裸键)
    assert _lookup(s, 'gripper_finger_left_joint') == 0.030
    assert _lookup(s, 'gripper_finger_right_joint') == 0.031
    # arm2(别名表前缀键)
    assert _lookup(s, 'arm2_gripper_finger_left_joint') == 0.020
    assert _lookup(s, 'arm2_gripper_finger_right_joint') == 0.021
    assert _JOINT_ALT_PREFIXES == ('arm2_',)   # 前缀表钉 gripper.xacro 口径


def test_prefixed_publish_world_zero_regression():
    """前缀发布者世界(gz_ros_control 实录口径):逐键原样存主表,
    别名表不启用;arm2 直接查得到(10-06 l1_a dual 轮触发即此世界)。"""
    msg = _FakeMsg(
        ['gripper_finger_left_joint', 'gripper_finger_right_joint',
         'arm2_gripper_finger_left_joint', 'arm2_gripper_finger_right_joint'],
        [0.030, 0.031, 0.020, 0.021])
    s = _store(msg)
    assert s._joints_alias == {}               # 无重名 ⇒ 别名表零触碰
    assert _lookup(s, 'arm2_gripper_finger_left_joint') == 0.020
    assert _lookup(s, 'gripper_finger_left_joint') == 0.030


def test_triplicate_names_only_two_aliases():
    """超过表容量的重名份(理论不发生)只别名前 len(表) 份,主表恒为
    第 1 份 ⇒ 任何情况下 arm1 裸键不被后续份覆盖(跨臂误读的硬保证)。"""
    msg = _FakeMsg(['x_joint'] * 3, [1.0, 2.0, 3.0])
    s = _store(msg)
    assert s._joints_now['x_joint'] == 1.0
    assert s._joints_alias == {'arm2_x_joint': 2.0}


def test_lookup_falls_back_to_alias_then_none():
    """查键次序:主表优先 → 别名表 → None(触发启动自检的"查不到"口径)。"""
    s = _store(_FakeMsg(['gripper_finger_left_joint'] * 2, [0.03, 0.02]))
    assert _lookup(s, 'gripper_finger_left_joint') == 0.03
    assert _lookup(s, 'arm2_gripper_finger_left_joint') == 0.02
    assert _lookup(s, 'arm3_gripper_finger_left_joint') is None


def test_snapshot_semantics_for_travel_read():
    """轻触段轮询语义互证:基准=轮询前查到的值,行程=基准-当前;
    取数层修复前两臂均可产生非零 dl/dr(旧世界 arm2 恒 None→不轮询)。"""
    s = _store(_FakeMsg(['gripper_finger_left_joint'] * 2, [0.040, 0.030]))
    base_l = _lookup(s, 'arm2_gripper_finger_left_joint')
    # 下一帧:arm1 指钉住不动,arm2 左指空扫到 12mm(单侧先行场景)
    _store_into(s, _FakeMsg(['gripper_finger_left_joint'] * 2,
                            [0.040, 0.012]))
    dl = base_l - _lookup(s, 'arm2_gripper_finger_left_joint')
    assert dl == pytest.approx(0.018)          # arm2 行程真实可读
    assert should_cap(dl, 0.0) is True         # 先行 18mm ≥15mm ⇒ 封顶
    # arm1 侧同帧互证:裸键恒跟第 1 份(钉住不动的 0.040),行程 0 不触发
    _store_into(s, _FakeMsg(['gripper_finger_left_joint'] * 2,
                            [0.040, 0.010]))
    assert _lookup(s, 'gripper_finger_left_joint') == 0.040


def _store_into(s, msg):
    """同桩二次灌帧(_store 每次新建桩,轮询语义需要同一桩多帧)。"""
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode
    PickPlaceExecutorNode._store_joint_states(s, msg)
