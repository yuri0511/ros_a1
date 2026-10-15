"""Gazebo 抓取附着助手:运动学跟随(kinematic follow)实现。

为什么不用 fixed joint:实测(见 README"简化说明")libsdformat/gz-sim 8 的
EntityFactory 不允许通过 /world/<w>/create 建立跨模型 scoped parent/child
的 joint("parent frame not found in model ..."错误,joint 的 parent/child
只能在所在 model 自身作用域内解析),物理 weld 这条路在服务接口下走不通。

本实现(课程允许并注明的教学简化):
- attach:记录工件相对夹爪链接(TF, world 系)的齐次变换,启动跟随线程,
  以 rate_hz 频率把 T_world_gripper * T_rel 作为工件新位姿,经
  /world/<w>/set_pose 服务(ros_gz_interfaces/srv/SetEntityPose)下发;
- detach:停止跟随;工件恢复物理自由(重力/接触),落在桌面上。

真实场景中工件被抓起时初速近似为零,40Hz 更新间隙内自由落体仅 ~3mm,
每次下发都会拉回,视觉上是"随爪刚性移动"。
"""
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import Buffer, TransformException, TransformListener

MODEL = Entity.MODEL  # 实体类型常量(=2)


def pose_to_mat(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose -> 4x4 齐次矩阵。"""
    x, y, z, w = (pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
    mat = np.eye(4)
    mat[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    mat[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
    return mat


def mat_to_pose(mat: np.ndarray) -> Pose:
    """4x4 齐次矩阵 -> geometry_msgs/Pose(旋转矩阵转四元数,剪切忽略)。"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = mat[:3, 3]
    m = mat[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    pose.orientation.x, pose.orientation.y = x, y
    pose.orientation.z, pose.orientation.w = z, w
    return pose


class GzKinematicAttach:
    """工件随夹爪刚性移动的跟随控制器(同一时刻一个工件)。

    附着瞬间以逻辑相机的工件真值(xy 分量可靠)校正目标位姿:执行层的
    TCP 实际位置可能与命令值有厘米级偏差(IK 微扰重试/数值解误差),
    若按命令值记录相对位姿,工件会被钉在偏离夹爪中心的位置,与手指
    穿插挤压,释放时接触冲量把工件弹飞。
    """

    def __init__(self, node, world_name: str, rate_hz: float = 40.0,
                 timeout_sec: float = 5.0,
                 logical_camera_topic: str = '/overhead_logical',
                 camera_xy: tuple[float, float] = (0.65, 0.0)):
        self._node = node
        self._world = world_name
        self._period = 1.0 / max(1.0, rate_hz)
        self._timeout = timeout_sec
        self._client = node.create_client(
            SetEntityPose, f'/world/{world_name}/set_pose')
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node, spin_thread=False)
        self._stop = threading.Event()
        self._thread = None
        self._pending = None
        self._true_xy: dict[str, tuple[float, float]] = {}
        self._cam_xy = camera_xy
        if logical_camera_topic:
            from ros_gz_interfaces.msg import LogicalCameraImage
            node.create_subscription(LogicalCameraImage, logical_camera_topic,
                                     self._on_logical, 10)
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn('set_pose 服务未就绪,抓取附着将不可用')
        else:
            node.get_logger().info(f'附着助手就绪: /world/{world_name}/set_pose')

    def _on_logical(self, msg):
        """缓存工件真值 xy(世界系)。

        俯视逻辑相机 rpy=(0,90°,0):传感器系 y=世界 Δy, z=世界 Δx,
        x=深度(有系统偏差,不用)。相机安装见 worlds/sorting_cell.sdf。
        """
        for model in msg.model:
            self._true_xy[model.name] = (
                self._cam_xy[0] + model.pose.position.z,
                self._cam_xy[1] + model.pose.position.y)

    # ------------------------------------------------------------- 基础调用
    def _gripper_mat(self, gripper_frame: str):
        """最新 TF: world -> gripper_frame 齐次矩阵;不可用时返回 None。"""
        try:
            tf = self._tf.lookup_transform('world', gripper_frame,
                                           rclpy.time.Time())
        except TransformException:
            return None
        t = tf.transform
        mat = np.eye(4)
        q = t.rotation
        pose = Pose()
        pose.position.x = t.translation.x
        pose.position.y = t.translation.y
        pose.position.z = t.translation.z
        pose.orientation = q
        return pose_to_mat(pose)

    def _send_pose(self, entity: str, mat: np.ndarray):
        """异步 set_pose;**不丢拍**(搬运失真修复 2026-10-09)。

        旧版上一条未完成就整拍 return:大机动时 set_pose 往返延迟 >
        周期,实际更新率塌到 ~15-20Hz,工件"一跳一跳"落后夹爪(同步
        遥测实测搬运中偏差 22-43mm)。现改为最多等一个周期,然后照发
        **本拍最新位姿**(请求不堆积,节拍不被旧请求拖死)。"""
        if self._pending is not None and not self._pending.done():
            end = time.monotonic() + self._period
            while not self._pending.done() and time.monotonic() < end:
                time.sleep(0.002)
        request = SetEntityPose.Request()
        request.entity.name = entity
        request.entity.type = MODEL
        request.pose = mat_to_pose(mat)
        self._pending = self._client.call_async(request)

    # ------------------------------------------------------------ 附着/释放
    def attach(self, entity_name: str, gripper_frame: str,
               part_pose_world: Pose) -> bool:
        """开始让 entity 跟随 gripper_frame(以当前相对位姿刚性连接)。

        part_pose_world 的 xy 分量会被逻辑相机真值覆盖(若有缓存):
        执行层 TCP 实际位置可能与命令值有厘米级偏差,按命令值记录会把
        工件钉在偏离夹爪中心处,与手指穿插挤压导致释放时弹飞。
        """
        true_xy = self._true_xy.get(entity_name)
        if true_xy is not None:
            part_pose_world = self._clone_with_xy(part_pose_world, true_xy)
            self._node.get_logger().info(
                f'附着真值校正: {entity_name} -> '
                f'({true_xy[0]:.3f}, {true_xy[1]:.3f})')
        if self._thread is not None:
            self._node.get_logger().warn('已有跟随中的工件,先释放再附着')
            return False
        gripper = self._gripper_mat(gripper_frame)
        if gripper is None:
            self._node.get_logger().error(f'TF 不可用: world->{gripper_frame}')
            return False
        rel = np.linalg.inv(gripper) @ pose_to_mat(part_pose_world)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._follow, args=(entity_name, gripper_frame, rel),
            daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            f'已附着: {entity_name} 跟随 {gripper_frame}')
        return True

    @staticmethod
    def _clone_with_xy(pose: Pose, xy: tuple[float, float]) -> Pose:
        new = Pose()
        new.orientation = pose.orientation
        new.position.z = pose.position.z
        new.position.x, new.position.y = xy
        return new

    def _follow(self, entity: str, gripper_frame: str, rel: np.ndarray):
        while not self._stop.is_set():
            gripper = self._gripper_mat(gripper_frame)
            if gripper is not None:
                self._send_pose(entity, gripper @ rel)
            self._stop.wait(self._period)

    def detach(self) -> bool:
        """停止跟随(工件恢复物理自由)。未附着时为无害空操作。"""
        if self._thread is None:
            return True
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._thread = None
        self._node.get_logger().info('已解除附着(停止跟随)')
        return True
