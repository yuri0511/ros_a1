"""视觉分类节点:RGB 相机颜色判定 + 逻辑相机位姿 -> 工件类别判定。

工作方式:
1. 订阅 Gazebo 桥接的俯视 RGB 相机图像与相机内参;
2. 订阅逻辑相机话题(连续发布视野内模型名与相对位姿,不依赖渲染);
3. 把逻辑相机给出的工件相对位姿与已知安装位姿复合成世界位姿,
   再投影到图像像素,在像素窗口内做 HSV 颜色分类;
4. 连续 stable_count 帧判定一致后,发布 /parts/arrival (PartInfo) 登记来料。

简化声明:颜色判定是真实感知(看 RGB 图像);工件位置来自逻辑相机
(仿真传感器),未做像素级定位估计(README 有说明)。
"""
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from a1_interfaces.msg import PartInfo
from a1_vision.color_classifier import (classify_region, rpy_to_matrix,
                                        world_to_pixel)
from ros_gz_interfaces.msg import LogicalCameraImage


class VisionClassifierNode(Node):

    def __init__(self):
        super().__init__('vision_classifier')
        self.declare_parameter('camera_topic', '/overhead_camera')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('logical_camera_topic', '/overhead_logical')
        self.declare_parameter('parts', ['part_1', 'part_2', 'part_3'])
        # 动态发现:逻辑相机视野内任何 part_* 模型自动纳入跟踪表,
        # 持续来料演示(part_4 及以后)不依赖静态 parts 列表
        self.declare_parameter('auto_discover', True)
        self.declare_parameter('discover_limit', 200)   # 跟踪表上限
        self.declare_parameter('stale_timeout', 10.0)   # 已登记件消失多久后剔除
        # 相机安装位姿(必须与 a1_cell_sim 世界模型中的相机一致)
        self.declare_parameter('camera.x', 0.65)
        self.declare_parameter('camera.y', 0.0)
        self.declare_parameter('camera.z', 1.55)
        self.declare_parameter('camera.roll_deg', 0.0)
        self.declare_parameter('camera.pitch_deg', 90.0)
        self.declare_parameter('camera.yaw_deg', 0.0)
        # 分类稳定性
        self.declare_parameter('sample_radius', 14)
        self.declare_parameter('min_ratio', 0.20)
        self.declare_parameter('stable_count', 3)

        gp = lambda k: self.get_parameter(k).value
        self._parts = [str(x) for x in gp('parts')]
        self._radius = int(gp('sample_radius'))
        self._min_ratio = float(gp('min_ratio'))
        self._stable = int(gp('stable_count'))

        self._cam_pos = np.array([
            float(gp('camera.x')), float(gp('camera.y')), float(gp('camera.z'))])
        self._cam_rot = rpy_to_matrix(
            np.radians(float(gp('camera.roll_deg'))),
            np.radians(float(gp('camera.pitch_deg'))),
            np.radians(float(gp('camera.yaw_deg'))))

        self._intrinsics = None
        self._poses: dict[str, np.ndarray] = {}
        # part -> {'votes': deque(类别), 'published': bool}
        self._state = {p: {'votes': deque(maxlen=self._stable),
                           'published': False}
                       for p in self._parts}
        self._auto_discover = bool(gp('auto_discover'))
        self._discover_limit = int(gp('discover_limit'))
        self._stale_timeout = float(gp('stale_timeout'))
        # part -> 最近一次在逻辑相机视野内出现的仿真钟秒(陈旧剔除用)
        self._last_seen: dict[str, float] = {}

        self._pub = self.create_publisher(PartInfo, '/parts/arrival', 10)
        self.create_subscription(
            LogicalCameraImage, str(gp('logical_camera_topic')),
            self._on_logical, 10)
        self.create_subscription(
            CameraInfo, str(gp('camera_info_topic')),
            self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, str(gp('camera_topic')), self._on_image,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'视觉节点就绪: 工件 {self._parts},等待相机图像与逻辑相机位姿 ...')

    # ------------------------------------------------------------- 相机内参
    def _on_camera_info(self, msg: CameraInfo):
        k = msg.k
        self._intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy

    # ------------------------------------------------------------- 逻辑相机
    def _on_logical(self, msg: LogicalCameraImage):
        """工件相对位姿(逻辑相机系) -> 世界系(与安装位姿复合)。

        auto_discover 开启时,视野内新的 part_* 模型自动建跟踪项
        (持续来料 part_4+ 无需静态列表);已登记且消失超过
        stale_timeout 的件从跟踪表剔除(reuse/回收后名字重新出现时
        会以全新状态重新发现)。
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        for model in msg.model:
            name = str(model.name)
            if name not in self._state:
                if not (self._auto_discover
                        and name.startswith('part_')
                        and len(self._state) < self._discover_limit):
                    continue
                self._state[name] = {'votes': deque(maxlen=self._stable),
                                     'published': False}
                self.get_logger().info(f'动态发现工件: {name}')
            self._last_seen[name] = now
            rel = np.array([model.pose.position.x,
                            model.pose.position.y,
                            model.pose.position.z])
            self._poses[name] = self._cam_pos + self._cam_rot @ rel
        # 已发布登记的件若长期离开视野,剔除跟踪表与位姿缓存
        stale = [p for p, st in self._state.items()
                 if st['published']
                 and now - self._last_seen.get(p, now) > self._stale_timeout]
        for p in stale:
            del self._state[p]
            self._poses.pop(p, None)
            self._last_seen.pop(p, None)

    # ------------------------------------------------------------- 图像分类
    def _on_image(self, msg: Image):
        if self._intrinsics is None:
            return
        if msg.encoding == 'rgb8':
            image = cv2.cvtColor(
                np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        else:
            return  # 其他编码暂不处理

        # 遍历动态跟踪表(静态 parts + 自动发现新件);回收剔除后条目
        # 可能消失,用 .get 防御(2026-10-03 修:曾按静态列表直取
        # _state[part_id],回收件被剔除后 KeyError 节点崩溃)
        for part_id in list(self._state):
            state = self._state.get(part_id)
            if state is None or state['published']:
                continue
            point = self._poses.get(part_id)
            if point is None:
                continue
            try:
                u, v = world_to_pixel(point, self._cam_pos, self._cam_rot,
                                      self._intrinsics)
            except ValueError:
                continue
            if not (0 <= u < msg.width and 0 <= v < msg.height):
                self.get_logger().warn(
                    f'{part_id} 投影像素 ({u:.0f},{v:.0f}) 超出图像,'
                    f'请检查相机位姿参数', throttle_duration_sec=5.0)
                continue
            category = classify_region(image, (u, v), self._radius,
                                       min_ratio=self._min_ratio)
            if category is None:
                continue
            votes = state['votes']
            votes.append(category)
            if len(votes) == votes.maxlen and len(set(votes)) == 1:
                self._publish_arrival(part_id, category, point)
                state['published'] = True

    def _publish_arrival(self, part_id: str, category: str, point: np.ndarray):
        msg = PartInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.part_id = part_id
        msg.category = category
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.position.z = float(point[2])
        msg.pose.orientation.w = 1.0
        self._pub.publish(msg)
        self.get_logger().info(f'来料识别完成: {part_id} -> {category}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
