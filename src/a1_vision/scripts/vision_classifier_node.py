#!/usr/bin/env python3
"""视觉分类节点(可选拓展,ROS1 移植自组长 a1_vision/vision_classifier_node.py)。

组长版:订阅 Gazebo 俯视 RGB 相机 + 逻辑相机,把工件世界坐标投影到像素,
在像素窗口里做 HSV 颜色分类(color_classifier.classify_region),连续 stable_count
帧一致后发布 /parts/arrival 登记来料。

ROS1/Panda 版:demo.launch 没有 Gazebo,也就没有相机图像。这里做一个"虚拟相机":
- 位置:从 MoveIt 规划场景读取来料区内 part_* 的位置(相当于组长的逻辑相机真值);
- 像素:按规划场景里工件的显示颜色合成一小块 BGR 图像并加高斯噪声(相当于相机拍到的像素);
- 分类:原样调用组长的 color_classifier.classify_region(HSV 阈值完全相同);
- 连续 stable_count 次一致后发布 /parts/arrival(PartInfo,category = 识别结果)。
说明:颜色判定逻辑是组长的真实视觉代码,但"图像"是合成的,答辩时要如实说明。
使用:roslaunch a1_sorting_core sorting_demo.launch vision:=true
     (spawn_parts 自动改为 publish_arrival:=false,来料只由本节点登记)
"""
import numpy as np
import rospy
from std_msgs.msg import String

from a1_interfaces.msg import PartInfo
from a1_sorting_core.batch_protocol import BATCH_TOPIC, parse_event
from a1_sorting_core.scene_utils import SceneClient
from a1_vision.color_classifier import classify_region


class VisionClassifierNode(object):

    def __init__(self):
        gp = rospy.get_param
        self.frame = gp('/a1/frame', 'panda_link0')
        self.prefix = gp('/a1/part_prefix', 'part_')
        self.slots = list(zip(gp('/a1/feed_slots_x'), gp('/a1/feed_slots_y')))
        self.feed_radius = float(gp('~feed_radius', 0.06))     # 离来料槽多近算"在视野内"
        self.sample_radius = int(gp('~sample_radius', 7))      # 与组长 vision_params.yaml 相同
        self.min_ratio = float(gp('~min_ratio', 0.20))
        self.stable_count = int(gp('~stable_count', 3))
        self.noise_std = float(gp('~noise_std', 8.0))          # 合成像素噪声(0-255)
        self.rate = float(gp('~rate', 2.0))
        self.scene = SceneClient()
        self.pub = rospy.Publisher('/parts/arrival', PartInfo, queue_size=10)
        self.history = {}      # part_id -> 最近几次分类结果
        self.registered = set()
        self._batch_id = ''
        rospy.Subscriber(BATCH_TOPIC, String, self._on_batch)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        rospy.loginfo('视觉分类节点就绪(虚拟相机),stable_count=%d', self.stable_count)

    def _on_batch(self, msg):
        kind, batch_id = parse_event(msg.data)
        if kind in ('reset', 'begin'):
            self.registered.clear()
            self.history.clear()
        if kind == 'begin':
            self._batch_id = batch_id

    def in_feed_area(self, x, y):
        return any((x - sx) ** 2 + (y - sy) ** 2 <= self.feed_radius ** 2 for sx, sy in self.slots)

    def synth_patch(self, rgb):
        size = 2 * self.sample_radius + 6
        bgr = np.array([rgb[2], rgb[1], rgb[0]], dtype=float) * 255.0
        img = np.tile(bgr, (size, size, 1)) + np.random.normal(0.0, self.noise_std, (size, size, 3))
        return np.clip(img, 0, 255).astype(np.uint8), (size // 2, size // 2)

    def tick(self, _evt):
        try:
            positions = self.scene.world_positions(self.prefix)
            colors = self.scene.colors()
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(5.0, '读取规划场景失败: %s', e)
            return
        visible = {p: xyz for p, xyz in positions.items() if self.in_feed_area(xyz[0], xyz[1])}
        # 离开视野(被抓走/放进料区)的件取消登记,下一轮重新生成时可以再次登记
        for p in list(self.registered):
            if p not in visible:
                self.registered.discard(p)
                self.history.pop(p, None)
        for p, xyz in visible.items():
            if p in self.registered or p not in colors:
                continue
            img, center = self.synth_patch(colors[p])
            cat = classify_region(img, center, self.sample_radius, min_ratio=self.min_ratio)
            hist = self.history.setdefault(p, [])
            hist.append(cat)
            del hist[:-self.stable_count]
            if len(hist) == self.stable_count and cat is not None and all(h == cat for h in hist):
                msg = PartInfo()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame
                msg.part_id, msg.category = p, cat
                msg.batch_id = self._batch_id
                msg.ground_truth_category = ''
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
                msg.pose.orientation.w = 1.0
                self.pub.publish(msg)
                self.registered.add(p)
                rospy.loginfo('视觉登记: %s -> %s', p, cat)


if __name__ == '__main__':
    rospy.init_node('vision_classifier')
    VisionClassifierNode()
    rospy.spin()
