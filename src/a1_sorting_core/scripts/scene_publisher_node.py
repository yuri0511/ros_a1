#!/usr/bin/env python3
"""静态规划场景发布节点(ROS1 移植自组长 scene_publisher_node.py)。

组长版:以 2s 周期向 /collision_object 重发 ADD;因 UR5e 网格与桌面误碰撞,
最终 _SCENE 清空(只保留框架)。
ROS1/Panda 版:只发布一张桌子(顶面 z=0,避开 Panda 底座),通过
/apply_planning_scene 同步写入;再按 publish_period 周期重发(幂等,
覆盖 move_group 重启的情况,保留组长的思路)。
"""
import rospy

from a1_sorting_core.scene_utils import SceneClient, make_box, make_color

# (id, size[x,y,z], 中心[x,y,z])。x 从 0.15 开始,不与 panda_link0 相交
_SCENE = [
    ('table', (0.75, 1.2, 0.04), (0.55, 0.0, -0.022)),   # 顶面 z = -0.002
]


class ScenePublisherNode(object):

    def __init__(self):
        self.frame = rospy.get_param('/a1/frame', 'panda_link0')
        period = float(rospy.get_param('~publish_period', 10.0))
        self.scene = SceneClient()
        self.publish_all()
        if period > 0:
            rospy.Timer(rospy.Duration(period), lambda _e: self.publish_all())
        rospy.loginfo('静态规划场景发布器就绪:%d 个碰撞体,周期 %.1fs', len(_SCENE), period)

    def publish_all(self):
        objs = [make_box(i, self.frame, s, c) for i, s, c in _SCENE]
        colors = [make_color(i, (0.55, 0.45, 0.35)) for i, _s, _c in _SCENE]
        self.scene.add_objects(objs, colors)


if __name__ == '__main__':
    rospy.init_node('scene_publisher')
    ScenePublisherNode()
    rospy.spin()
