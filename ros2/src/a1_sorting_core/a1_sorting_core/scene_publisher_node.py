"""静态规划场景发布节点(pipeline 模式):把工作站固定几何作为
CollisionObject 发布给 move_group 的规划场景监控。

为什么需要:planning scene 默认只知道机器人自身链接(SRDF/URDF),世界里的
桌面/传送带/挡停板/箱子/推板/相机立柱对规划器不可见——KDL 的 IK 分支随机性
会让 Cartesian 目标 PTP 选中"臂身翻转扫过工作区"的分支,物理上撞上带/挡板
(规划器不知情,照常放行;联调实证 carry 段卡死与工件被剜飞)。发布静态
碰撞体后,该类分支在规划期即被拒(COLLISION_CHECK_FAILED),重试会落在
无碰撞分支。

几何与 a1_cell_sim/worlds/sorting_cell.sdf 保持一致(契约 §2;改动世界
几何时必须同步本表):
- table   1.2×0.8×0.72   @(0.65, 0, 0.36)
- belt    0.35×0.10×0.06 @(0.325,0, 0.75)   顶面 0.78
- plate   0.01×0.14×0.02 @(0.47, 0, 0.79)    顶面 0.80
- bin_A/B/out 外廓 0.14×0.19×0.065 @z 0.7525(箱口 0.785)
- pusher  0.02×0.14×0.05 @(0.13, 0, 0.805)  回位态(推进行程不含障碍)
- pole    r0.015×0.83    @(0.65, 0, 1.135)

以 2s 周期重发 ADD(幂等,覆盖 move_group 晚于本节点启动的时序)。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


# (id, size[x,y,z 或 cylinder 参数], pose[x,y,z], is_cylinder)
# 注:桌面(1.2×0.8×0.72)不发布——待机构型下臂部碰撞网格与桌面盒
# 相交(历史标定从未把桌面纳入场景),发布会导致全状态
# START_STATE_IN_COLLISION;旧世界验证 6/6 也从未物理撞桌,略去无害。
# 场景碰撞体整体停用(2026-10-01 联调定论):历史验证链(6/6)在盲规划下
# 运行;加入碰撞体后 FCL 的网格级保守接触(如推板 vs upper_arm 假阳性)
# 反而封死唯一可用分支,而它防的卡死已由高度/间隙/到位闸根治。保留节点
# 与发布框架,后续需要时往 _SCENE 填回条目即可。
_SCENE = []


class ScenePublisherNode(Node):

    def __init__(self):
        super().__init__('scene_publisher')
        self.declare_parameter('four_slot_trial_permit','')
        self.declare_parameter('four_slot_mode',False)
        self.declare_parameter('four_slot_world','')
        self._four_slot_objects=[]
        if self.get_parameter('four_slot_mode').value:
            from .four_slot_scene import collision_objects
            self._four_slot_objects=collision_objects(self.get_parameter('four_slot_world').value)
        self.declare_parameter('publish_period', 2.0)
        period = float(self.get_parameter('publish_period').value)
        self._pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self._timer = self.create_timer(period, self._publish_all)
        self.get_logger().info(
            f'静态规划场景发布器就绪:{len(_SCENE)+len(self._four_slot_objects)} 个碰撞体,'
            f'周期 {period}s(/collision_object)')

    def _publish_all(self):
        for msg in self._four_slot_objects:self._pub.publish(msg)
        for obj_id, size, xyz, is_cyl in _SCENE:
            msg = CollisionObject()
            msg.header.frame_id = 'world'
            msg.id = obj_id
            msg.operation = CollisionObject.ADD
            prim = SolidPrimitive()
            prim.type = (SolidPrimitive.CYLINDER if is_cyl
                         else SolidPrimitive.BOX)
            if is_cyl:
                prim.dimensions = [size[1] / 2.0, size[0]]  # [height, radius]
            else:
                prim.dimensions = list(size)
            msg.primitives.append(prim)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = xyz
            pose.orientation.w = 1.0
            msg.primitive_poses.append(pose)
            self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
