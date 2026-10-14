"""规划场景小工具(ROS1 移植新增):代替组长版本里 Gazebo 世界的 create / remove / 逻辑相机真值。

工件、桌子都作为 MoveIt 规划场景里的碰撞体;"抓住"用 attach,"放下"用 detach。
"""
import rospy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject,
                             ObjectColor, PlanningScene,
                             PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

# 与组长 a1_cell_sim/scripts/spawn_parts.py 的 COLOR_RGB 相同
COLOR_RGB = {
    'red': (0.85, 0.10, 0.10),
    'green': (0.10, 0.75, 0.15),
    'blue': (0.15, 0.35, 0.90),
    'yellow': (0.95, 0.80, 0.10),
}


def make_box(obj_id, frame, size, xyz):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = obj_id
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = list(size)
    p = Pose()
    p.position.x, p.position.y, p.position.z = xyz
    p.orientation.w = 1.0
    co.primitives.append(prim)
    co.primitive_poses.append(p)
    co.operation = CollisionObject.ADD
    return co


def make_color(obj_id, rgb, alpha=1.0):
    oc = ObjectColor()
    oc.id = obj_id
    oc.color.r, oc.color.g, oc.color.b = rgb
    oc.color.a = alpha
    return oc


class SceneClient(object):
    """/apply_planning_scene + /get_planning_scene 的同步封装。"""

    def __init__(self, timeout=60.0):
        rospy.wait_for_service('/apply_planning_scene', timeout)
        rospy.wait_for_service('/get_planning_scene', timeout)
        self._apply = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self._get = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

    @staticmethod
    def diff():
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        return ps

    def apply(self, ps):
        return self._apply(ps).success

    def add_objects(self, objects, colors=()):
        ps = self.diff()
        ps.world.collision_objects.extend(objects)
        ps.object_colors.extend(colors)
        return self.apply(ps)

    def remove_objects(self, ids, attach_link='panda_hand'):
        """先解除附着(若在手上),再从世界删除。"""
        if not ids:
            return True
        ps = self.diff()
        for i in ids:
            aco = AttachedCollisionObject()
            aco.link_name = attach_link
            aco.object.id = i
            aco.object.operation = CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(aco)
        try:
            self.apply(ps)
        except rospy.ServiceException:
            pass
        ps = self.diff()
        for i in ids:
            co = CollisionObject()
            co.id = i
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        return self.apply(ps)

    def get_scene(self):
        comp = PlanningSceneComponents()
        comp.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                           | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                           | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
                           | PlanningSceneComponents.OBJECT_COLORS)
        return self._get(comp).scene

    def world_positions(self, prefix=''):
        """世界中(未附着)物体的中心位置 {id: (x, y, z)}。
        Noetic 的 CollisionObject 有 pose 字段,primitive_poses 相对于它;老版本没有,两种都兼容。"""
        out = {}
        for co in self.get_scene().world.collision_objects:
            if prefix and not co.id.startswith(prefix):
                continue
            base = getattr(co, 'pose', None)
            bx = base.position.x if base else 0.0
            by = base.position.y if base else 0.0
            bz = base.position.z if base else 0.0
            if co.primitive_poses:
                pp = co.primitive_poses[0].position
                out[co.id] = (bx + pp.x, by + pp.y, bz + pp.z)
            else:
                out[co.id] = (bx, by, bz)
        return out

    def colors(self):
        return {oc.id: (oc.color.r, oc.color.g, oc.color.b) for oc in self.get_scene().object_colors}
