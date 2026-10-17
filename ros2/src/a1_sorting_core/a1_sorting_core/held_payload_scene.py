"""Real 50 mm payload lifecycle. ACK and full readback before any motion.

This is MoveIt planning state, distinct from Gazebo set_pose following. World
pose and TF must refer to the same fresh simulation timestamp. No position fixes.
"""
import copy,math,time
import numpy as np
from moveit_msgs.msg import AttachedCollisionObject,CollisionObject,PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene
from .retained_scene_sync import observed_object,SceneSyncError
from .attach_helper import pose_to_mat,mat_to_pose

EPS=64*np.finfo(float).eps

def matrix_key(m):
    names=sorted(m.entry_names);ix={n:i for i,n in enumerate(m.entry_names)}
    return (tuple(names),tuple(tuple(m.entry_values[ix[a]].enabled[ix[b]] for b in names) for a in names),tuple(sorted(zip(m.default_entry_names,m.default_entry_values))))

def allow_pair(base,body,support):
    from moveit_msgs.msg import AllowedCollisionEntry
    if body==support or not body.startswith('retained_part_'):raise SceneSyncError('support_pair_identity_invalid')
    if base.default_entry_names:raise SceneSyncError('support_default_ACM_requires_review')
    out=copy.deepcopy(base)
    for name in (body,support):
        if name not in out.entry_names:
            for row in out.entry_values:row.enabled.append(False)
            out.entry_names.append(name);out.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(out.entry_names)))
    i,j=[out.entry_names.index(x) for x in (body,support)];out.entry_values[i].enabled[j]=out.entry_values[j].enabled[i]=True
    return out

class HeldPayloadScene:
    def __init__(self,apply,read,emit=lambda **kw:None):
        self.apply,self.read,self.emit=apply,read,emit;self.held=None;self.state='WORLD';self.base=None;self.support=None
    def _ack(self,request):
        if self.apply(request) is not True:raise SceneSyncError('payload_scene_not_acknowledged')
        return self.read()
    def attach(self,part,observation,gripper_pose,tf_stamp,now,link,touch_links,*,clamp_verified):
        if self.held is not None or not clamp_verified:raise SceneSyncError('payload_not_uniquely_clamped')
        if not part.startswith('part_') or not 0<=now-observation['stamp']<=.2 or abs(tf_stamp-observation['stamp'])>.05:raise SceneSyncError('payload_pose_TF_not_fresh_or_aligned')
        world=observed_object(part,observation['position'],observation['quaternion'],observation['stamp'],now)
        before=self.read();oid=world.id
        if any(o.object.id==oid for o in before.robot_state.attached_collision_objects):raise SceneSyncError('payload_already_attached')
        # Original full cube, transformed using the real TF at the observation stamp.
        M=np.linalg.inv(pose_to_mat(gripper_pose))@pose_to_mat(world.primitive_poses[0])
        attached=AttachedCollisionObject();attached.link_name=link;attached.touch_links=list(touch_links);attached.object=copy.deepcopy(world);attached.object.header.frame_id=link;attached.object.primitive_poses=[mat_to_pose(M)]
        # ADD as attached atomically removes an identically named world object.
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;req.scene.robot_state.attached_collision_objects=[attached]
        self.held=attached;self.state='UNCERTAIN'
        scene=self._ack(req);found=[o for o in scene.robot_state.attached_collision_objects if o.object.id==oid]
        if len(found)!=1 or any(o.id==oid for o in scene.world.collision_objects):raise SceneSyncError('payload_duplicate_or_missing_identity')
        actual=found[0]
        if actual.link_name!=link or actual.object.header.frame_id!=link or actual.touch_links!=list(touch_links) or list(actual.object.primitives[0].dimensions)!=[.05]*3:raise SceneSyncError('payload_attachment_readback_changed')
        if not np.allclose(pose_to_mat(actual.object.pose)@pose_to_mat(actual.object.primitive_poses[0]),M,atol=EPS,rtol=0):raise SceneSyncError('payload_transform_readback_changed')
        before_other={o.id:str(o) for o in before.world.collision_objects if o.id!=oid};after_other={o.id:str(o) for o in scene.world.collision_objects}
        if before_other!=after_other:raise SceneSyncError('payload_changed_other_obstacles')
        self.held=copy.deepcopy(actual);self.state='ATTACHED';self.emit(event='PAYLOAD_ATTACHED',part=part,observation=observation,tf_stamp=tf_stamp,relative_matrix=M.tolist());return self.held
    def permit_support(self,support,*,contact_verified,clamp_verified,direction):
        if self.state!='ATTACHED' or not contact_verified or not clamp_verified or direction!='depart':raise SceneSyncError('support_departure_not_verified')
        if self.base is not None:raise SceneSyncError('support_permission_already_active')
        self.base=copy.deepcopy(self.read().allowed_collision_matrix);self.support=support
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.allowed_collision_matrix=allow_pair(self.base,self.held.object.id,support)
        scene=self._ack(req)
        if matrix_key(scene.allowed_collision_matrix)!=matrix_key(req.scene.allowed_collision_matrix):raise SceneSyncError('support_ACM_readback_failed')
        self.emit(event='PAYLOAD_SUPPORT_SCOPED',pair=[self.held.object.id,support])
    def restore_support(self,*,separation_verified=False,world_paused=False):
        if self.base is None:return
        if not (separation_verified or world_paused):raise SceneSyncError('support_restore_phase_unconfirmed')
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.allowed_collision_matrix=copy.deepcopy(self.base)
        scene=self._ack(req)
        if matrix_key(scene.allowed_collision_matrix)!=matrix_key(self.base):raise SceneSyncError('support_ACM_restore_failed')
        self.base=None;self.emit(event='PAYLOAD_SUPPORT_RESTORED')
    def release(self,observation,now,*,gripper_released):
        if self.state!='ATTACHED' or not gripper_released or self.base is not None:raise SceneSyncError('payload_release_not_verified')
        part=self.held.object.id.removeprefix('retained_');world=observed_object(part,observation['position'],observation['quaternion'],observation['stamp'],now)
        remove=AttachedCollisionObject();remove.link_name=self.held.link_name;remove.object.id=world.id;remove.object.operation=CollisionObject.REMOVE
        req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.robot_state.is_diff=True;req.scene.robot_state.attached_collision_objects=[remove];req.scene.world.collision_objects=[world]
        self.state='UNCERTAIN';scene=self._ack(req);found=[o for o in scene.world.collision_objects if o.id==world.id]
        if len(found)!=1 or any(o.object.id==world.id for o in scene.robot_state.attached_collision_objects):raise SceneSyncError('payload_release_identity_failed')
        if not np.allclose(pose_to_mat(found[0].pose)@pose_to_mat(found[0].primitive_poses[0]),pose_to_mat(world.primitive_poses[0]),atol=EPS,rtol=0):raise SceneSyncError('payload_release_pose_changed')
        self.emit(event='PAYLOAD_WORLD_RESTORED',part=part,observation=observation);self.held=None;self.state='WORLD'
    def assert_attached(self):
        if self.state!='ATTACHED':raise SceneSyncError('payload_attachment_unconfirmed')
        s=self.read();oid=self.held.object.id
        if len([x for x in s.robot_state.attached_collision_objects if x.object.id==oid])!=1 or any(o.id==oid for o in s.world.collision_objects):raise SceneSyncError('payload_scene_lost_or_duplicated')
    def fault(self,reason):
        # Do not pretend a still-held object was released; keep the collision body.
        self.state='FAULT';self.emit(event='PAYLOAD_FAULT_HOLD_NO_NEW_MOTION',reason=reason)

class RosHeldPayloadScene(HeldPayloadScene):
    def __init__(self,node,emit):
        self.node=node;self.a=node.create_client(ApplyPlanningScene,'/apply_planning_scene');self.r=node.create_client(GetPlanningScene,'/get_planning_scene')
        super().__init__(self._apply,self._read,emit)
    def _call(self,c,req):
        if not c.wait_for_service(timeout_sec=1):raise SceneSyncError('payload_scene_service_missing')
        f=c.call_async(req);end=time.monotonic()+2
        while not f.done() and time.monotonic()<end:time.sleep(.001)
        if not f.done():f.cancel();raise SceneSyncError('payload_scene_timeout')
        return f.result()
    def _apply(self,req):return self._call(self.a,req).success
    def _read(self):
        q=GetPlanningScene.Request();q.components.components=PlanningSceneComponents.ROBOT_STATE|PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        return self._call(self.r,q).scene
