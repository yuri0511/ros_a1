import copy
import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene,AllowedCollisionEntry
from a1_sorting_core.held_payload_scene import HeldPayloadScene,SceneSyncError,allow_pair,matrix_key
from a1_sorting_core.retained_scene_sync import observed_object

def backend():
    scene=PlanningScene();scene.world.collision_objects=[observed_object('other',[1.,0.,.8],[0.,0.,0.,1.],1.,1.)]
    def apply(q):
        for a in q.scene.robot_state.attached_collision_objects:
            oid=a.object.id;scene.robot_state.attached_collision_objects=[x for x in scene.robot_state.attached_collision_objects if x.object.id!=oid]
            if a.object.operation!=a.object.REMOVE:
                scene.robot_state.attached_collision_objects.append(copy.deepcopy(a));scene.world.collision_objects=[x for x in scene.world.collision_objects if x.id!=oid]
        for o in q.scene.world.collision_objects:
            scene.world.collision_objects=[x for x in scene.world.collision_objects if x.id!=o.id]+[copy.deepcopy(o)]
        if q.scene.allowed_collision_matrix.entry_names:scene.allowed_collision_matrix=copy.deepcopy(q.scene.allowed_collision_matrix)
        return True
    return scene,apply

def attached():
    scene,apply=backend();model=HeldPayloadScene(apply,lambda:copy.deepcopy(scene));p=Pose();p.orientation.w=1.;p.position.z=1.
    obs=dict(position=[.1,.2,.8],quaternion=[0.,0.,0.,1.],stamp=1.)
    model.attach('part_1',obs,p,1.,1.1,'tool0',['finger_L','finger_R'],clamp_verified=True)
    return model,scene,obs

def test_full_size_real_relative_pose_and_no_ghost():
    m,s,o=attached();a=s.robot_state.attached_collision_objects[0]
    assert list(a.object.primitives[0].dimensions)==[.05]*3 and a.object.primitive_poses[0].position.z==pytest.approx(-.2)
    assert [o.id for o in s.world.collision_objects]==['retained_other']
    assert a.touch_links==['finger_L','finger_R'];m.assert_attached()

def test_release_restores_measured_world_and_other_obstacle():
    m,s,o=attached();o=dict(o,position=[.3,.4,.745],stamp=2.);m.release(o,2.1,gripper_released=True)
    assert m.state=='WORLD' and not s.robot_state.attached_collision_objects
    assert {x.id for x in s.world.collision_objects}=={'retained_other','retained_part_1'}
    assert next(x for x in s.world.collision_objects if x.id=='retained_part_1').primitive_poses[0].position.y==.4

def test_support_preconditions_cannot_be_skipped():
    m,s,o=attached()
    with pytest.raises(SceneSyncError):m.permit_support('belt_belt_collision',contact_verified=False,clamp_verified=True,direction='depart')
    with pytest.raises(SceneSyncError):m.permit_support('belt_belt_collision',contact_verified=True,clamp_verified=True,direction='land')

def test_one_pair_only_and_restore_after_actual_separation():
    m,s,o=attached();s.allowed_collision_matrix.entry_names=['robot','wall'];s.allowed_collision_matrix.entry_values=[AllowedCollisionEntry(enabled=[True,False]),AllowedCollisionEntry(enabled=[False,True])];key=matrix_key(s.allowed_collision_matrix)
    m.permit_support('belt_belt_collision',contact_verified=True,clamp_verified=True,direction='depart')
    a=s.allowed_collision_matrix;ix={n:i for i,n in enumerate(a.entry_names)}
    assert a.entry_values[ix['retained_part_1']].enabled[ix['belt_belt_collision']] and not a.entry_values[ix['retained_part_1']].enabled[ix['wall']]
    with pytest.raises(SceneSyncError):m.restore_support()
    m.restore_support(separation_verified=True);assert matrix_key(s.allowed_collision_matrix)==key

@pytest.mark.parametrize('tf_stamp,now,clamped',[(.8,1.1,True),(1.,1.3,True),(1.,1.1,False)])
def test_missing_clamp_or_stale_aligned_pose_refuses(tf_stamp,now,clamped):
    s,apply=backend();m=HeldPayloadScene(apply,lambda:copy.deepcopy(s));p=Pose();p.orientation.w=1.
    with pytest.raises(SceneSyncError):m.attach('part_1',dict(position=[0.,0.,.8],quaternion=[0.,0.,0.,1.],stamp=1.),p,tf_stamp,now,'tool0',['finger'],clamp_verified=clamped)
    assert not s.robot_state.attached_collision_objects

def test_scene_readback_missing_or_rpc_rejection_stops():
    s,_=backend();m=HeldPayloadScene(lambda q:False,lambda:s);p=Pose();p.orientation.w=1.
    with pytest.raises(SceneSyncError):m.attach('part_1',dict(position=[0.,0.,.8],quaternion=[0.,0.,0.,1.],stamp=1.),p,1.,1.1,'tool0',['finger'],clamp_verified=True)
    assert m.state=='UNCERTAIN'

def test_fault_retains_held_body_and_prevents_false_release():
    m,s,o=attached();m.fault('controller aborted');assert s.robot_state.attached_collision_objects and m.state=='FAULT'
    with pytest.raises(SceneSyncError):m.release(o,1.1,gripper_released=True)
