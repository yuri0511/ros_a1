import copy,math,pytest
from a1_sorting_core.four_slot_feedback import ReleaseEvidence
from a1_sorting_core.retained_scene_sync import RetainedSceneSync,SceneSyncError,camera_observation
from a1_sorting_core.four_slot_capacity import SlotBank,CapacityError
from geometry_msgs.msg import Pose

def monitor():
    m=ReleaseEvidence(('L','R'));m.begin(wall=0);m.observe(['L','R'],[.011,.011],1,.01,'one');assert m.begin_exit(1,.01);return m

def test_release_feedback_spans_exit_and_never_commands():
    m=monitor()
    for i in range(1,20):m.observe(['L','R'],[.010,.012],1+i*.002,.01+i*.002,'one')
    assert m.check(1.04,.05)
    assert not m.finish(1.04,.05,geometry_clearance_verified=False)
    assert m.fault=='exit_clearance_unverified'

@pytest.mark.parametrize('q',[(.007,.011),(.011,.015),(-.003,.0037)])
def test_either_finger_violation_latches(q):
    m=monitor();m.observe(['L','R'],q,1,.02,'one');assert m.fault=='opening_envelope_exceeded'
    m.observe(['L','R'],[.011,.011],1.002,.03,'one');assert not m.check(1.002,.03)

def test_duplicate_does_not_refresh_age_or_hide_bad_payload():
    m=monitor();m.observe(['L','R'],[.011,.011],1,.6,'one');assert m.samples==1
    assert not m.check(1,.6);assert m.fault=='stale_feedback'

@pytest.mark.parametrize('names,q,stamp,source',[(('L','L'),(.011,.011),1.002,'one'),(('L','R'),(.011,.011),.9,'one'),(('L','R'),(.011,.011),1.002,'other'),(('L','R'),(.011,.011),1.2,'one')])
def test_invalid_stream_blocked(names,q,stamp,source):
    m=monitor();m.observe(names,q,stamp,.02,source);assert not m.check(stamp,.02)

def test_inventory_real_pose_ack_sequence_and_fifth_refusal():
    bins={'bin_green':{'slots':[{'id':f'green/{i}','xyz':[.9+i*.09,-.3,.745]} for i in range(4)]}}
    bank=SlotBank(bins);world={};counts=[]
    def apply(objects):
        for o in objects:world[o.id]=copy.deepcopy(o)
        return True
    sync=RetainedSceneSync(apply,lambda:list(world.values()));obs={}
    for i in range(4):
        part=f'part_{i}';sid=bank.reserve('bin_green',part);pos=list(bank.slots[sid]['xyz']);pos[0]+=.001
        bank.occupied(sid,part,pos,1,1);obs[part]={'position':pos,'quaternion':[0,0,math.sin(.1),math.cos(.1)],'stamp':1}
        ack=sync.sync(bank.slots,obs,1);counts.append(len(ack['objects']))
        assert list(world['retained_'+part].primitives[0].dimensions)==[.05]*3
        assert world['retained_'+part].primitive_poses[0].position.x==pos[0]
    assert counts==[1,2,3,4]
    with pytest.raises(CapacityError):bank.reserve('bin_green','fifth')
    with pytest.raises(SceneSyncError):sync.sync(bank.slots,{},1)
    assert sync.ack is None and len(world)==4

@pytest.mark.parametrize('mode',['reject','mismatch','stale','missing'])
def test_scene_failure_has_no_ack(mode):
    inventory={'s':{'state':'OCCUPIED','part':'p'}};obs={'p':{'position':[0.,0.,.745],'quaternion':[0.,0.,0.,1.],'stamp':1.}}
    s=RetainedSceneSync(lambda objects:mode!='reject',lambda:[])
    if mode=='missing':obs={}
    with pytest.raises(SceneSyncError):s.sync(inventory,obs,2 if mode=='stale' else 1)
    assert s.ack is None

def test_camera_pose_composition_preserves_orientation():
    camera=Pose();camera.position.z=2.;camera.orientation.w=1.
    part=Pose();part.position.x=.3;part.position.z=-1.;part.orientation.z=math.sin(.2);part.orientation.w=math.cos(.2)
    observed=camera_observation(camera,part,4)
    assert observed['position']==[.3,0.,1.]
    assert observed['quaternion']==[0.,0.,math.sin(.2),math.cos(.2)]

@pytest.mark.parametrize('status,code',[(6,-5),(5,0),(4,-5)])
def test_four_slot_controller_failure_never_uses_settled_fallback(status,code):
    from test_arm_lifecycle import client
    m,result=client(status,code);m._node._four_slot=True
    assert not m._execute_split(result,'four_slot_settle_bottom',60,lambda:True)

def test_four_slot_fault_cancels_active_goal():
    from types import SimpleNamespace as N
    from test_arm_lifecycle import client,Future
    m,result=client(4,0);m._node._four_slot=True;m._node._arm_fault='release_feedback:opening_envelope_exceeded'
    cancelled=[];handle=N(accepted=True,get_result_async=lambda:N(done=lambda:False),cancel_goal_async=lambda:cancelled.append(True))
    m._traj_client=N(send_goal_async=lambda goal:Future(handle))
    assert not m._execute_split(result,'four_slot_rise',60,lambda:True)
    assert cancelled and m._own_goal_inflight

def test_released_current_part_is_obstacle_before_placement_ack():
    world={}
    def apply(objects):
        world.update({o.id:o for o in objects});return True
    sync=RetainedSceneSync(apply,lambda:list(world.values()))
    inventory={'s':{'state':'RELEASED_UNCONFIRMED','part':'current'}}
    obs={'current':{'position':[.3,.2,.745],'quaternion':[0.,0.,0.,1.],'stamp':1.}}
    assert sync.sync(inventory,obs,1)['objects']==['retained_current']
    assert inventory['s']['state']=='RELEASED_UNCONFIRMED'

def test_unknown_inventory_state_is_not_silently_omitted():
    sync=RetainedSceneSync(lambda _:True,lambda:[])
    with pytest.raises(SceneSyncError):sync.sync({'s':{'state':'INVALID','part':'p'}},{},1)
    assert sync.ack is None
