import sys,json,time
from pathlib import Path
import pytest
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1];ROOT=WS.parents[1]
sys.path.insert(0,str(HERE))
from run_yellow_single import task_record,commands
from yellow_task import load_task
from a1_sorting_core.voice_batch_state import VoiceBatchState
from a1_sorting_core.voice_batch_plan import validate_candidate,PlanRejected
from a1_sorting_core.four_slot_capacity import load_motion_candidate,CapacityError
from a1_sorting_core.four_slot_trial import load_trial,TrialError,gripper_bottoms

def test_new_yellow_task_identity_and_scope(tmp_path):
 a=task_record();b=task_record();assert a['request_id']!=b['request_id']
 p=tmp_path/'task.json';p.write_text(json.dumps(a));_,plan=load_task(p);assert plan.counts==(0,1,0,0) and plan.order==('yellow',)
 a['plan']['counts']['red']=1;p.write_text(json.dumps(a))
 with pytest.raises(PlanRejected):load_task(p)

def test_false_motion_flag_cannot_be_flipped_by_missing_permit():
 cfg=WS/'src/a1_sorting_core/config/four_slot_candidate.json'
 assert json.loads(cfg.read_text())['motion_validated'] is False
 with pytest.raises(CapacityError):load_motion_candidate(cfg)
 with pytest.raises(FileNotFoundError):load_motion_candidate(cfg,'/no-permit')

def test_gui_world_and_overlay_wiring(tmp_path):
 import yaml
 cmd=commands(tmp_path,Path('/not-a-real-permit'));d=yaml.safe_load((tmp_path/'single_overlay.yaml').read_text())
 assert 'demo_parts:=1' in cmd and 'auto_start:=false' in cmd and 'gazebo_gui:=true' in cmd and 'world_file:=four_slot_1ms.sdf' in cmd
 p=d['/pick_place_executor']['ros__parameters'];assert p['four_slot_mode'] and p['split_plan_execute']
 assert d['/pick_place_relay']['ros__parameters']['four_slot_mode'] is False
 assert Path(d['/scene_publisher']['ros__parameters']['four_slot_world']).name=='four_slot_1ms.sdf'
 assert p['four_slot_trial_permit']=='/not-a-real-permit'

def test_yellow_geometry_exit_original_envelope():
 cfg=json.loads((WS/'src/a1_sorting_core/config/four_slot_candidate.json').read_text());slot=cfg['bins']['bin_yellow']['slots'][0]
 from geometry import SUFFIXES
 joints=dict(zip(SUFFIXES,slot['waypoints']['carry']));joints.update(gripper_finger_left_joint=.014,gripper_finger_right_joint=.014)
 bottoms=gripper_bottoms((ROOT/'audit/robot-static.urdf').read_text(),joints)
 assert min(bottoms.values())>.815

@pytest.mark.parametrize('event',['ARM_FAULT','FAILED','PART_REJECTED'])
def test_one_yellow_confirmation_and_fault_latch(event):
 p=validate_candidate(json.dumps(task_record()['plan']));s=VoiceBatchState(retain_capacity=4)
 assert s.reserve(0,True) is None and s.issued==[]
 s.hardware_ready=True
 with pytest.raises(PlanRejected):s.accept(s.session_id,'a'*32,False,p,p.sha256,0)
 s.accept(s.session_id,'a'*32,True,p,p.sha256,0);assert s.reserve(1,True)=='yellow'
 s.arrival('real-camera-id','yellow',2);s.status('real-camera-id','arm1',event,'fault');assert s.reserve(3,True) is None and s.state=='HALTED'

def test_one_yellow_completed_never_requests_second():
    # Explicit protocol mock; this claim is not Gazebo physical evidence.
    p=validate_candidate(json.dumps(task_record()['plan']));s=VoiceBatchState(retain_capacity=4);s.ready();s.accept(s.session_id,'b'*32,True,p,p.sha256,0)
    assert s.reserve(1,True)=='yellow';assert s.arrival('MOCK_ONLY_ID','yellow',2)
    s.status('MOCK_ONLY_ID','arm1','PART_PLACED',json.dumps(dict(result='PLACED',target_bin='bin_yellow',final_bin='bin_yellow')))
    assert s.reserve(3,True) is None
    claim=json.dumps(dict(evidence=dict(ready=True,samples=50,stable_span=.25,max_reported_speed=.001,max_difference_speed=.002,max_position_error=.001),controller_result=dict(success=True,status=4,error_code=0),no_active_goal=True))
    s.status('MOCK_ONLY_ID','arm1','ARM_READY',claim);s.release('MOCK_ONLY_ID','arm1',4)
    assert s.state=='COMPLETED' and s.reserve(5,True) is None and s.issued==['yellow']

def test_payload_review_gap_keeps_B_blocked():
    from concentrated_acceptance import yellow_preflight
    assert any('payload' in s for s in yellow_preflight())

def test_gui_child_failure_cannot_crash_supervisor(tmp_path,monkeypatch):
    from types import SimpleNamespace
    import gui_evidence
    monkeypatch.setattr(gui_evidence.subprocess,'run',lambda *a,**k:SimpleNamespace(returncode=1,stdout='',stderr='X drawable vanished'))
    assert gui_evidence.confirm(tmp_path,'isolated-test') is False
    assert 'vanished' in (tmp_path/'gui-capture-error.log').read_text()

def test_gui_capture_timeout_is_closed_gate(tmp_path,monkeypatch):
    import gui_evidence
    def timeout(*a,**kw):raise gui_evidence.subprocess.TimeoutExpired('capture',5)
    monkeypatch.setattr(gui_evidence.subprocess,'run',timeout)
    assert gui_evidence.confirm(tmp_path,'isolated-test') is False

@pytest.mark.parametrize('planner',['PTP','LIN'])
def test_motion_request_preserves_explicit_planner_without_executing(planner):
    from types import SimpleNamespace as N
    from a1_sorting_core.motion_client import MotionClient
    class Future:
        def done(self):return True
        def result(self):return N(accepted=False)
    seen=[];m=object.__new__(MotionClient);m._group='ur_manipulator';m._vel=.2;m._acc=.2;m._split=True;m._wait_slot_free=lambda **k:None
    m._client=N(send_goal_async=lambda goal:(seen.append(goal) or Future()))
    log=N(error=lambda *a:None)
    assert not m._move_to_joints_once(['j'],[0.],'test-only',1,log,planner_id=planner)
    assert seen[0].request.planner_id==planner and seen[0].planning_options.plan_only
