from types import SimpleNamespace as N
import threading
import pytest
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from a1_sorting_core.motion_client import MotionClient
from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode,GraspError,FatalGraspError
from a1_sorting_core.relay_sequencer import RelaySequencer
from a1_sorting_core.task_dispatcher_node import TaskDispatcherNode

class Logger:
 def __getattr__(self,k):return lambda *a:None
class Future:
 def __init__(self,value):self.value=value
 def done(self):return True
 def result(self):return self.value

def client(status,code):
 m=object.__new__(MotionClient);m._node=N(get_logger=lambda:Logger())
 traj=JointTrajectory();traj.joint_names=['j'];traj.points=[JointTrajectoryPoint()]
 wrapped=N(status=status,result=N(error_code=code))
 handle=N(accepted=True,get_result_async=lambda:Future(wrapped))
 m._traj_client=N(send_goal_async=lambda goal:Future(handle))
 return m,N(planned_trajectory=N(joint_trajectory=traj))
@pytest.mark.parametrize('status,code,ok',[(GoalStatus.STATUS_SUCCEEDED,0,True),(GoalStatus.STATUS_SUCCEEDED,-5,False),(GoalStatus.STATUS_ABORTED,-5,False),(GoalStatus.STATUS_CANCELED,0,False)])
def test_return_needs_terminal_success_and_successful_error_code(status,code,ok):
 m,result=client(status,code);assert m._execute_split(result,'return_standby',60,lambda:True)==ok
 assert m.last_controller_result['success']==ok and not m._own_goal_inflight

def test_standby_no_settled_fallback_on_abort():
 m,result=client(GoalStatus.STATUS_ABORTED,-5);assert not m._execute_split(result,'standby',60,lambda:True)

def test_controller_reject_cannot_be_ready():
 m,result=client(4,0);m._traj_client=N(send_goal_async=lambda goal:Future(N(accepted=False)))
 assert not m._execute_split(result,'return_standby',60,lambda:True)
 assert m._own_goal_inflight and m.last_controller_result is None

def test_controller_timeout_is_unresolved_and_cancel_requested(monkeypatch):
 m,result=client(4,0);cancel=[];never=N(done=lambda:False)
 handle=N(accepted=True,get_result_async=lambda:never,cancel_goal_async=lambda:cancel.append(True))
 m._traj_client=N(send_goal_async=lambda goal:Future(handle));clock=iter([0.,61.])
 monkeypatch.setattr('a1_sorting_core.motion_client.time.monotonic',lambda:next(clock))
 assert not m._execute_split(result,'return_standby',60,lambda:False)
 assert cancel and m._own_goal_inflight

@pytest.mark.parametrize('returned',[True,False,'exception'])
def test_executor_placement_survives_fault_but_slot_only_released_by_ready(returned):
 q=RelaySequencer(per_arm_slots=True,require_arm_ready=True);q.enqueue('p');q.try_dispatch('arm1')
 events=[]
 def emit(part,state,detail):
  events.append(state)
  if state=='PART_PLACED':q.note_outcome('p','arm1',state)
  if state=='ARM_FAULT':q.arm_fault('p','arm1')
  if state=='ARM_READY':q.arm_ready('p','arm1')
 def impl(part):n._terminal_status=('t',part,'done','bin_blue');emit(part,'PART_PLACED','bin_blue')
 def ret():
  if returned=='exception':raise RuntimeError('planning_rejected')
  return returned
 n=N(_arm_fault=None,_busy=True,_pending=[],_terminal_status=None,_command_lock=threading.Lock(),_run_task_impl=impl,
     _try_return_standby=ret,_await_placement_result=lambda p:True,_current_ready_evidence=lambda:{'ready':True},_emit_arm_event=emit,_publish_zone_clear=lambda p,r:events.append('ZONE_CLEAR'),
     _flush_terminal_status=lambda:events.append('LEGACY_COMPLETED'),get_logger=lambda:Logger())
 PickPlaceExecutorNode._run_task(n,N(part_id='p'))
 assert events[0]=='PART_PLACED'
 if returned is True:assert events==['PART_PLACED','ZONE_CLEAR','LEGACY_COMPLETED','ARM_READY'] and q.slot('arm1') is None
 else:assert events==['PART_PLACED','ARM_FAULT'] and q.slot('arm1')=='p' and n._busy

def test_initial_standby_failure_does_not_set_ready_or_enter_pick():
 def fail(*a,**kw):raise GraspError('controller_aborted')
 n=N(_seq=0,_init_motion=lambda:None,_wedge_detect=False,_at_standby=False,
     _standby=(0,0,1),_down_pose=lambda *a:None,_plan_execute=fail,get_logger=lambda:Logger())
 with pytest.raises(FatalGraspError):PickPlaceExecutorNode._run_task_impl(n,N(part_id='p',category='blue'))
 assert not n._at_standby and n._arm_fault.startswith('initial_standby_failed')

def test_dispatcher_fault_latches_before_future_dispatch_and_keeps_outcome():
 q=RelaySequencer(per_arm_slots=True,require_arm_ready=True);q.enqueue('p');q.try_dispatch('arm1');q.note_outcome('p','arm1','PART_PLACED')
 stops=[]
 n=N(_note_sim=lambda s:None,_seq=q,_lock=threading.Lock(),_halted=False,
     _destroy_spawn_timer=lambda:stops.append('spawn'),_destroy_next_timer=lambda:stops.append('next'),
     _destroy_relay_timer=lambda:stops.append('relay'),_pub_belt=N(publish=lambda m:stops.append(m.data)),get_logger=lambda:Logger())
 TaskDispatcherNode._on_status(n,N(header=N(stamp=None),state='ARM_FAULT',part_id='p',arm='arm1',detail='unstable'))
 assert n._halted and q.slot('arm1')=='p' and q.zone_arm=='arm1' and stops[-1]==0
 response=N(accepted=True,message='');assert not TaskDispatcherNode._on_sort_task(n,None,response).accepted


def test_ready_and_validated_outcome_can_arrive_in_either_order():
 q=RelaySequencer(per_arm_slots=True,require_arm_ready=True);q.enqueue('p');q.try_dispatch('arm1')
 assert not q.arm_ready('p','arm1')[0] and q.slot('arm1')=='p'
 assert q.has_ready('p','arm1') and q.note_outcome('p','arm1','PART_PLACED')
 assert q.arm_ready('p','arm1')[0] and q.slot('arm1') is None


def test_missing_placement_confirmation_faults_even_if_arm_returned():
 events=[]
 n=N(_arm_fault=None,_busy=True,_pending=[],_terminal_status=None,_command_lock=threading.Lock(),
     _run_task_impl=lambda p:None,_try_return_standby=lambda:True,
     _await_placement_result=lambda p:False,_emit_arm_event=lambda p,s,d:events.append(s),get_logger=lambda:Logger())
 PickPlaceExecutorNode._run_task(n,N(part_id='p'))
 assert events==['ARM_FAULT'] and n._busy


@pytest.mark.parametrize('xy,expected',[( (.3,.2),'PART_PLACED'),((1.,1.),'PART_REJECTED'),(None,'PART_REJECTED')])
def test_only_recorder_ground_truth_confirms_part_placed(xy,expected):
 from a1_sorting_core.result_recorder_node import ResultRecorderNode
 from a1_sorting_core.bins import Bin
 from a1_interfaces.msg import TaskStatus
 rows=[];acks=[];stamp=TaskStatus().header.stamp
 n=N(_finished=set(),_poses={'p':xy} if xy is not None else {},_categories={'p':'blue'},
     _bins=[Bin('bin_blue',.3,.2,.06,.085)],_failed_parts=set(),_recycle_delay=0.,
     _book=N(record=lambda *a,**k:rows.append(a)),_pub_placement_result=N(publish=lambda m:acks.append(m)),
     get_clock=lambda:N(now=lambda:N(to_msg=lambda:stamp)),get_logger=lambda:Logger(),
     _publish_counts=lambda:None,_publish_bin_markers=lambda:None,_batch_finished=lambda:False)
 msg=TaskStatus();msg.part_id='p';msg.arm='arm1';msg.task_id='t';msg.state='PLACEMENT_FINISHED';msg.detail='bin_blue'
 ResultRecorderNode._on_status(n,msg);assert len(rows)==1 and acks[0].state==expected
 msg.state='COMPLETED';ResultRecorderNode._on_status(n,msg);assert len(rows)==1


def test_conflicting_outcomes_and_late_zone_clear_fail_closed():
 q=RelaySequencer(per_arm_slots=True,require_arm_ready=True);q.enqueue('p');q.try_dispatch('arm1')
 assert q.note_outcome('p','arm1','PART_PLACED')
 assert not q.note_outcome('p','arm1','FAILED') and q.is_faulted('arm1')
 assert not q.release_zone('p','arm1') and not q.arm_ready('p','arm1')[0]
 assert q.slot('arm1')=='p' and q.zone_arm=='arm1'

@pytest.mark.parametrize('accepted,status,code',[(False,4,1),(True,6,-1),(True,4,-1)])
def test_planning_rejection_or_moveit_error_never_executes_controller(accepted,status,code):
 m=object.__new__(MotionClient)
 m._group='arm1';m._vel=.5;m._acc=.5;m._split=True
 m._wait_slot_free=lambda **k:None;m._note_outcome=lambda *a,**k:None
 wrapped=N(status=status,result=N(error_code=N(val=code)))
 m._client=N(send_goal_async=lambda g:Future(N(accepted=accepted,get_result_async=lambda:Future(wrapped))))
 m._execute_split=lambda *a:pytest.fail('planner failure must not execute trajectory')
 assert not m._move_to_joints_once(['j'],[0],'return_standby',60,Logger(),settle_check=False)


def test_executor_duplicate_or_faulted_commands_do_not_start_new_work():
 n=N(_command_lock=threading.Lock(),_arm_fault=None,_command_ids=set(),_busy=True,_pending=[],get_logger=lambda:Logger())
 p=N(part_id='p');PickPlaceExecutorNode._on_command(n,p);PickPlaceExecutorNode._on_command(n,p)
 assert len(n._pending)==1
 n._arm_fault='stale_state';PickPlaceExecutorNode._on_command(n,N(part_id='q'));assert len(n._pending)==1 and 'q' not in n._command_ids


def test_placement_ack_timeout_is_bounded_and_latches_fault(monkeypatch):
 from a1_sorting_core.state_machine import TaskState
 n=N(_terminal_status=('t',None,TaskState.COMPLETED,'bin_blue'),_active_task_id='t',_placement_ack={})
 clock=iter([0.,1.]);monkeypatch.setattr('a1_sorting_core.pick_place_executor_node.time.monotonic',lambda:next(clock))
 assert not PickPlaceExecutorNode._await_placement_result(n,N(part_id='p'),timeout=.1)
 assert n._arm_fault=='placement_result_unconfirmed'


def test_late_placement_ack_cannot_reuse_stale_ready_snapshot():
 events=[]
 n=N(_arm_fault=None,_busy=True,_pending=[],_terminal_status=None,_run_task_impl=lambda p:None,
     _try_return_standby=lambda:True,_await_placement_result=lambda p:True,
     _current_ready_evidence=lambda:{'ready':False,'reason':'stale_joint_receipt'},
     _emit_arm_event=lambda p,s,d:events.append(s),get_logger=lambda:Logger())
 PickPlaceExecutorNode._run_task(n,N(part_id='p'))
 assert events==['ARM_FAULT'] and n._arm_fault.startswith('ready_changed_before_publication')


def test_rejected_placement_ack_never_becomes_arm_ready():
 from a1_sorting_core.state_machine import TaskState
 n=N(_terminal_status=('t',None,TaskState.COMPLETED,'bin_blue'),_active_task_id='t',
     _placement_ack={('p','t'):'PART_REJECTED'})
 assert not PickPlaceExecutorNode._await_placement_result(n,N(part_id='p'))
 assert n._arm_fault=='placement_rejected_requires_inspection'


def test_permanent_task_exception_stops_without_new_recovery_motion():
 events=[]
 def fail(p):raise RuntimeError('unknown_pose')
 n=N(_arm_fault=None,_busy=True,_pending=[],_terminal_status=None,_run_task_impl=fail,
     _try_return_standby=lambda:pytest.fail('unknown state must stop, not start another motion'),
     _emit_arm_event=lambda p,s,d:events.append(s),get_logger=lambda:Logger())
 PickPlaceExecutorNode._run_task(n,N(part_id='p'))
 assert events==['FAILED','ARM_FAULT'] and n._busy
