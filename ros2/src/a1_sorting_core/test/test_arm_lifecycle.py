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
 n=N(_arm_fault=None,_busy=True,_pending=[],_terminal_status=None,_run_task_impl=impl,
     _try_return_standby=ret,_emit_arm_event=emit,_publish_zone_clear=lambda p,r:events.append('ZONE_CLEAR'),
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
