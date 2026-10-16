import pytest
from a1_sorting_core.physical_ready import PhysicalReadyMonitor
from a1_sorting_core.relay_sequencer import RelaySequencer

def window(**kw):
 m=PhysicalReadyMonitor(**kw);m.begin(10.)
 for k in range(151):m.observe(['j','arm2_j'],[0.,k*.02],[0.,10.],1.+k*.002,10.+k*.002)
 return m

def evaluate(m,**kw):
 args=dict(names=['j'],targets=[0.],now_sim=1.3,now_wall=10.3,controller_ok=True,no_active_goal=True);args.update(kw);return m.evaluate(**args)

def test_success_stable_own_arm_ignores_other_arm_motion():assert evaluate(window())['ready']
@pytest.mark.parametrize('v',[.31,2.885672,-2.885672])
def test_success_but_reported_motion_blocks(v):
 m=window();m.observe(['j'],[0.],[v],1.302,10.302);assert not evaluate(m,now_sim=1.302,now_wall=10.302)['ready']

def test_position_difference_blocks_even_zero_reported_speed():
 m=window();m.observe(['j'],[.1],[0.],1.302,10.302);assert not evaluate(m,now_sim=1.302,now_wall=10.302)['ready']
@pytest.mark.parametrize('fields',[{'controller_ok':False},{'no_active_goal':False},{'now_wall':11.},{'now_sim':2.},{'now_sim':1.0},{'targets':[1.]}])
def test_missing_action_or_freshness_or_position(fields):assert not evaluate(window(),**fields)['ready']
@pytest.mark.parametrize('names,pos,vel,stamp',[(['j','j'],[0,0],[0,0],1.302),(['arm2_j'],[0],[0],1.302),(['j'],[0],[],1.302),(['j'],[float('nan')],[0],1.302),(['j'],[0],[0],1.1),(['j'],[0],[0],1.3)])
def test_invalid_or_wrong_joint_or_time(names,pos,vel,stamp):
 m=window();m.observe(names,pos,vel,stamp,10.302);assert not evaluate(m,now_sim=1.302,now_wall=10.302)['ready']

def test_stable_window_is_continuous_not_single_sample():
 m=PhysicalReadyMonitor();m.begin(10);m.observe(['j'],[0],[0],1,10);assert not evaluate(m,now_sim=1,now_wall=10)['ready']

def seq():
 q=RelaySequencer(per_arm_slots=True,require_arm_ready=True);q.enqueue('p1');q.enqueue('p2');assert q.try_dispatch('arm1')=='p1';return q

def test_zone_outcome_ready_are_separate_and_deduplicated():
 q=seq();assert q.release_zone('p1','arm1');assert q.slot('arm1')=='p1'
 assert q.try_dispatch('arm2')=='p2';assert q.note_outcome('p1','arm1','PART_PLACED')
 assert not q.on_terminal('p1','arm1','COMPLETED')[0];assert q.slot('arm1')=='p1'
 accepted,_,outcome=q.arm_ready('p1','arm1');assert accepted and outcome=='COMPLETED'
 assert q.slot('arm2')=='p2' and q.zone_arm=='arm2'
 assert not q.arm_ready('p1','arm1')[0];assert not q.enqueue('p1')

def test_ready_without_outcome_and_wrong_arm_cannot_free():
 q=seq();assert not q.arm_ready('p1','arm1')[0];assert not q.note_outcome('p1','arm2','PART_PLACED');assert q.slot('arm1')=='p1'

def test_fault_keeps_slot_zone_and_blocks_late_ready_until_full_restart():
 q=seq();assert q.note_outcome('p1','arm1','PART_PLACED');assert q.arm_fault('p1','arm1')
 assert not q.arm_ready('p1','arm1')[0] and q.slot('arm1')=='p1' and q.zone_arm=='arm1'
 assert 'arm1' not in q.idle_arms() and q.try_dispatch('arm1') is None
 # Explicit fresh process/world is the supported recovery; no automatic latch clear.
 fresh=RelaySequencer(per_arm_slots=True,require_arm_ready=True);assert fresh.enqueue('new_batch_p1');assert fresh.try_dispatch('arm1')=='new_batch_p1'

def test_permanent_task_failure_stays_latched_even_after_return_ready():
 q=seq();q.note_outcome('p1','arm1','FAILED');assert q.slot('arm1')=='p1';assert not q.arm_ready('p1','arm1')[0]


def test_duplicate_stamp_resets_window_without_becoming_a_permanent_startup_fault():
 m=window();m.observe(['j'],[0],[0],1.3,10.301)
 assert not evaluate(m,now_wall=10.301)['ready'] and m.duplicate_stamps==1
 for k in range(151):m.observe(['j'],[0],[0],1.302+k*.002,10.302+k*.002)
 assert evaluate(m,now_sim=1.602,now_wall=10.602)['ready']

def test_clock_rollback_stays_latched_even_after_new_stable_data():
 m=window();m.observe(['j'],[0],[0],.5,10.301)
 for k in range(151):m.observe(['j'],[0],[0],2+k*.002,11+k*.002)
 assert not evaluate(m,now_sim=2.3,now_wall=11.3)['ready']


@pytest.mark.parametrize('offset,ready',[(.009,True),(.011,False)])
def test_standby_uses_existing_joint_goal_tolerance_not_compensation_gate(offset,ready):
 m=PhysicalReadyMonitor();m.begin(10)
 for k in range(151):m.observe(['j'],[offset],[0],1+k*.002,10+k*.002)
 assert evaluate(m)['ready']==ready
