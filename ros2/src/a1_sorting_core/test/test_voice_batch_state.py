import json
import pytest
from a1_sorting_core.voice_batch_plan import validate_candidate,PlanRejected
from a1_sorting_core.voice_batch_state import VoiceBatchState,physical_ready_claim
PLAN=validate_candidate(json.dumps(dict(schema_version=1,action='sort_batch',counts=dict(blue=0,yellow=1,red=1,green=3),priority_first='yellow',proposed_order=['yellow','red','green','green','green'])))
CLAIM=json.dumps(dict(evidence=dict(ready=True,samples=50,stable_span=.25,max_reported_speed=.001,max_difference_speed=.002,max_position_error=.001),controller_result=dict(success=True,status=4,error_code=0),no_active_goal=True))
def start(before=True):
 s=VoiceBatchState()
 if before:s.ready()
 s.accept(s.session_id,'a'*32,True,PLAN,PLAN.sha256,0)
 if not before:s.ready()
 return s

def placed(s,p,c):s.status(p,'arm1' if c in ('yellow','blue') else 'arm2','PART_PLACED',json.dumps(dict(result='PLACED',target_bin='bin_'+c,final_bin='bin_'+c)))
def ready(s,p,c):s.status(p,'arm1' if c in ('yellow','blue') else 'arm2','ARM_READY',CLAIM)
@pytest.mark.parametrize('before',[True,False])
def test_readiness_plan_order_and_no_double_feed(before):
 s=start(before);assert s.reserve(1,True)=='yellow';assert s.reserve(1,True) is None

def test_unconfirmed_plan_and_ready_alone_never_feed():
 s=VoiceBatchState();s.ready();assert s.reserve(1,True) is None
 with pytest.raises(PlanRejected,match='confirmation'):s.accept(s.session_id,'a'*32,False,PLAN,PLAN.sha256,0)
 assert not s.issued

def test_plan_pending_readiness_does_not_lose_first():
 s=VoiceBatchState();s.accept(s.session_id,'a'*32,True,PLAN,PLAN.sha256,0);assert s.reserve(1,True) is None;s.ready();assert s.reserve(2,True)=='yellow'

def test_yellow_needs_placement_and_ready_and_slot_release():
 s=start();s.reserve(1,True);s.arrival('yellow_seen','yellow',2)
 assert s.reserve(3,True) is None
 placed(s,'yellow_seen','yellow');assert s.reserve(4,True) is None
 ready(s,'yellow_seen','yellow');assert s.reserve(5,True) is None
 s.release('yellow_seen','arm1',6);assert s.reserve(7,True)=='red'

def test_exact_five_actual_ids_and_same_bin_recycle_ack_plus_new_logical():
 s=start();seen=[]
 for i,c in enumerate(PLAN.order):
  previous=[p for p,r in s.parts.items() if r['color']==c]
  if previous:
   assert s.reserve(i*10+1,True) is None
   for p in previous:s.recycle(p,i*10+1)
   assert s.reserve(i*10+1,True,(),i*10) is None
   assert s.reserve(i*10+1,True,previous,i*10+2) is None
  assert s.reserve(i*10+3,True,(),i*10+2)==c
  part='actual_camera_id_'+str(700+i);seen.append(part);assert s.arrival(part,c,i*10+4);assert not s.arrival(part,c,i*10+4)
  placed(s,part,c);ready(s,part,c);s.release(part,'arm1' if c=='yellow' else 'arm2',i*10+5)
 assert s.state=='COMPLETED' and len(set(seen))==5 and s.issued==list(PLAN.order)
 assert s.reserve(100,True) is None and all(r['placed'] and r['ready'] and r['released'] for r in s.parts.values())

@pytest.mark.parametrize('failure',['ARM_FAULT','FAILED','PART_REJECTED'])
def test_fault_is_latched_and_late_completed_or_ready_cannot_release(failure):
 s=start();s.reserve(1,True);s.arrival('p','yellow',2);s.status('p','arm1',failure,'x');placed(s,'p','yellow');ready(s,'p','yellow');s.release('p','arm1',3)
 assert s.state=='HALTED' and not s.parts['p']['released'] and s.reserve(4,True) is None

@pytest.mark.parametrize('color',['red','blue','green','unknown'])
def test_visual_mismatch_not_relabelled(color):
 s=start();s.reserve(1,True);assert not s.arrival('actual',color,2);assert s.state=='HALTED'

def test_unrequested_entity_before_plan_faults():
 s=VoiceBatchState();s.observe_entities({'part_1'});assert s.state=='HALTED' and s.reserve(1,True) is None

def test_extra_arrival_faults():
 s=start();s.reserve(1,True);s.arrival('a','yellow',2);s.arrival('b','yellow',3);assert s.state=='HALTED'

@pytest.mark.parametrize('change',[dict(session_id='stale'),dict(request_id='model-generated'),dict(plan_sha256='wrong')])
def test_bad_confirmation_or_restart_nonce(change):
 s=VoiceBatchState();kw=dict(session_id=s.session_id,request_id='a'*32,confirmed=True,plan=PLAN,plan_sha256=PLAN.sha256,now=0);kw.update(change)
 with pytest.raises(PlanRejected):s.accept(**kw)
 assert not s.issued

def test_disconnect_and_duplicate_requests_never_reissue():
 s=start();s.reserve(1,True)
 for rid in ('a'*32,'b'*32):
  with pytest.raises(PlanRejected):s.accept(s.session_id,rid,True,PLAN,PLAN.sha256,2)
 assert s.issued==['yellow']

@pytest.mark.parametrize('phase',['startup','vision','task','recycle'])
def test_wall_timeout_all_stages(phase):
 s=start(False)
 if phase!='startup':s.reserve(1,True)
 if phase in ('task','recycle'):s.arrival('p','yellow',2)
 if phase=='recycle':placed(s,'p','yellow');ready(s,'p','yellow');s.release('p','arm1',3)
 s.check_timeout(1000);assert s.state=='HALTED' and s.reserve(1001,True) is None

@pytest.mark.parametrize('detail',['{}',CLAIM.replace('0.001','2.886'),CLAIM.replace('0.25','Infinity'),CLAIM.replace('true','false'),CLAIM.replace('"status": 4','"status": 6')])
def test_invalid_ready_rejected(detail):
 assert not physical_ready_claim(detail)
 s=start();s.reserve(1,True);s.arrival('p','yellow',2);placed(s,'p','yellow');s.status('p','arm1','ARM_READY',detail);s.release('p','arm1',3);assert s.state=='HALTED' and not s.parts['p']['released']
