import json
from types import SimpleNamespace as N
import pytest
from a1_sorting_core.voice_batch_plan import validate_candidate,audit_utterance,PlanRejected
from a1_sorting_core.voice_batch_state import VoiceBatchState
from a1_sorting_core.voice_batch_safety import ensure_geometry_review
from a1_sorting_core.voice_demo_audit import BatchEventAudit
from a1_sorting_core.voice_batch_results import validate_batch_results
from a1_sorting_core.bins import ResultBook
from test_voice_batch_state import CLAIM,placed,ready
from test_voice_batch_integration import node

PLAN=validate_candidate(json.dumps(dict(schema_version=1,action='sort_batch',counts=dict(blue=0,yellow=1,red=1,green=2),priority_first='yellow',proposed_order=['yellow','red','green','green'])))
TEXT='开始分拣，一个黄色、一个红色、两个绿色，先完成黄色。'
def test_new_four_text_is_audited_and_not_old_five():
    assert audit_utterance(TEXT,PLAN);ensure_geometry_review(PLAN);assert PLAN.total==4
    with pytest.raises(PlanRejected):audit_utterance('开始分拣，三个绿色，一个红色，一个黄色，先分拣黄色。',PLAN)

def test_original_recorder_format_is_complete_at_four_not_eight(tmp_path):
    path=tmp_path/'four.csv';book=ResultBook(str(path),['bin_'+c for c in ('blue','yellow','red','green')])
    for i,c in enumerate(PLAN.order):book.record('real_'+str(i),c,'bin_'+c,'bin_'+c,'PLACED',arm='arm1' if c=='yellow' else 'arm2',phase='sort' if c=='yellow' else 'relay')
    result=validate_batch_results(path,PLAN)
    assert result['rows']==4 and result['unique_parts']==4 and result['bin_counts']==dict(bin_yellow=1,bin_red=1,bin_green=2)

@pytest.mark.parametrize('before',[True,False])
def test_real_dispatcher_spawn_entry_whole_four_serial_protocol(before):
    n=node();s=n._voice
    if before:s.ready()
    n._request_spawn_and_start_belt();assert not n._pub_spawn.messages
    s.accept(s.session_id,'a'*32,True,PLAN,PLAN.sha256,0)
    if not before:
        n._request_spawn_and_start_belt();assert not n._pub_spawn.messages;s.ready()
    for i,c in enumerate(PLAN.order):
        prior=[p for p,r in s.parts.items() if r['color']==c]
        if prior:
            n._request_spawn_and_start_belt();assert len(n._pub_spawn.messages)==i
            for p in prior:s.recycle(p,1.)
            n._voice_visible=set(prior);n._request_spawn_and_start_belt();assert len(n._pub_spawn.messages)==i
            n._voice_visible=set()
        n._request_spawn_and_start_belt();assert [m.data for m in n._pub_spawn.messages]==list(PLAN.order[:i+1])
        part='camera_actual_'+str(i);s.arrival(part,c,i+1)
        n._feed_inflight=False # original ZONE_CLEAR clears feeder, NOT arm slot.
        n._seq=N(busy_parts=lambda:{part},zone_arm=None)
        placed(s,part,c);n._request_spawn_and_start_belt();assert len(n._pub_spawn.messages)==i+1
        ready(s,part,c);n._request_spawn_and_start_belt();assert len(n._pub_spawn.messages)==i+1
        s.release(part,'arm1' if c=='yellow' else 'arm2',i+2)
        n._seq=N(busy_parts=lambda:set(),zone_arm=None)
    for _ in range(5):n._request_spawn_and_start_belt()
    assert s.state=='COMPLETED' and n._spawned==4

@pytest.mark.parametrize('state',['ARM_FAULT','FAILED','PART_REJECTED'])
def test_four_fault_before_yellow_finish_blocks_all_following(state):
    n=node();s=n._voice;s.accept(s.session_id,'a'*32,True,PLAN,PLAN.sha256,0);s.ready();n._request_spawn_and_start_belt();s.arrival('y','yellow',1)
    s.status('y','arm1',state,'fault');placed(s,'y','yellow');ready(s,'y','yellow');s.release('y','arm1',2)
    n._feed_inflight=False
    with n._lock:n._voice_flush_locked()
    n._request_spawn_and_start_belt();assert n._halted and n._spawned==1 and not s.parts['y']['released']

def test_monitor_detects_premature_red_and_missing_ready():
    a=BatchEventAudit(PLAN,'a'*32)
    with pytest.raises(PlanRejected):a.event(dict(event='SPAWN_REQUEST',index=1,color='yellow'))
    a.event(dict(event='VOICE_PLAN',request_id='a'*32,plan=PLAN.as_dict(),plan_sha256=PLAN.sha256))
    a.event(dict(event='SPAWN_REQUEST',index=1,color='yellow'));a.event(dict(event='PartInfo',part='y',color='yellow'))
    a.event(dict(event='PART_PLACED',part='y'))
    with pytest.raises(PlanRejected):a.event(dict(event='SPAWN_REQUEST',index=2,color='red'))
    with pytest.raises(PlanRejected):a.event(dict(event='SLOT_RELEASED',part='y'))

def test_recycle_needs_new_sim_source_stamp_not_only_new_receipt_time():
    s=VoiceBatchState();s.ready();s.accept(s.session_id,'a'*32,True,PLAN,PLAN.sha256,0)
    for i,c in enumerate(PLAN.order[:3]):
        assert s.reserve(i+1,True)==c;s.arrival(str(i),c,i+1);placed(s,str(i),c);ready(s,str(i),c);s.release(str(i),'arm1' if c=='yellow' else 'arm2',i+2)
    s.recycle('2',10,50.)
    for source_stamp in (None,49.,50.):assert s.reserve(11,True,(),11,source_stamp) is None
    assert s.reserve(11,True,(),11,50.001)=='green'

@pytest.mark.parametrize('home',[None,.20])
def test_no_new_feed_when_pusher_not_confirmed_at_original_home_threshold(home):
    n=node();n._voice.ready();n._voice.accept(n._voice.session_id,'a'*32,True,PLAN,PLAN.sha256,0);n._startup_pusher=home
    n._request_spawn_and_start_belt();assert not n._pub_spawn.messages

def test_independent_audit_accepts_four_only_with_matching_ready_and_slots():
    a=BatchEventAudit(PLAN,'a'*32);a.event(dict(event='VOICE_PLAN',request_id='a'*32,plan=PLAN.as_dict(),plan_sha256=PLAN.sha256))
    for i,c in enumerate(PLAN.order):
        if i==3:a.event(dict(event='PART_RECYCLED',part='p2'))
        a.event(dict(event='SPAWN_REQUEST',index=i+1,color=c));a.event(dict(event='PartInfo',part='p'+str(i),color=c))
        for kind in ('PART_PLACED','ARM_READY','SLOT_RELEASED'):a.event(dict(event=kind,part='p'+str(i)))
        a.ready('p'+str(i),CLAIM)
    a.event(dict(event='COMPLETED'));assert a.finish()['total']==4
    with pytest.raises(PlanRejected):a.event(dict(event='SPAWN_REQUEST',index=5,color='green'))
