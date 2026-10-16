import copy,csv,json,threading,time
from types import SimpleNamespace as N
from pathlib import Path
import pytest
from a1_interfaces.msg import PartInfo
from a1_sorting_core.task_dispatcher_node import TaskDispatcherNode
from a1_sorting_core.voice_batch_state import VoiceBatchState
from a1_sorting_core.voice_batch_results import validate_batch_results
from a1_sorting_core.bins import ResultBook
from a1_sorting_core.voice_batch_plan import PlanRejected
from test_voice_batch_state import PLAN
class Pub:
 def __init__(self):self.messages=[]
 def publish(self,msg):self.messages.append(copy.deepcopy(msg))
def node():
 n=object.__new__(TaskDispatcherNode);n._voice_mode=True;n._voice=VoiceBatchState();n._lock=threading.Lock();n._halted=False;n._voice_pub=Pub();n._pub_spawn=Pub();n._pub_belt=Pub();n._feed_inflight=False;n._startup_rgb_wall=time.monotonic();n._voice_logical_wall=time.monotonic();n._voice_visible=set();n._spawned=0;n._feed_counts=dict.fromkeys(('blue','yellow','red','green'),0);n._belt_speed=.1;n._belt=N(start=lambda:None)
 n._seq=N(busy_parts=lambda:set(),zone_arm=None);n._spawn_timer=n._next_timer=n._relay_timer=n._settle_timer=None
 n.get_clock=lambda:N(now=lambda:N(nanoseconds=10**9));n.get_logger=lambda:N(info=lambda *a:None,error=lambda *a:None)
 return n

def test_real_dispatcher_entry_waits_until_confirmed_and_atomic_no_double_spawn():
 n=node();n._voice.ready();n._request_spawn_and_start_belt();assert not n._pub_spawn.messages
 n._voice.accept(n._voice.session_id,'a'*32,True,PLAN,PLAN.sha256,time.monotonic())
 threads=[threading.Thread(target=n._request_spawn_and_start_belt) for _ in range(4)]
 for t in threads:t.start()
 for t in threads:t.join()
 assert [m.data for m in n._pub_spawn.messages]==['yellow'] and n._spawned==1

def test_halted_never_spawns_or_accepts_legacy_service():
 n=node();n._voice.halt('test_fault')
 with n._lock:n._voice_flush_locked()
 n._request_spawn_and_start_belt();assert not n._pub_spawn.messages
 r=n._on_sort_task(N(),N());assert not r.accepted and 'exclusive' in r.message

def test_camera_stale_defers_without_losing_request():
 n=node();n._voice.accept(n._voice.session_id,'a'*32,True,PLAN,PLAN.sha256,time.monotonic());n._voice.ready();n._startup_rgb_wall=0;n._request_spawn_and_start_belt();assert not n._pub_spawn.messages
 n._startup_rgb_wall=time.monotonic();n._request_spawn_and_start_belt();assert len(n._pub_spawn.messages)==1

def make_csv(tmp_path):
 p=tmp_path/'actual.csv';book=ResultBook(str(p),['bin_'+c for c in ('blue','yellow','red','green')])
 for i,c in enumerate(PLAN.order):book.record('actual_'+str(i),c,'bin_'+c,'bin_'+c,'PLACED',arm='arm1' if c=='yellow' else 'arm2',phase='sort' if c=='yellow' else 'relay')
 return p,book

def test_original_resultbook_dynamic_five_csv_counts(tmp_path):
 p,book=make_csv(tmp_path);r=validate_batch_results(p,PLAN);assert r['rows']==5 and r['bin_counts']==dict(bin_yellow=1,bin_red=1,bin_green=3);assert book.counts['bin_blue']==0

@pytest.mark.parametrize('mutation',['extra','duplicate','wrong_bin','wrong_color','bad_result'])
def test_csv_rejects_extra_duplicate_wrong_result(tmp_path,mutation):
 p,book=make_csv(tmp_path)
 with p.open() as f:rows=list(csv.DictReader(f))
 if mutation=='extra':rows.append(dict(rows[-1],part_id='extra'))
 elif mutation=='duplicate':rows[-1]['part_id']=rows[0]['part_id']
 elif mutation=='wrong_bin':rows[-1]['final_bin']='bin_blue'
 elif mutation=='wrong_color':rows[-1]['category']='blue'
 else:rows[-1]['result']='FAILED'
 with p.open('w') as f:w=csv.DictWriter(f,fieldnames=ResultBook.HEADER);w.writeheader();w.writerows(rows)
 with pytest.raises(PlanRejected):validate_batch_results(p,PLAN)

def test_primary_plan_is_rejected_by_unresolved_green3_execution_gate():
 from a1_sorting_core.voice_batch_safety import ensure_geometry_review
 with pytest.raises(PlanRejected,match='green_third_slot'):ensure_geometry_review(PLAN)
