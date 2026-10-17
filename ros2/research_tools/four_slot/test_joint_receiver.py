import queue,time,json,gzip
from pathlib import Path
from joint_receiver import JointReceiver
from a1_sorting_core.four_slot_feedback import ReleaseEvidence
from lock_window import LockWindow

class Stamp:sec=1;nanosec=0
class Header:stamp=Stamp()
class Msg:header=Header();position=[.012,.012];name=['l','r']

def bare(capacity):
    r=JointReceiver.__new__(JointReceiver);r.queue=queue.Queue(capacity);r.received=r.dropped=r.highwater=0;r.max_callback=0.;r.fault=None
    return r

def test_ingress_never_replaces_received_messages():
    r=bare(3);messages=[Msg() for _ in range(3)]
    for m in messages:r._receive(m,{})
    assert [r.queue.get()[0] for _ in messages]==messages
    assert r.dropped==0 and r.highwater==3

def test_overflow_is_fault_and_not_silent_overwrite():
    r=bare(1);first=Msg();r._receive(first,{});r._receive(Msg(),{})
    assert r.queue.get()[0] is first and r.dropped==1 and r.fault=='joint_ingress_queue_overflow'

def test_callback_has_no_discovery_or_file_dependency():
    r=bare(10);r._receive(Msg(),{})
    assert r.received==1 # bare receiver has no node, handler or log attribute

def test_true_gap_still_faults():
    m=ReleaseEvidence(['l','r']);m.begin(wall=0)
    m.observe(['l','r'],[.012,.012],1.,0.,'one');m.observe(['l','r'],[.012,.012],1.058,.001,'one')
    assert m.fault=='observation_gap'

def test_stale_not_made_fresh_by_processing():
    m=ReleaseEvidence(['l','r']);m.begin(wall=0);m.observe(['l','r'],[.012,.012],1.,0.,'one')
    assert not m.check(1.1,.501) and m.fault=='stale_feedback'

def test_recorded_unique_callback_window():
    p=Path(__file__).resolve().parents[4]/'calibration_runs/C-20261017T111639-43f307/raw-ros.jsonl.gz'
    w=LockWindow(['arm2_gripper_finger_left_joint','arm2_gripper_finger_right_joint']);passed=False
    with gzip.open(p,'rt') as f:
        for l in f:
            a=json.loads(l)
            if a['kind']!='joint_state' or not 1792207031.197159<=a['wall']<=1792207033.2011728:continue
            m=a['message'];s=m['header']['stamp'];w.observe(m['name'],m['position'],s['sec']+s['nanosec']*1e-9,'recorded-source');passed=passed or w.snapshot()[0]
    assert passed and w.fault is None

def test_future_single_preflight_is_fail_closed():
    from concentrated_acceptance import yellow_preflight,calibration_pass
    assert yellow_preflight() and not calibration_pass(Path('/nonexistent-evidence'))

def test_processing_backlog_does_not_renew_receipt_freshness(tmp_path):
    import threading
    r=bare(3);r.stop=threading.Event();r.log=tmp_path/'ingress.jsonl'
    r.processed=0;r.max_age=0.;r.publishers=[object()];r.discovery_wall=time.monotonic()
    called=[];faults=[];r.handler=lambda *x:called.append(x);r.fault_handler=faults.append
    old=time.monotonic()-.6;r.queue.put((Msg(),{},old,time.time()-.6,old,1));r.stop.set();r._process()
    assert not called and faults==['joint_consumer:joint_processing_stale']
    row=json.loads(r.log.read_text());assert row['receipt_mono']==old and row['process_mono']-old>=.6
