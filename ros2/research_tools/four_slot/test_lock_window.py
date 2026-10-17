from lock_window import LockWindow

def feed(w,n=61,dt=.002,moving=False):
    for i in range(n):w.observe(['l','r'],[i*.0001 if moving else .001,.001],i*dt,'source')

def test_all_callbacks_survive_slow_reader():
    w=LockWindow(['l','r']);feed(w)
    ok,rows,fault=w.snapshot();assert ok and len(rows)>=50 and fault is None

def test_duplicates_do_not_advance():
    w=LockWindow(['l','r'])
    for _ in range(100):w.observe(['l','r'],[.001,.001],1.,'source')
    assert not w.snapshot()[0] and len(w.snapshot()[1])==1

def test_motion_still_rejected():
    w=LockWindow(['l','r']);feed(w,moving=True);assert not w.snapshot()[0]

def test_pause_still_not_ready():
    w=LockWindow(['l','r']);feed(w,n=10);assert not w.snapshot()[0]

def test_reversal_rejected():
    w=LockWindow(['l','r']);feed(w);w.observe(['l','r'],[.001,.001],0.,'source')
    assert w.snapshot()[2]=='locked_finger_time_reversed'

def test_source_change_rejected():
    w=LockWindow(['l','r']);feed(w);w.observe(['l','r'],[.001,.001],.122,'other')
    assert w.snapshot()[2]=='locked_finger_source_changed'

def test_gap_rejected():
    w=LockWindow(['l','r']);feed(w);w.observe(['l','r'],[.001,.001],.2,'source')
    assert w.snapshot()[2]=='locked_finger_source_gap'
