"""No ROS/Gazebo processes: verify request order and bounded failure paths."""
import subprocess
from types import SimpleNamespace
from shutdown import stop_requests

def test_pause_ack_is_requested_before_ros_rpc(tmp_path):
    calls=[]
    def runner(cmd,**kwargs):
        calls.append((cmd,kwargs['timeout']))
        return SimpleNamespace(returncode=0,stdout='data: true\n' if cmd[0]=='gz' else 'success=True',stderr='')
    records=stop_requests(tmp_path,{},runner)
    assert [c[0][0] for c in calls]==['gz','ros2']
    assert [r['step'] for r in records]==['pause_world','halt_new_feed']
    assert all(r['acknowledged'] for r in records)

def test_unacknowledged_pause_is_recorded_and_ros_halt_still_attempted(tmp_path):
    calls=[]
    def runner(cmd,**kwargs):
        calls.append(cmd[0])
        if cmd[0]=='gz':raise subprocess.TimeoutExpired(cmd,kwargs['timeout'])
        return SimpleNamespace(returncode=0,stdout='success=True',stderr='')
    records=stop_requests(tmp_path,{},runner)
    assert calls==['gz','ros2'] and not records[0]['acknowledged'] and records[1]['acknowledged']
    assert (tmp_path/'stop-timeline.jsonl').exists()

def test_both_commands_missing_still_bounded_and_recorded(tmp_path):
    calls=[]
    def runner(cmd,**kwargs):
        calls.append((cmd[0],kwargs['timeout']))
        raise FileNotFoundError(cmd[0])
    records=stop_requests(tmp_path,{},runner)
    assert calls==[('gz',4),('ros2',3)]
    assert len(records)==2 and not any(r['acknowledged'] for r in records)


def test_repeated_stop_preserves_each_attempt(tmp_path):
    def runner(cmd,**kwargs):
        return SimpleNamespace(returncode=0,stdout='data: true' if cmd[0]=='gz' else 'success=True',stderr='')
    first=stop_requests(tmp_path,{},runner);second=stop_requests(tmp_path,{},runner)
    assert first[0]['attempt']!=second[0]['attempt']
    assert all((tmp_path/r['evidence']).exists() for r in first+second)
    assert len((tmp_path/'stop-timeline.jsonl').read_text().splitlines())==4
    assert all(r['acknowledged'] and r['duration_monotonic_s']>=0 for r in first+second)
