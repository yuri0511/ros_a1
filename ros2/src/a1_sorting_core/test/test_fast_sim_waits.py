"""Safety of event shortening: original verifier, adequate samples, fresh camera."""
import threading
from concurrent.futures import Future
from a1_sorting_core.pick_place_executor_node import _FollowSampler
from a1_sorting_core.grasp_verifier import GraspVerifier
from a1_sorting_core.motion_client import MotionClient


def sample_fixture():
    s = _FollowSampler(lambda: None, lambda: None)
    s._gripper = [(i*.002, 0.) for i in range(40)]
    s._part = [(i*.002, 0.) for i in range(40)]
    s._sample_times = [i*.05 for i in range(40)]
    s._source_stamps = [i//2*.1 for i in range(40)]
    return s


def test_completed_motion_with_sufficient_fresh_evidence_can_finish():
    s = sample_fixture()
    assert s.wait_evidence(1.5, GraspVerifier().verify)


def test_a_high_sample_count_with_no_actual_time_window_cannot_finish():
    s = sample_fixture();s._sample_times = [0.]*40
    assert not s.wait_evidence(.01, GraspVerifier().verify)


def test_stationary_part_does_not_pass_event_gate():
    s = sample_fixture();s._part = [(0.,0.)]*40
    assert not s.wait_evidence(0., GraspVerifier().verify)


def test_repeated_or_stale_camera_values_cannot_shorten_wait():
    s = sample_fixture();s._source_stamps = [1.]*40
    assert not s.wait_evidence(0., GraspVerifier().verify)
    s = sample_fixture();s._source_fresh = lambda: False
    assert not s.wait_evidence(0., GraspVerifier().verify)


def test_future_completion_before_registration_is_not_lost():
    m = MotionClient.__new__(MotionClient)
    m._fast_waits = True;m._wait_future = None;m._wait_event = None
    f = Future();f.set_result('done')
    m._future_pause(f,.01)
    assert m._wait_event.is_set()
    other = Future()
    m._future_pause(other,0.)
    assert not m._wait_event.is_set()
    other.set_result('done')
    assert m._wait_event.is_set()
