import pytest
from a1_sorting_core.four_slot_grip_control import opening_efforts

def test_asymmetry_requires_opposite_efforts():
    left,right=opening_efforts([.010,.013])
    assert left>0 and right<0

def test_force_cap_does_not_change_feedback():
    q=[-.003,.04];before=q.copy()
    assert opening_efforts(q)==[1.5,-1.5]
    assert q==before

@pytest.mark.parametrize('q',[[.01],[float('nan'),.01],[float('inf'),.01]])
def test_invalid_feedback_is_rejected(q):
    with pytest.raises(ValueError):opening_efforts(q)

def test_target_cannot_relax_release_envelope():
    with pytest.raises(ValueError):opening_efforts([.012,.012],.014)

def test_candidate_timer_uses_independent_fingers_and_keeps_legacy_timer_out():
    from types import SimpleNamespace as N
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode as E
    sent=[]
    fake=N(_four_slot=True,_four_slot_open_hold=True,_arm_fault=None,
           _release_evidence=N(phase='EXITING',fault=None,check=lambda *a:True),
           _finger_joint_keys=lambda:('L','R'),_joint_lookup=lambda k:{'L':.010,'R':.013}[k],
           get_clock=lambda:N(now=lambda:N(nanoseconds=10**9)),
           _pub_gripper=N(publish=lambda m:sent.append(list(m.data))))
    E._four_slot_feedback_tick(fake)
    assert sent[0][0]>0 and sent[0][1]<0
    E._grip_hold_tick(fake)
    assert len(sent)==1


def test_candidate_fault_disables_hold_and_latches_arm_fault():
    from types import SimpleNamespace as N
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode as E
    commands=[]
    fake=N(_four_slot_open_hold=True,_arm_fault=None,_release_fault_announced=False,
           _release_evidence=N(phase='EXITING',fault='stale_feedback',check=lambda *a:False),
           get_clock=lambda:N(now=lambda:N(nanoseconds=10**9)),_pub_gripper_cmd=commands.append)
    E._four_slot_feedback_tick(fake)
    assert not fake._four_slot_open_hold and commands==[0.]
    assert fake._arm_fault=='release_feedback:stale_feedback'


def test_legacy_constant_effort_is_preserved():
    from types import SimpleNamespace as N
    from a1_sorting_core.pick_place_executor_node import PickPlaceExecutorNode as E
    sent=[];fake=N(_four_slot=False,_grip_hold_target=None,_grip_hold=-8.,_gripper_joints=2,
                  _pub_gripper=N(publish=lambda m:sent.append(list(m.data))))
    E._grip_hold_tick(fake)
    assert sent==[[-8.,-8.]]
