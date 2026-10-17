"""Explainable contact categories. Unknown geometry remains stop-worthy.
No arbitrary penetration allowance and no new permission to contact a bin wall.
"""
from dataclasses import dataclass
@dataclass(frozen=True)
class ContactDecision:
    category:str
    action:str
    reason:str

def classify(a,b,depth,*,held_part=None,placed_bins=None,controller_ok=True,arm_fault=False):
    if arm_fault or not controller_ok:return ContactDecision('controller_or_ARM_FAULT','STOP','motion state invalid')
    pair=(a,b);robot=[x for x in pair if x.startswith('ur::')]
    if len(robot)==2 and ('arm2_' in robot[0])!=('arm2_' in robot[1]):return ContactDecision('inter_arm','STOP','independent robot bodies')
    if depth<=0:return ContactDecision('no_positive_penetration','RECORD','not a permission to move')
    other=b if a.startswith('ur::') else a
    if robot and held_part and other.startswith(held_part+'::') and 'gripper_finger_' in robot[0]:return ContactDecision('expected_grip','RECORD','current identified held object only')
    for part,bin_name in (placed_bins or {}).items():
        if any(x.startswith(part+'::') for x in pair) and any('tabletop::' in x and 'floor_'+bin_name.removeprefix('bin_') in x for x in pair):return ContactDecision('expected_bin_floor','RECORD','identity and matching floor')
    if robot and other.startswith('bin_'):return ContactDecision('unverified_finger_wall' if 'gripper_finger_' in robot[0] else 'robot_body_wall','STOP','not automatically called severe; no validated allowance')
    if robot and other.startswith('part_'):return ContactDecision('robot_existing_part','STOP','not current held object')
    return ContactDecision('unclassified_contact','STOP','conservative until reviewed')
