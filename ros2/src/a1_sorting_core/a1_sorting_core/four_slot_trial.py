"""Scoped simulation trial authority; never sets global motion_validated.

A permit only enables one yellow slot, with completed calibration and exact
source/config/trajectory hashes. Live exit evidence remains a separate gate.
"""
import hashlib,json,time,math
from pathlib import Path
import numpy as np
import xml.etree.ElementTree as ET

class TrialError(RuntimeError):pass

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load_trial(path, candidate):
    d=json.loads(Path(path).read_text())
    if json.loads(Path(candidate).read_text()).get('schema')!=1:raise TrialError('candidate_schema_changed')
    if d.get('scope') not in ('isolated-yellow-single','isolated-five-task') or d.get('maximum_parts')!=(1 if d['scope']=='isolated-yellow-single' else 5):raise TrialError('trial_scope_invalid')
    if d['scope']=='isolated-five-task':
        b=json.loads((Path(d['yellow_run'])/'summary.json').read_text())
        if b.get('reason')!='completed' or not b.get('cleanup_ok') or b['result']['csv']['rows']!=1:raise TrialError('yellow_real_validation_missing')
    expected_counts=[0,1,0,0] if d['scope']=='isolated-yellow-single' else [0,1,1,3]
    if d.get('counts')!=expected_counts:raise TrialError('trial_count_scope_invalid')
    if not time.time()<d.get('expires_wall',0)<=time.time()+3600:raise TrialError('trial_expired')
    if d.get('candidate_sha256')!=sha(candidate):raise TrialError('trial_candidate_changed')
    for p,h in d.get('evidence_sha256',{}).items():
        if sha(p)!=h:raise TrialError('trial_evidence_changed')
    if not d.get('evidence_sha256') or not d.get('source_sha256'):raise TrialError('trial_missing_evidence')
    for p,h in d['source_sha256'].items():
        if sha(p)!=h:raise TrialError('trial_source_changed')
    # Revalidate substantive evidence, not a PASS field.
    run=Path(d['calibration_run']);summary=json.loads((run/'summary.json').read_text());rows=json.loads((run/'support-held-cycle.json').read_text())
    if summary.get('outcome')!='fixture_cycle_pass' or summary.get('observer_exit')!=0 or summary.get('scene_exit')!=0 or (run/'guard-fault.json').exists() or not (run/'FINGER_DONE').exists():raise TrialError('calibration_incomplete')
    if [r['stage'] for r in rows]!=['depart_32mm','exit_above_bin','return_above_support','land_original_pose']:raise TrialError('calibration_stages_missing')
    for r in rows:
        if r['action']['status']!=4 or r['action']['result']['error_code']!=0 or not r.get('post_action_stability') or not r.get('actual_native') or not r.get('independent_checks') or any(not x['validity']['valid'] for x in r['independent_checks']):raise TrialError('calibration_evidence_invalid')
    if json.loads((run/'support-contact-permit.json').read_text()).get('phase')!='normal':raise TrialError('support_constraints_not_restored')
    review=json.loads(Path(d['path_review']).read_text())
    if not review.get('pickup_to_carry_payload_scene_verified'):raise TrialError('pickup_payload_scene_not_verified')
    if not review.get('no_gazebo') or not review.get('no_motion') or len(review.get('segments',[]))!=48:raise TrialError('path_review_missing')
    for r in review['segments']:
        if r['code']!=1 or r['samples']<1 or r['invalid'] or r.get('above_bin',True) is not True:raise TrialError('path_review_failed')
    for p,h in review['sha256'].items():
        if sha(p)!=h:raise TrialError('review_source_changed')
    return d

def gripper_bottoms(urdf, joints, prefix=''):
    """Full original collision boxes transformed by real joint feedback."""
    root=ET.fromstring(urdf);parents={j.find('child').get('link'):j for j in root.findall('joint')};cache={}
    def rot(axis,q):
        a=np.array(axis,dtype=float);a/=np.linalg.norm(a);x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K)
    def origin(e):
        M=np.eye(4)
        if e is not None:
            M[:3,3]=list(map(float,e.get('xyz','0 0 0').split()));r,p,y=map(float,e.get('rpy','0 0 0').split());M[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
        return M
    def fk(link):
        if link in cache:return cache[link]
        if link not in parents:return np.eye(4)
        j=parents[link];M=fk(j.find('parent').get('link'))@origin(j.find('origin'));J=np.eye(4)
        if j.get('type')!='fixed':
            name=j.get('name')
            if name not in joints or not math.isfinite(joints[name]):raise TrialError('exit_joint_missing')
            axis=list(map(float,j.find('axis').get('xyz').split()))
            if j.get('type')=='prismatic':J[:3,3]=np.array(axis)*joints[name]
            else:J[:3,:3]=rot(axis,joints[name])
        cache[link]=M@J;return cache[link]
    out={}
    for name in [prefix+'gripper_base',prefix+'gripper_finger_left_link',prefix+'gripper_finger_right_link']:
        link=root.find(f"link[@name='{name}']")
        if link is None:raise TrialError('exit_link_missing')
        for i,c in enumerate(link.findall('collision')):
            box=c.find('geometry/box')
            if box is None:raise TrialError('exit_unsupported_shape')
            M=fk(name)@origin(c.find('origin'));size=np.array(list(map(float,box.get('size').split())))
            out[name+str(i)]=float(M[2,3]-np.abs(M[2,:3])@(size/2))
    if not out:raise TrialError('exit_empty_geometry')
    return out
