"""Dense collision audit of actual Pilz/JTC spline, exact finger boxes/walls.

Uses original positions/velocities/accelerations to reconstruct the quintic
interpolation. No trajectory commands or altered collision geometry.
"""
from pathlib import Path
import json,itertools
import numpy as np
from geometry import ROBOT,ARM2,fk,origin
R=Path(__file__).resolve().parents[1]
plans=json.loads((R/'audit/planned-trajectories.json').read_text())
wall=[([.9,y,.7675],[.13,.01,.095]) for y in (-.37,-.19)] + [([x,-.28,.7675],[.01,.19,.095]) for x in (.835,.965)]
def separates(A,sa,B,sb):
    axes=[*A[:3,:3].T,*B[:3,:3].T]
    axes.extend(np.cross(a,b) for a in A[:3,:3].T for b in B[:3,:3].T)
    gaps=[]
    for axis in axes:
        norm=np.linalg.norm(axis)
        if norm<1e-12:continue
        axis=axis/norm
        gaps.append(abs((A[:3,3]-B[:3,3])@axis)-sum(abs(A[:3,:3].T@axis)*np.array(sa)/2)-sum(abs(B[:3,:3].T@axis)*np.array(sb)/2))
    return max(gaps)
def spline(a,b,u):
    def t(p):return p['time_from_start']['sec']+p['time_from_start']['nanosec']*1e-9
    h=t(b)-t(a);p0=np.array(a['positions']);p1=np.array(b['positions']);v0=np.array(a['velocities']);v1=np.array(b['velocities']);acc0=np.array(a['accelerations']);acc1=np.array(b['accelerations'])
    c0=p0;c1=h*v0;c2=h*h*acc0/2
    b0=p1-c0-c1-c2;b1=h*v1-c1-2*c2;b2=h*h*acc1-2*c2
    c3=10*b0-4*b1+b2/2;c4=-15*b0+7*b1-b2;c5=6*b0-3*b1+b2/2
    return c0+c1*u+c2*u*u+c3*u**3+c4*u**4+c5*u**5
links=[l for l in ROBOT.findall('link') if l.get('name') in ('arm2_gripper_finger_left_link','arm2_gripper_finger_right_link')]
out={'kind':'exact_OBB_separating_axis_on_dense_JTC_command_spline','dt_max':.002,'slot':2,
 'limits':'commanded trajectory, not physical tracking; held-part offsets bounded sensitivity, not actual fresh attachment',
 'samples':0,'finger_wall_intersections':[],'part_wall_intersections':[],
 'min_positive_separating_axis_gap_m':1e9,'fingers_swept_range':[-.003,.04],
 'part_local_xy_offsets_m':[-.03,0.,.03]}
for plan in plans:
    if plan['slot']!=2:continue
    # Held part belongs only to carry/settle phases; return/retreat are released.
    carried=plan['label'] not in ('return','retreat')
    jt=plan['response']['motion_plan_response']['trajectory']['joint_trajectory'];points=jt['points'];start=plan['request']['motion_plan_request']['start_state']['joint_state'];base=dict(zip(start['name'],start['position']))
    for a,b in zip(points,points[1:]):
        ta=a['time_from_start']['sec']+a['time_from_start']['nanosec']*1e-9;tb=b['time_from_start']['sec']+b['time_from_start']['nanosec']*1e-9
        for u in np.linspace(0,1,max(2,int(np.ceil((tb-ta)/.002))+1)):
            q=spline(a,b,u);v=dict(base);v.update(zip(jt['joint_names'],q));out['samples']+=1
            for link in links:
                c=link.find('collision');size=list(map(float,c.find('geometry/box').get('size').split()))
                # Union of all prismatic positions is an exact longer box:
                # translation only, unchanged orientation, covers independently
                # asymmetric fingers including the entire opening operation.
                j=next(j for j in ROBOT.findall('joint') if j.find('child').get('link')==link.get('name'))
                v[j.get('name')]=(.04-.003)/2
                M=fk(link.get('name'),v)@origin(c.find('origin'));size[1]+=.043
                for wi,(xyz,wallsize) in enumerate(wall):
                    B=np.eye(4);B[:3,3]=xyz;gap=separates(M,size,B,wallsize)
                    out['min_positive_separating_axis_gap_m']=min(out['min_positive_separating_axis_gap_m'],gap)
                    if gap<=0:out['finger_wall_intersections'].append({'label':plan['label'],'time':ta+u*(tb-ta),'link':link.get('name'),'wall':wi,'overlap':-gap})
            if carried:
                M=fk('arm2_gripper_base',v)
                # Reconstructed attachment's frame is almost down at grasp;
                # use its actual recorded rotational relation for cube pose.
                rel=np.array(json.loads((R/'audit/attachment-reconstruction.json').read_text())['relative_transform_approx'])
                for dx,dy in itertools.product((-.03,0.,.03),repeat=2):
                    local=rel.copy();local[0,3]=dx;local[1,3]=dy;A=M@local
                    for wi,(xyz,wallsize) in enumerate(wall):
                        B=np.eye(4);B[:3,3]=xyz;gap=separates(A,[.05]*3,B,wallsize)
                        if gap<=0:out['part_wall_intersections'].append({'label':plan['label'],'time':ta+u*(tb-ta),'offset':[dx,dy],'wall':wi,'overlap':-gap})
(R/'audit/center-dense-sweep.json').write_text(json.dumps(out,indent=2));print(json.dumps({k:v if not isinstance(v,list) or len(v)<10 else len(v) for k,v in out.items()},indent=2))
