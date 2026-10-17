"""Dense command-spline OBB audit: exact fingers + retained cubes + held envelope.
Mesh/body checks are in real MoveIt/FCL; this independent audit covers the gaps.
"""
from pathlib import Path
import json,math,xml.etree.ElementTree as E
import numpy as np
from geometry import ROOT,ROBOT,SUFFIXES,fk,origin
W=Path(__file__).resolve().parents[2];cfg=json.loads((W/'src/a1_sorting_core/config/four_slot_candidate.json').read_text());plans=json.loads((ROOT/'audit/plans.json').read_text())
world=E.parse(W/'src/a1_cell_sim/worlds/four_slot_1ms.sdf');walls=[]
for m in world.findall('.//model'):
    if m.get('name','').startswith('bin_'):
        for c in m.findall('link/collision'):
            M=origin(m.find('pose'),True)@origin(c.find('pose'),True);walls.append((m.get('name')+'/'+c.get('name'),M,np.array(list(map(float,c.find('geometry/box/size').text.split())))))
def gap(A,sa,B,sb):
    # Exact SAT; max positive projection gap is a conservative separation metric.
    delta=A[:3,3]-B[:3,3];ra=A[:3,:3];rb=B[:3,:3]
    axes=np.vstack([ra.T,rb.T,np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)])
    norms=np.linalg.norm(axes,axis=1);axes=axes[norms>1e-10]/norms[norms>1e-10,None]
    return float(np.max(np.abs(axes@delta)-np.abs(axes@ra)@(sa/2)-np.abs(axes@rb)@(sb/2)))
def t(p):return p['time_from_start']['sec']+p['time_from_start']['nanosec']*1e-9
def spline(a,b,u):
    h=t(b)-t(a);p0=np.array(a['positions']);p1=np.array(b['positions']);v0=np.array(a['velocities']);v1=np.array(b['velocities']);acc0=np.array(a['accelerations']);acc1=np.array(b['accelerations'])
    c0=p0;c1=h*v0;c2=h*h*acc0/2;b0=p1-c0-c1-c2;b1=h*v1-c1-2*c2;b2=h*h*acc1-2*c2
    return c0+c1*u+c2*u*u+(10*b0-4*b1+b2/2)*u**3+(-15*b0+7*b1-b2)*u**4+(6*b0-3*b1+b2/2)*u**5
out={'dt_max':.002,'samples':0,'segments':[],'intersections':[],'held_envelope':'50mm cube + +/-30mm lateral pose uncertainty; nominal z offset 0.204m from tool0; conservative, not measured new grasp','full_open_only_at_return':True}
for plan in plans:
    resp=plan['response']['motion_plan_response'];slotid=plan['slot'];bn=slotid.split('/')[0];slot=next(s for s in cfg['bins'][bn]['slots'] if s['id']==slotid);prefix='' if slot['arm']=='arm1' else 'arm2_';jt=resp['trajectory']['joint_trajectory'];base=dict(zip(plan['request']['motion_plan_request']['start_state']['joint_state']['name'],plan['request']['motion_plan_request']['start_state']['joint_state']['position']))
    obstacles=list(walls)
    for s in cfg['bins'][bn]['slots']:
        # After release, current item also exists under fingers during rise/return.
        if s['id']!=slotid or plan['stage'] in ('rise','return'):
            M=np.eye(4);M[:3,3]=s['xyz'];obstacles.append((s['id'],M,np.array([.066,.066,.05])))
    report={'slot':slotid,'stage':plan['stage'],'samples':0,'minimum_separating_gap':1e9,'hits':0}
    for a,b in zip(jt['points'],jt['points'][1:]):
        for u in np.linspace(0,1,max(2,int(math.ceil((t(b)-t(a))/.002))+1)):
            vals=dict(base);vals.update(zip(jt['joint_names'],spline(a,b,u)));report['samples']+=1;out['samples']+=1
            objects=[]
            # Swept finger envelope covers asymmetric left/right states continuously.
            lo,hi=((- .003,.04) if plan['stage']=='return' else ((.008,.014) if plan['stage']=='rise' else (-.003,.014)))
            for side in ('left','right'):
                n=prefix+'gripper_finger_'+side+'_link';joint=prefix+'gripper_finger_'+side+'_joint';vals[joint]=(lo+hi)/2
                c=next(l for l in ROBOT.findall('link') if l.get('name')==n).find('collision');sz=np.array(list(map(float,c.find('geometry/box').get('size').split())));sz[1]+=hi-lo
                objects.append((n,fk(n,vals)@origin(c.find('origin')),sz))
            if plan['stage'] not in ('rise','return'):
                M=fk(prefix+'tool0',vals);local=np.eye(4);local[2,3]=.204;objects.append(('held_uncertainty_envelope',M@local,np.array([.11,.11,.05])))
            for name,A,sa in objects:
                for other,B,sb in obstacles:
                    # Broadphase AABB avoids expensive SAT far from the active bin.
                    if np.any(np.abs(A[:3,3]-B[:3,3])>np.abs(A[:3,:3])@(sa/2)+np.abs(B[:3,:3])@(sb/2)+.05):continue
                    g=gap(A,sa,B,sb);report['minimum_separating_gap']=min(report['minimum_separating_gap'],g)
                    if g<=0:
                        report['hits']+=1
                        if len(out['intersections'])<150:out['intersections'].append({'slot':slotid,'stage':plan['stage'],'time':t(a)+u*(t(b)-t(a)),'a':name,'b':other,'overlap_m':-g})
    out['segments'].append(report);(ROOT/'audit/dense-sweep.json').write_text(json.dumps(out,indent=2));print(report,flush=True)
