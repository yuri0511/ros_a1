"""FCL inputs from real Pilz output points, never from hand interpolation."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from geometry import ROOT,ROBOT,SUFFIXES,ARM2,fk,origin
R=Path(__file__).resolve().parents[1];W=R/'worktree/ros2'
plans=json.loads((R/'audit/planned-trajectories.json').read_text())
world=ET.parse(W/'src/a1_cell_sim/worlds/fast_sim_1ms.sdf').getroot()
lines=[];cases=[]
def row(k,g,n,kind,abc,M,path='',scale=(1,1,1)):
    lines.append(' '.join(map(str,[k,g,n,kind,*abc,*M[:3,:].flatten(),json.dumps(path),*scale])))
def robot(k,v):
    for link in ROBOT.findall('link'):
        n=link.get('name')
        if n in ('world','pedestal','pedestal2'):continue
        for i,c in enumerate(link.findall('collision')):
            x=list(c.find('geometry'))[0];kind=x.tag;path='';scale=(1,1,1)
            if kind=='mesh':
                abc=(0,0,0);path=x.get('filename').removeprefix('file://');scale=tuple(map(float,x.get('scale','1 1 1').split()))
            elif kind=='box':abc=tuple(map(float,x.get('size').split()))
            elif kind=='cylinder':abc=(float(x.get('radius')),float(x.get('length')),0)
            elif kind=='sphere':abc=(float(x.get('radius')),0,0)
            else:raise ValueError(kind)
            row(k,'arm2' if n.startswith('arm2_') else 'arm1',n+'__'+str(i),kind,abc,fk(n,v)@origin(c.find('origin')),path,scale)
def obstacles(k):
    for model in world.findall('.//model'):
        if not model.get('name','').startswith('bin_'):continue
        for link in model.findall('link'):
            for c in link.findall('collision'):
                size=c.find('geometry/box/size')
                if size is not None:row(k,'bin',model.get('name')+'_'+c.get('name'),'box',list(map(float,size.text.split())),origin(model.find('pose'),True)@origin(link.find('pose'),True)@origin(c.find('pose'),True))
for plan in plans:
    response=plan['response']['motion_plan_response']
    if response['error_code']['val']!=1:continue
    jt=response['trajectory']['joint_trajectory'];start=plan['request']['motion_plan_request']['start_state']['joint_state']
    base=dict(zip(start['name'],start['position']))
    # Closed, small release, and fully-open bounds. No claim that endpoints
    # alone certify the swept opening operation; this audit finds violations.
    for i,point in enumerate(jt['points']):
        for left,right in ((-.003,-.003),(-.001,-.001),(0.,0.),(.005,.005),(.04,.04)):
            k=len(cases);v=dict(base);v.update(zip(jt['joint_names'],point['positions']))
            v.update(arm2_gripper_finger_left_joint=left,arm2_gripper_finger_right_joint=right)
            cases.append({'case':k,'slot':plan['slot'],'label':plan['label'],'point':i,'time':point['time_from_start'],'fingers':[left,right],'joints':{n:v[n] for n in ARM2}})
            robot(k,v);obstacles(k)
            # Same old nominal part model; tagged explicitly, not actual attached state.
            M=np.eye(4);M[:3,3]=fk('arm2_tool0',v)[:3,3]-[0,0,.204]
            row(k,'held','held_part_NOMINAL_offset_0204','box',(.05,.05,.05),M)
(R/'audit/planned-cases.json').write_text(json.dumps(cases))
(R/'audit/planned-queries.txt').write_text('\n'.join(lines)+'\n')
print('cases',len(cases),'objects',len(lines))
