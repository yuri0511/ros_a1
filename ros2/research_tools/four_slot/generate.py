"""Generate isolated 240mm candidate bins, exact slot identities and numerical IK candidates."""
import sys,json,hashlib,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np,yaml
from types import SimpleNamespace
def rotation_vector(M):
    theta=np.arccos(np.clip((np.trace(M)-1)/2,-1,1))
    v=np.array([M[2,1]-M[1,2],M[0,2]-M[2,0],M[1,0]-M[0,1]])
    return v/2 if theta<1e-7 else v*theta/(2*np.sin(theta))
def least_squares(fun,q,bounds,**kw):
    q=q.copy()
    for iteration in range(400):
        error=fun(q)
        if np.linalg.norm(error)<1e-9:break
        J=np.column_stack([(fun(q+np.eye(6)[i]*1e-6)-error)/1e-6 for i in range(6)])
        step=np.linalg.solve(J.T@J+np.eye(6)*1e-6,-J.T@error)
        step*=min(1,.2/max(np.linalg.norm(step),1e-12))
        for scale in (1,.5,.25,.125,.0625):
            nxt=np.clip(q+scale*step,bounds[0]+1e-7,bounds[1]-1e-7)
            if np.linalg.norm(fun(nxt))<np.linalg.norm(error):q=nxt;break
        else:break
    return SimpleNamespace(x=q)
from geometry import ROOT,ROBOT,SUFFIXES,fk
W=Path(__file__).resolve().parents[2];C=W/'src/a1_sorting_core/config';S=W/'src/a1_cell_sim/worlds'
world=E.parse(S/'fast_sim_1ms.sdf');old=[];centers={}
for m in world.findall('.//model'):
    name=m.get('name','')
    if not name.startswith('bin_'):continue
    pose=list(map(float,m.find('pose').text.split()))
    if '--spread-x' in sys.argv:pose[0]=.25 if name in ('bin_blue','bin_red') else 1.05
    m.find('pose').text=' '.join(map(str,pose));centers[name]=pose[:2]
    for element in list(m.find('link')):
        n=element.get('name','');size=element.find('geometry/box/size');pose=element.find('pose')
        if size is None:continue
        old.append({'bin':name,'element':n,'size':size.text,'pose':pose.text if pose is not None else None})
        if 'wall_' in n or 'rim_' in n:
            xyz=list(map(float,pose.text.split()));axis=0 if '_x_' in n else 1;xyz[axis]=.115*(1 if '_pos' in n else -1)
            # Rims share the wall footprint, no fake extra exterior dimensions.
            size.text='0.01 0.24 '+('0.006' if 'rim_' in n else '0.095') if axis==0 else '0.22 0.01 '+('0.006' if 'rim_' in n else '0.095')
            if 'rim_' in n:xyz[2]=.812 # visual top no higher than collision rim
            pose.text=' '.join(map(str,xyz))
        elif n=='floor_visual':size.text='0.22 0.22 0.005'
# The old tabletop has only four local physical floor patches, NOT a full slab.
for model in world.findall('.//model'):
    if model.get('name')=='tabletop':
        for c in model.findall('link/collision'):
            bn='bin_'+c.get('name').removeprefix('floor_');cx,cy=centers[bn]
            c.find('pose').text=f'{cx-.70} {cy} 0 0 0 0';c.find('geometry/box/size').text='0.24 0.24 0.02'
E.indent(world);world.write(S/'four_slot_1ms.sdf',encoding='unicode',xml_declaration=True)
params=yaml.safe_load((C/'sorting_params_pipeline.yaml').read_text());base=yaml.safe_load((C/'sorting_params.yaml').read_text());offsets=[(-.045,-.045),(.045,-.045),(-.045,.045),(.045,.045)]
result={'schema':1,'motion_validated':False,'world':'four_slot_1ms.sdf','outer':[.24,.24],'inner':[.22,.22],'height':.095,'wall':.01,'cube':.05,'recycle_delay':0.,'offsets':offsets,'bins':{}}
overlay={'/result_recorder':{'ros__parameters':{'recycle_delay':0.}},'/task_dispatcher':{'ros__parameters':{'four_slot_mode':True}},'/scene_publisher':{'ros__parameters':{'four_slot_mode':True}}}
for nodename,colors,prefix in [('/pick_place_executor',['blue','yellow'],''),('/pick_place_relay',['red','green'],'arm2_')]:
    p=params[nodename]['ros__parameters'];names=[prefix+n for n in SUFFIXES];targetR=fk(prefix+'tool0',dict(zip(names,p['carry_anchor1_joints'])))[:3,:3]
    bounds=[]
    for n in names:
        lim=next(j for j in ROBOT.findall('joint') if j.get('name')==n).find('limit');bounds.append([float(lim.get('lower')),float(lim.get('upper'))])
    op={'four_slot_mode':True,'bin_grid_offsets.x':[x for x,y in offsets],'bin_grid_offsets.y':[y for x,y in offsets]}
    for bi,color in enumerate(colors):
        bn='bin_'+color;cx,cy=centers[bn];slots=[];seed=np.array(p[f'carry_anchor{bi*3+2}_joints'])
        if '--elbow-negative' in sys.argv:
            seed[1]+=seed[2];seed[3]+=seed[2];seed[2]*=-1
        for i,(dx,dy) in enumerate(offsets,1):
            xyz=[cx+dx,cy+dy,.745];entry={'id':f'{bn}/slot_{i}','index':i,'xyz':xyz,'arm':'arm2' if prefix else 'arm1','waypoints':{}}
            q=seed.copy()
            for stage,z in [('carry',1.08),('settle_top',.99),('settle_bottom',.958)]:
                target=np.array([xyz[0],xyz[1],z])
                def fun(v):
                    M=fk(prefix+'tool0',dict(zip(names,v)));return np.r_[M[:3,3]-target,rotation_vector(targetR.T@M[:3,:3])]
                opt=least_squares(fun,q,bounds=np.array(bounds).T,xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=400)
                if np.linalg.norm(fun(opt.x))>1e-7:raise RuntimeError(f'IK failed {entry["id"]} {stage}')
                q=opt.x;entry['waypoints'][stage]=q.tolist()
            slots.append(entry);n=bi*4+i
            op[f'carry_anchor{n}_xy']=xyz[:2];op[f'carry_anchor{n}_joints']=entry['waypoints']['carry'];op[f'carry_settle_top{n}_joints']=entry['waypoints']['settle_top'];op[f'carry_settle_bottom{n}_joints']=entry['waypoints']['settle_bottom']
        result['bins'][bn]={'center':[cx,cy],'slots':slots}
        op[f'{bn}.x']=cx;op[f'{bn}.y']=cy
        rp=overlay['/result_recorder']['ros__parameters']
        for k,v in [('x',cx),('y',cy),('half_x',.12),('half_y',.12),('inner_half_x',.11),('inner_half_y',.11)]:rp[f'bins.{bn}.{k}']=v
    overlay[nodename]={'ros__parameters':op}
(C/'four_slot_candidate.json').write_text(json.dumps(result,indent=2));(C/'four_slot_overlay.yaml').write_text(yaml.safe_dump(overlay,sort_keys=False))
(ROOT/'audit/original-bin-geometry.json').write_text(json.dumps(old,indent=2));(ROOT/'audit/candidate-ik.json').write_text(json.dumps(result,indent=2))
print('16 slots x 3 FK/IK targets generated; NOT trajectory certified')
