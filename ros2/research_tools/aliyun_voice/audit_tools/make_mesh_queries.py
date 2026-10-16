from pathlib import Path
import json,xml.etree.ElementTree as E,runpy,contextlib,io,numpy as np
R=Path('/home/yu/yuri_gazebo_performance/voice_batch_aliyun');W=R/'worktree/ros2'
with contextlib.redirect_stdout(io.StringIO()):d=runpy.run_path(str(R/'tools/static_green.py'))
root=d['root'];parents=d['parents'];rot=d['rot'];params=d['params'];names=d['names']
def origin(x,sdf=False):
 M=np.eye(4)
 if x is None:return M
 v=[float(a) for a in (x.text if sdf else x.get('xyz','0 0 0')).split()]
 xyz=v[:3];rpy=v[3:] if sdf else [float(a) for a in x.get('rpy','0 0 0').split()]
 if len(rpy)<3:rpy=[0,0,0]
 M[:3,3]=xyz;M[:3,:3]=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0]);return M

def fk(child,vals):
 if child not in parents:return np.eye(4)
 j=parents[child];M=fk(j.find('parent').get('link'),vals)@origin(j.find('origin'));J=np.eye(4);a=j.find('axis');axis=np.array([float(x) for x in (a.get('xyz') if a is not None else '1 0 0').split()]);q=vals.get(j.get('name'),0.)
 if j.get('type') in ('revolute','continuous'):J[:3,:3]=rot(axis,q)
 elif j.get('type')=='prismatic':J[:3,3]=axis*q
 return M@J

world=E.fromstring((W/'src/a1_cell_sim/worlds/fast_sim_1ms.sdf').read_text());green=world.find(".//model[@name='bin_green']");out=[];cases=[];baseyaml=__import__('yaml').safe_load((W/'src/a1_sorting_core/config/sorting_params_pipeline.yaml').read_text());arm1=__import__('yaml').safe_load((W/'src/a1_sorting_core/config/sorting_params.yaml').read_text())['/pick_place_executor']['ros__parameters']['standby_joints'];base=dict(zip([n.removeprefix('arm2_') for n in names],arm1))
def row(k,g,n,kind,abc,M,path='',scale=(1,1,1)):
 out.append(' '.join(map(str,[k,g,n,kind,*abc,*M[:3,:].flatten(),json.dumps(path),*scale])))
for slot in (4,5,6):
 anchors=[params[f'carry_anchor{slot}_joints'],params[f'carry_settle_top{slot}_joints'],params[f'carry_settle_bottom{slot}_joints']]
 points=[np.array(anchors[0])]
 for a,b in zip(anchors,anchors[1:]):points.extend(np.array(a)+(np.array(b)-a)*t for t in np.linspace(0,1,9)[1:])
 for index,point in enumerate(points):
  for gap in (-.003,0.,.04):
   k=len(cases);vals=dict(base);vals.update(zip(names,point));vals.update({prefix+'gripper_finger_'+side+'_joint':gap for prefix in ('','arm2_') for side in ('left','right')});cases.append({'index':k,'slot':slot-3,'path_sample':index,'finger_position':gap,'arm2_joints':point.tolist()})
   for link in root.findall('link'):
    name=link.get('name')
    if name in ('world','pedestal','pedestal2'):continue
    g='arm2' if name.startswith('arm2_') else 'arm1'
    for i,c in enumerate(link.findall('collision')):
     M=fk(name,vals)@origin(c.find('origin'));geom=c.find('geometry');x=list(geom)[0];kind=x.tag;path='';scale=(1,1,1)
     if kind=='mesh':abc=(0,0,0);path=x.get('filename').removeprefix('file://');scale=tuple(map(float,x.get('scale','1 1 1').split()));assert Path(path).is_file(),path
     elif kind=='box':abc=tuple(map(float,x.get('size').split()))
     elif kind=='cylinder':abc=(float(x.get('radius')),float(x.get('length')),0)
     elif kind=='sphere':abc=(float(x.get('radius')),0,0)
     else:raise RuntimeError(kind)
     row(k,g,name+'__'+str(i),kind,abc,M,path,scale)
   for link in green.findall('link'):
    for c in link.findall('collision'):
     M=origin(green.find('pose'),True)@origin(link.find('pose'),True)@origin(c.find('pose'),True);size=list(map(float,c.find('geometry/box/size').text.split()));row(k,'bin','green_'+c.get('name'),'box',size,M)
   # Nominal held 50mm cube centered below tool0 by original tcp_z_offset.
   M=np.eye(4);M[:3,3]=fk('arm2_tool0',vals)[:3,3]-np.array([0,0,float(params['tcp_z_offset'])]);row(k,'held','held_part_nominal','box',(.05,.05,.05),M)
(R/'audit/mesh-queries.txt').write_text('\n'.join(out)+'\n');(R/'audit/mesh-cases.json').write_text(json.dumps(cases,indent=2));print('cases',len(cases),'objects',len(out))
