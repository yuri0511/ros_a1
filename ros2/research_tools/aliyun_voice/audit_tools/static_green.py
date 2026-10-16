from pathlib import Path
import json,math,hashlib,xml.etree.ElementTree as E
import numpy as np
import yaml
R=Path('/home/yu/yuri_gazebo_performance/voice_batch_aliyun');W=R/'worktree/ros2';params=yaml.safe_load((W/'src/a1_sorting_core/config/sorting_params_pipeline.yaml').read_text())['/pick_place_relay']['ros__parameters'];root=E.parse(R/'audit/robot-static.urdf').getroot();joints=list(root.findall('joint'));parents={j.find('child').get('link'):j for j in joints}
def rot(axis,a):
 x=np.array(axis);x=x/np.linalg.norm(x);K=np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]]);return np.eye(3)+math.sin(a)*K+(1-math.cos(a))*(K@K)
def fk(child,values):
 if child not in parents:return np.eye(4)
 j=parents[child];M=fk(j.find('parent').get('link'),values);O=np.eye(4);orig=j.find('origin')
 if orig is not None:
  O[:3,3]=[float(x) for x in orig.get('xyz','0 0 0').split()];r,p,y=[float(x) for x in orig.get('rpy','0 0 0').split()];O[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
 J=np.eye(4)
 if j.get('type') in ('revolute','continuous'):
  a=j.find('axis');axis=[float(x) for x in (a.get('xyz') if a is not None else '1 0 0').split()];J[:3,:3]=rot(axis,values.get(j.get('name'),0.))
 return M@O@J
names=['arm2_'+n for n in ('shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint')];anchor=[]
for kind,key in [('carry','carry_anchor6_joints'),('settle_top','carry_settle_top6_joints'),('settle_bottom','carry_settle_bottom6_joints')]:
 v=list(map(float,params[key]));lim=[]
 for n,q in zip(names,v):
  j=next(j for j in joints if j.get('name')==n);b=j.find('limit');lim.append(float(b.get('lower','-inf'))<=q<=float(b.get('upper','inf')))
 pose=fk('arm2_tool0',dict(zip(names,v)))
 anchor.append({'stage':kind,'joints':v,'within_urdf_joint_limits':all(lim),'tool0_xyz':pose[:3,3].tolist(),'xy_error_m':float(np.linalg.norm(pose[:2,3]-[.87,-.23]))})
slots=[(.90+x,-.28+y) for x,y in zip(params['bin_grid_offsets.x'],params['bin_grid_offsets.y'])];half=.025
neighbor=[]
for i,a in enumerate(slots):
 for j,b in enumerate(slots[:i]):neighbor.append({'slots':[j+1,i+1],'axis_aligned_clearance_xy_m':[abs(a[0]-b[0])-.05,abs(a[1]-b[1])-.05]})
out={'green_slots_xy':slots,'third_carry_anchor_xy':params['carry_anchor6_xy'],'third_slot_interior_clearance_xy_m':[.06-abs(slots[2][0]-.90)-half,.085-abs(slots[2][1]+.28)-half],'anchors':anchor,'neighbor_nominal_clearance':neighbor,'recycle_delay_sim_seconds':12,'recycle_success_is_not_guaranteed':True,'missing_original_anchor_provenance':'ros2/docs/evidence/G0_anchors_v3.json remains missing; not fabricated','full_mesh_collision_or_trajectory_checked':False,'physical_third_green_verified':False,'provisional_requirement':'Before a repeated same-color part can be dispatched, require successful original recycling/absence proof for earlier same-bin parts, plus normal MoveIt collision planning and physical guards. Never rely only on 12s elapsed.'}
(R/'audit/green-third-static.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
