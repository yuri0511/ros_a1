from pathlib import Path
import json,shlex,numpy as np
R=Path('/home/yu/yuri_gazebo_performance/voice_batch_aliyun');summary=json.loads((R/'audit/mesh-summary.json').read_text());hits=[h for h in summary['collisions'] if h['config']['slot']==3 and h['config']['finger_position']==-.003];worst=max(hits,key=lambda h:float(h['penetration_depth_m']));k=int(worst['case']);objects={}
for line in (R/'audit/mesh-queries.txt').read_text().splitlines():
 v=shlex.split(line)
 if int(v[0])!=k:continue
 if v[2] not in (worst['object1'],worst['object2']):continue
 assert v[3]=='box';M=np.array([float(x) for x in v[7:19]]).reshape(3,4);objects[v[2]]=(np.array([float(x) for x in v[4:7]])/2,M[:,:3],M[:,3])
a,b=[objects[n] for n in (worst['object1'],worst['object2'])];axes=[*a[1].T,*b[1].T]+[np.cross(x,y) for x in a[1].T for y in b[1].T];tests=[]
for axis in axes:
 length=np.linalg.norm(axis)
 if length<1e-12:continue
 axis=axis/length;radius_a=float(np.abs(a[1].T@axis)@a[0]);radius_b=float(np.abs(b[1].T@axis)@b[0]);distance=abs(float((a[2]-b[2])@axis));tests.append({'axis':axis.tolist(),'signed_separation_m':distance-radius_a-radius_b})
result={'purpose':'Independent exact box-vs-box 15-axis SAT; no mesh conservatism or expanded tolerance','sample':worst,'axis_checks':tests,'all_axes_overlap':all(x['signed_separation_m']<0 for x in tests),'least_overlap_m':-max(x['signed_separation_m'] for x in tests),'conclusion':'Static nominal green3 path includes finger/wall intersection even at the closed mechanical limit. Not proof an actual Gazebo collision occurred. Gate physical validation until reviewed; no geometry/threshold/path modification authorized.'}
assert result['all_axes_overlap'];(R/'audit/green3-independent-sat.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:result[k] for k in ('all_axes_overlap','least_overlap_m','conclusion')},indent=2))
