"""Cross-source simulation check. Nearest-time mismatch is not a pose-error bound."""
import bisect,gzip,json,math,sys
from pathlib import Path
from analyze_static import pose
from a1_sorting_core.retained_scene_sync import camera_observation
r=Path(sys.argv[1]);native=[]
for l in gzip.open(r/'raw-ros.jsonl.gz','rt'):
 d=json.loads(l)
 if d['kind']!='native_pose':continue
 s=d['message'].get('header',{}).get('stamp',{});stamp=float(s.get('sec',0))+float(s.get('nsec',0))/1e9
 for p in d['message'].get('pose',[]):
  if p['name']=='calibration_red4':native.append((stamp,p))
native.sort(key=lambda x:x[0]);times=[p[0] for p in native];rows=[]
for l in (r/'events.jsonl').open():
 d=json.loads(l)
 if d['kind']!='logical':continue
 for p in d['models']:
  if p['name']!='calibration_red4' or not native:continue
  observed=camera_observation(pose(d['camera_pose']),pose(p['pose']),d['stamp']);i=bisect.bisect_left(times,d['stamp']);candidates=native[max(0,i-1):min(len(native),i+1)];stamp,n=min(candidates,key=lambda x:abs(x[0]-d['stamp']))
  rows.append(dict(camera_sim=d['stamp'],native_sim=stamp,time_difference_s=stamp-d['stamp'],camera_world=observed,native=n,position_difference_m=math.dist(observed['position'],[n.get('position',{}).get(k,0) for k in 'xyz'])))
result=dict(scope='simulation sensor vs native nearest-time comparison, not real-robot calibration or certified error envelope',samples=len(rows),max_time_difference_s=max(abs(x['time_difference_s']) for x in rows),max_position_difference_m=max(x['position_difference_m'] for x in rows),rows=rows)
(r/'fixture-pose-comparison.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='rows'})
