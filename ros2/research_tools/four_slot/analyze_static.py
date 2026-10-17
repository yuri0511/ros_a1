"""Compare simultaneous static model poses; never treats this as part localization calibration."""
import gzip,json,sys
from pathlib import Path
from types import SimpleNamespace as NS
from a1_sorting_core.retained_scene_sync import camera_observation

def pose(d):return NS(position=NS(**{k:float(d.get('position',{}).get(k,0)) for k in 'xyz'}),orientation=NS(**{k:float(d.get('orientation',{}).get(k,0)) for k in 'xyzw'}))
def main():
 r=Path(sys.argv[1]);native={};frames=[]
 for l in gzip.open(r/'raw-ros.jsonl.gz','rt'):
  d=json.loads(l)
  if d['kind']=='native_pose':
   for p in d['message'].get('pose',[]):native[p['name']]=p
 for l in (r/'events.jsonl').open():
  d=json.loads(l)
  if d['kind']=='logical':frames.append(d)
 import math
 rows=[]
 for name in ('bin_blue','bin_yellow','bin_red','bin_green','tabletop','ur'):
  errs=[];last=None
  for d in frames:
   for p in d['models']:
    if p['name']==name and name in native:
     last=camera_observation(pose(d['camera_pose']),pose(p['pose']),d['stamp'])
     target=native[name]['position'];errs.append(math.dist(last['position'],[target.get(k,0) for k in 'xyz']))
  rows.append(dict(name=name,samples=len(errs),max_position_error_m=max(errs) if errs else None,last_camera_world=last,native=native.get(name)))
 out=dict(scope='static_model_origins_only_not_grasp_or_released_part_calibration',rows=rows,zero_feed=json.loads((r/'observed-status.json').read_text())['spawn_requests']==[],native_models=len(native))
 (r/'coordinate-comparison.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':main()
