import json,csv,gzip,sys,collections,math
from pathlib import Path
source=Path(sys.argv[1]);dest=Path(sys.argv[2]);window=[float(x) for x in sys.argv[3:5]];pairs={};bad=0;total=0;missing_depth=0;min_t=math.inf;max_t=-math.inf
open_file=gzip.open if source.suffix=='.gz' else open

def category(a,b):
 r=lambda x:2 if 'arm2' in x else (1 if x.startswith('ur::') else 0)
 if r(a)*r(b)==2:return 'interarm'
 text=a+' '+b
 if 'positive_shoulder_box' in text or 'probe_box' in text:return 'deliberate_probe'
 finger='gripper_finger' in text
 if 'part_' in text:return 'finger_part' if finger else 'robot_part'
 if 'bin_' in text or 'wall_' in text:return 'finger_bin' if finger else 'body_bin'
 if 'belt' in text or 'pusher' in text:return 'finger_conveyor' if finger else 'body_conveyor'
 return 'other'
with open_file(source,'rt',newline='') as f:
 for x in csv.DictReader(f):
  try:
   t=float(x['sim_sec']);depth=float(x['depth']) if x['depth'] else 0.;a=x['collision1'];b=x['collision2']
  except (ValueError,KeyError,TypeError):bad+=1;continue
  if window and not window[0]<=t<=window[1]:continue
  if not x['depth']:missing_depth+=1
  total+=1;min_t=min(min_t,t);max_t=max(max_t,t);key=tuple(sorted((a,b)));g=pairs.setdefault(key,{'category':category(a,b),'points':0,'max_depth_m':0.,'times':set(),'deepest':None})
  g['points']+=1;g['times'].add(t)
  if depth>g['max_depth_m']:g['max_depth_m']=depth;g['deepest']={k:x[k] for k in ('sim_sec','x','y','z','depth')}
cats={};out=[]
for key,g in pairs.items():
 ts=sorted(g.pop('times'));segments=[]
 if ts:
  start=last=ts[0]
  for t in ts[1:]:
   if t-last>.0401:segments.append([start,last]);start=t
   last=t
  segments.append([start,last])
 g.update(collisions=list(key),first_sim=ts[0],last_sim=ts[-1],unique_sensor_ticks=len(ts),segments=segments,max_segment_span_s=max((b-a for a,b in segments),default=0.),observed_segment_span_sum_s=sum(b-a for a,b in segments),timestamp_median_gap_s=__import__('statistics').median([b-a for a,b in zip(ts,ts[1:])]) if len(ts)>1 else None)
 out.append(g);c=cats.setdefault(g['category'],{'points':0,'max_depth_m':0.,'pairs':0,'max_segment_span_s':0.})
 c['points']+=g['points'];c['pairs']+=1;c['max_depth_m']=max(c['max_depth_m'],g['max_depth_m']);c['max_segment_span_s']=max(c['max_segment_span_s'],g['max_segment_span_s'])
result={'source':str(source),'window':window,'rows':total,'missing_depth_rows':missing_depth,'bad_rows':bad,'first_contact_sim':min_t if total else None,'last_contact_sim':max_t if total else None,'categories':cats,'pairs':out,'duration_note':'Contact CSV stamps are rounded to 1ms; segment heuristic joins gaps <=40ms. Report observed spans only, not tick_count/assumed_frequency. Effective contact stamps can be 1ms apart despite nominal sensor update settings. Point counts are not independent collisions; spans may bridge unsampled gaps.'}
dest.write_text(json.dumps(result,indent=2));print(source.name,total,'bad',bad,'interarm',cats.get('interarm'))
