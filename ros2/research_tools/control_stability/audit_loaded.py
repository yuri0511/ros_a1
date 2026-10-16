"""Streaming, independent audit of a saved Rule run. No simulation or writes to raw data."""
from pathlib import Path
import json,gzip,csv,collections,re,sys,math
R=Path('/home/yu/yuri_gazebo_performance/control_stability');C=Path(sys.argv[1]);I=C/'instance-1';events=[];joints=collections.deque(maxlen=6000);ready=[];publishers=set();counts=collections.Counter()
configcase=R/'runs/20261017T030503-empty-gain1/instance-1';configs={}
for arm in ('arm1','arm2'):
 d=json.loads((configcase/(arm+'-result.json')).read_text());configs[arm]=next(x for x in d['records'] if x['kind']=='configuration')
with gzip.open(I/'high-rate.jsonl.gz','rt') as f:
 for line in f:
  o=json.loads(line);counts[o['kind']]+=1
  if o['kind']=='joint_state':joints.append(o);publishers.add(o.get('publisher_gid','not-recorded'))
  if o['kind']!='task_status':continue
  events.append(o)
  if o['state']!='ARM_READY':continue
  d=json.loads(o['detail']);e=d['evidence'];names=configs[o['arm']]['joint_names'];target=configs[o['arm']]['standby'];xs=[x for x in joints if e['last_stamp']-e['stable_span']-1e-9<=x['stamp']<=e['last_stamp']+1e-9];speeds=[];errors=[]
  for x in xs:
   p=dict(zip(x['names'],x['positions']));v=dict(zip(x['names'],x['velocities']));speeds.append(max(abs(v[n]) for n in names));errors.append(max(abs(p[n]-q) for n,q in zip(names,target)))
  ready.append({'part':o['part'],'arm':o['arm'],'wall':o['wall'],'sim':o['stamp'],'evidence':d,'raw_ros_samples':len(xs),'raw_unique_stamps':len(set(x['stamp'] for x in xs)),'raw_ros_max_speed':max(speeds,default=None),'raw_ros_max_position_error':max(errors,default=None),'native_samples':0,'native_max_speed':0.,'native_max_position_error':0.,'native_cmd_peak':0.})
with (I/'native-joints.csv').open() as f:
 for x in csv.DictReader(f):
  t=float(x['sim_sec']);name=x['joint']
  for a in ready:
   e=a['evidence']['evidence'];conf=configs[a['arm']]
   if name not in conf['joint_names'] or not e['last_stamp']-e['stable_span']<=t<=e['last_stamp']:continue
   k=conf['joint_names'].index(name);a['native_samples']+=1;a['native_max_speed']=max(a['native_max_speed'],abs(float(x['v'])));a['native_max_position_error']=max(a['native_max_position_error'],abs(float(x['q'])-conf['standby'][k]));a['native_cmd_peak']=max(a['native_cmd_peak'],abs(float(x.get('velocity_cmd_pre',0))))
for a in ready:
 e=a['evidence']['evidence'];c=a['evidence']['controller_result'];a['validated']=bool(e['ready'] and e['samples']>=50 and e['stable_span']>=.25 and c['success'] and c['status']==4 and c['error_code']==0 and a['evidence']['no_active_goal'] and a['raw_unique_stamps']>=50 and a['raw_ros_max_speed']<=.3 and a['raw_ros_max_position_error']<=.01 and a['native_samples']>=1400 and a['native_max_speed']<=.3 and a['native_max_position_error']<=.01)
text=(I/'launch.log').read_text();lines=text.splitlines();results=[];release=[]
for line in lines:
 if 'CONTROLLER_RESULT ' in line:results.append(json.loads(line.split('CONTROLLER_RESULT ',1)[1]))
 if 'ARM_SLOT_RELEASE ' in line:release.append(json.loads(line.split('ARM_SLOT_RELEASE ',1)[1]))
rows=list(csv.DictReader((I/'results.csv').open()));faults=[x for x in events if x['state'] in ('ARM_FAULT','FAILED','PART_REJECTED')];placed=[x for x in events if x['state']=='PART_PLACED'];zones=[x for x in events if x['state']=='ZONE_CLEAR'];perarm=collections.defaultdict(list)
for o in events:
 if o['state'] in ('PICKING','ARM_READY'):perarm[o['arm']].append((o['state'],o['part']))
slot_order_ok=True
for arm,seq in perarm.items():
 current=None
 for state,part in seq:
  if state=='PICKING':
   if current is not None:slot_order_ok=False
   current=part
  else:
   if current!=part:slot_order_ok=False
   current=None
summary=json.loads((C/'summary.json').read_text());startpick=next((x for x in events if x['state']=='PICKING'),None);lastready=next((x for x in reversed(events) if x['state']=='ARM_READY'),None)
out={'case':str(C),'commit':summary['commit'],'rows':rows,'bin_counts':dict(collections.Counter(x['final_bin'] for x in rows)),'color_counts':dict(collections.Counter(x['category'] for x in rows)),'result_counts':dict(collections.Counter(x['result'] for x in rows)),'events':events,'ready_audits':ready,'all_ready_validated':bool(ready) and all(x['validated'] for x in ready),'faults':faults,'controller_results':results,'controller_failed_results':[x for x in results if not x['success']],'slot_releases':release,'slot_order_ok':slot_order_ok,'part_placed_count':len(placed),'zone_clear_count':len(zones),'source_gids':sorted(publishers),'raw_message_counts':dict(counts),'retry_or_grasp_failure_lines':[l for l in lines if any(s in l for s in ('重试','grasp_not_verified','验证失败'))],'other_error_lines':[l for l in lines if any(s in l for s in ('动作未成功','目标被 move_group 拒绝','执行超时','goal_time'))],'task_wall_seconds':lastready['wall']-startpick['wall'] if lastready and startpick else None,'task_sim_seconds':lastready['stamp']-startpick['stamp'] if lastready and startpick else None,'reason':summary['reason'],'first_picking':startpick,'last_ready':lastready}
(R/'audit'/('loaded-'+C.name+'.json')).write_text(json.dumps(out,indent=2));print(json.dumps({k:out[k] for k in ('case','result_counts','bin_counts','all_ready_validated','slot_order_ok','task_wall_seconds','task_sim_seconds','reason')},indent=2));print('ready',len(ready),'faults',len(faults),'action failures',len(out['controller_failed_results']),'retry logs',len(out['retry_or_grasp_failure_lines']))
