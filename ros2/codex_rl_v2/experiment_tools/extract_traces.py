import re,json,csv,hashlib,bisect,statistics
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';split=json.loads((R/'data/ROUND_SPLIT.json').read_text())['rounds'];rows=[];events=[];manifest={};batches=[]
def sec(d):return float(d.get('sec',0))+float(d.get('nsec',0))/1e9
def savecsv(path,rs):
 fields=list(dict.fromkeys(k for r in rs for k in r))
 with path.open('w') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rs)
for case in sorted((I/'runs').iterdir()):
 run=case/'instance-1';meta=json.loads((case/'metadata.json').read_text());policy=meta['feed_policy'];parts={f'part_{i}':{'part':f'part_{i}','events':{},'picks':[],'verify':[],'retry':[],'issues':[]} for i in range(1,9)}
 raw=[]
 for num,line in enumerate((run/'launch.log').read_text().splitlines(),1):
  m=re.match(r'.*?\[(\d{10}\.\d+)\] \[([^\]]+)\]: (.*)',line)
  if m:raw.append({'wall':float(m[1]),'node':m[2],'text':m[3],'line':num})
 phases=list(csv.DictReader((run/'phase-events.csv').open()));queues={}
 for p in phases:queues.setdefault((p['part'],p['task_id'],p['state']),[]).append(p)
 stats=[(x['wall'],sec(x['stats']['sim_time'])) for x in map(json.loads,(run/'stats.jsonl').read_text().splitlines())];sx=[x[0] for x in stats]
 def sim_est(w):
  j=bisect.bisect_left(sx,w)
  if j==0 or j==len(sx):return None,None
  a,b=stats[j-1],stats[j];f=(w-a[0])/(b[0]-a[0]);return a[1]+f*(b[1]-a[1]),b[1]-a[1]
 def event(part,kind,line,sim=None,quality='native_stats_interpolated'):
  est,span=sim_est(line['wall']);e={'case':case.name,'split':split[case.name],'policy':policy,'part':part,'kind':kind,'wall':line['wall'],'sim':est,'ros_observed_sim':sim,'sim_source':quality,'native_bracket_sim_span':span,'log_line':line['line'],'text':line['text']}
  events.append(e)
  if part:parts[part]['events'].setdefault(kind,[]).append(e)
  return e
 current={};feeder=None
 for l in raw:
  t=l['text'];node=l['node'];p=None
  m=re.search(r'== (task_\d+) 开始: (part_\d+) \(类别 (\w+)\)',t)
  if m:
   current[node]=m[2];parts[m[2]].update(task=m[1],color=m[3],arm='arm2' if node=='pick_place_relay' else 'arm1');event(m[2],'task_begin',l)
  m=re.search(r'已请求 (\d+)/8\([^<]+<- (\w+)',t)
  if m:feeder=f'part_{m[1]}';parts[feeder]['color']=m[2];event(feeder,'request',l)
  m=re.search(r'生成工件 (part_\d+) \((\w+)\): 成功',t)
  if m:event(m[1],'spawn',l)
  if node=='task_dispatcher':
   if '停推进: 工件' in t:event(feeder,'retract_start',l)
   if '推板已回到起点附近' in t:event(feeder,'pusher_home',l)
   if '派发挂起:' in t:event(feeder,'dispatch_wait_notice',l)
   m=re.search(r'取件区释放: (part_\d+)',t)
   if m:event(m[1],'zone_released_dispatcher',l)
   m=re.search(r'(?:派发任务|并联派发): (part_\d+)',t)
   if m:event(m[1],'dispatch',l)
   if 'WAIT_METRIC {' in t:
    d=json.loads(t.split('WAIT_METRIC ',1)[1])
    if d['label']=='first_feed':parts['part_1']['request_sim_exact']=d['sim']
  if node in current:
   p=current[node]
   m=re.search(r'(task_\d+) (part_\d+): (PICKING|CARRYING|ZONE_CLEAR|PLACING|COMPLETED|FAILED)',t)
   if m:
    kind=m[3]
    if '待收尾后上报' in t:event(p,'place_finished_deferred',l)
    else:
     q=queues.get((m[2],m[1],kind),[]);qrow=q.pop(0) if q else None
     e=event(p,kind,l,float(qrow['stamp']) if qrow else None)
     if kind=='PICKING':parts[p]['picks'].append(e)
   if '夹取验证通过:' in t or '夹取验证失败:' in t:parts[p]['verify'].append(event(p,'verify',l))
   m=re.search(r'第 (\d+) 次尝试失败\(([^)]+)\)',t)
   if m:parts[p]['retry'].append(dict(event(p,'retry_after_recovery',l),attempt=int(m[1]),reason=m[2]))
   if t.startswith('return_standby:'):
    if '规划完成' in t:event(p,'return_controller_start',l)
    if '控制器完成' in t:event(p,'return_controller_done',l)
   if t.startswith('standby:'):
    if '规划完成' in t:event(p,'initial_standby_start',l)
    if '控制器完成' in t:event(p,'initial_standby_done',l)
   if any(s in t for s in ('返回待机位失败','待机位运动失败','控制器未成功','控制器执行超时','动作未成功')):parts[p]['issues'].append(event(p,'execution_issue',l))
  if 'Aborted due to goal_time' in t or "Found a contact between" in t:event(None,'controller_or_planning_collision',l)
 case_rows=[]
 for part,p in parts.items():
  ev=p['events'];get=lambda name:ev.get(name,[None])[0]
  required=('request','spawn','retract_start','pusher_home','PICKING','ZONE_CLEAR','place_finished_deferred','COMPLETED')
  missing=[k for k in required if get(k) is None]
  assert not missing,(case.name,part,missing)
  assert len(p['picks'])==len(p['retry'])+1,(case.name,part,'attempt log mismatch')
  if 'request_sim_exact' in p:get('request').update(ros_observed_sim=p['request_sim_exact'])
  row={'case':case.name,'policy':policy,'split':split[case.name],'part':part,'color':p['color'],'arm':p['arm'],'occurrence':sum(x['color']==p['color'] for x in case_rows),'retries':len(p['retry']),'retry_reasons':'|'.join(x['reason'] for x in p['retry']),'pusher_velocity_zero_evidence':False,'dispatch_present':get('dispatch') is not None}
  named={'request':get('request'),'spawn':get('spawn'),'retract_start':get('retract_start'),'pusher_home':get('pusher_home'),'dispatch':get('dispatch'),'picking_first':p['picks'][0],'picking_success_attempt':p['picks'][-1],'zone_clear':get('zone_released_dispatcher'),'zone_status':get('ZONE_CLEAR'),'placing':get('PLACING'),'place_finished':get('place_finished_deferred'),'return_start':get('return_controller_start'),'return_done':get('return_controller_done'),'completed':get('COMPLETED')}
  for k,e in named.items():
   for f in ('wall','sim','ros_observed_sim','sim_source','log_line'):row[k+'_'+f]=e[f] if e else None
  durations={'request_spawn':('request','spawn'),'advance':('spawn','retract_start'),'retract':('retract_start','pusher_home'),'home_to_pick':('pusher_home','picking_first'),'feed_to_pick':('request','picking_first'),'retry_prefix':('picking_first','picking_success_attempt'),'pick_nominal':('picking_success_attempt','zone_clear'),'pick_all':('picking_first','zone_clear'),'post':('zone_clear','completed'),'place_stage':('zone_clear','place_finished'),'return_tail':('place_finished','completed'),'task':('picking_first','completed')}
  for label,(a,b) in durations.items():
   for clock in ('wall','sim'):row[label+'_'+clock+'_s']=named[b][clock]-named[a][clock]
  row['verify_successes']=sum('验证通过:' in e['text'] for e in p['verify']);row['verify_failures']=sum('验证失败:' in e['text'] for e in p['verify']);row['execution_issues']=len(p['issues']);row['source_log']=str(run/'launch.log');case_rows.append(row);rows.append(row)
 # Critical chain identity: each new feed follows prior zone release.
 first=case_rows[0];last=case_rows[-1];batch={'case':case.name,'policy':policy,'split':split[case.name],'feed_order':[p['color'] for p in case_rows]}
 for clock in ('wall','sim'):
  gaps=sum(case_rows[i]['request_'+clock]-case_rows[i-1]['zone_clear_'+clock] for i in range(1,8));feed=sum(p['feed_to_pick_'+clock+'_s'] for p in case_rows);pick=sum(p['pick_nominal_'+clock+'_s'] for p in case_rows);retry=sum(p['retry_prefix_'+clock+'_s'] for p in case_rows);tail=max(p['completed_'+clock] for p in case_rows)-last['zone_clear_'+clock]
  total=max(p['completed_'+clock] for p in case_rows)-first['request_'+clock]
  assert abs(total-(gaps+feed+pick+retry+tail))<1e-5,(case.name,clock)
  batch[clock]={'feed':feed,'pick_nominal':pick,'retry_prefix':retry,'request_gaps':gaps,'terminal_tail':tail,'total':total,'total_without_recorded_retry_prefix':total-retry}
 batch['retries']=sum(p['retries'] for p in case_rows);batch['startup_wall_s']=first['request_wall']-meta['start'];batch['csv_measurement_wall_s']=json.loads((case/'analysis.json').read_text())['experiment_wall_s'];batch['recorder_tail_wall_s']=batch['csv_measurement_wall_s']-batch['startup_wall_s']-batch['wall']['total'];batches.append(batch)
 residuals=[]
 for e in events:
  if e['case']==case.name and e['ros_observed_sim'] is not None:
   est,span=sim_est(e['wall']);residuals.append(est-e['ros_observed_sim'])
 manifest[case.name]={'split':split[case.name],'source_hashes':{str(run/f):hashlib.sha256((run/f).read_bytes()).hexdigest() for f in ('launch.log','stats.jsonl','phase-events.csv','results.csv','ros-probe.jsonl')},'native_interpolation_vs_ROS_stamp_n':len(residuals),'native_minus_ROS_stamp_median_s':statistics.median(residuals),'native_minus_ROS_stamp_max_abs_s':max(map(abs,residuals)),'note':'Durations use one consistent native-stats interpolated clock, avoiding artificial negative gaps from mixing stale ROS stamps with native time. Exact observed ROS stamps are separate columns. ROS stamps themselves are observed /clock snapshots. Other sim times are native-stats linear estimates, not exact event timestamps; raw brackets and provenance retained.'}
savecsv(R/'CALIBRATION_DATA.csv',rows);savecsv(R/'data/TRACE_EVENTS.csv',events);(R/'data/parts.json').write_text(json.dumps(rows,indent=2));(R/'data/batches.json').write_text(json.dumps(batches,indent=2));(R/'data/SOURCE_MANIFEST.json').write_text(json.dumps(manifest,indent=2));print('Extracted',len(rows),'parts',len(events),'events')
for p in batches:
 if p['split']!='diagnostic_contact_only':print(p['case'],p['retries'],{k:round(v,3) for k,v in p['sim'].items()})
