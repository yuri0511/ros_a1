import sys,json,re,csv,statistics,math
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2')
def q(a,p):
 a=sorted(a)
 if not a:return None
 index=(len(a)-1)*p;lo=math.floor(index);hi=math.ceil(index);fraction=index-lo
 return a[lo]*(1-fraction)+a[hi]*fraction
def sec(x):return float(x.get('sec',0))+float(x.get('nsec',0))/1e9
def analyze(case):
 meta=json.loads((case/'metadata.json').read_text());summary=json.loads((case/'summary.json').read_text()) if (case/'summary.json').exists() else {};out={'case':str(case),'label':meta['label'],'target':meta['rate'],'step_ms':meta['step_ms'],'fast_waits':meta['fast_waits'],'contact_mode':meta['contact_mode'],'threads':meta['threads'],'probe_mode':meta.get('probe_mode','continuous-high-frequency-pilot'),'rmw':meta.get('rmw','rmw_fastrtps_cpp'),'egl':meta.get('egl','0'),'n':meta['instances'],'start':meta['start'],'reason':summary.get('reason','running'),'instances':[]};valid=0;endtimes=[]
 for ri in meta['runs']:
  run=Path(ri['run']);log=(run/'launch.log').read_text(errors='replace');rows=list(csv.DictReader((run/'results.csv').open())) if (run/'results.csv').exists() else []
  stamps=re.findall(r'\[(\d{10}\.\d+)\] \[result_recorder\]: part_\d+\[.*? -> ',log);completion=float(stamps[7]) if len(stamps)>=8 else None
  good=len(rows)==8 and {x['part_id'] for x in rows}=={f'part_{j}' for j in range(1,9)} and all(x['result']=='PLACED' and x['final_bin']==x['target_bin'] for x in rows) and all(sum(x['final_bin']==b for x in rows)==2 for b in ('bin_blue','bin_yellow','bin_red','bin_green'))
  if good:valid+=1
  if completion:endtimes.append(completion)
  raw=[json.loads(x) for x in (run/'stats.jsonl').read_text().splitlines() if x] if (run/'stats.jsonl').exists() else []
  end=completion or summary.get('measure_end',raw[-1]['wall'] if raw else 0)
  whole=[x for x in raw if x['wall']<=end and not x['stats'].get('paused',False)]
  fullrtf=(sec(whole[-1]['stats']['sim_time'])-sec(whole[0]['stats']['sim_time']))/(sec(whole[-1]['stats']['real_time'])-sec(whole[0]['stats']['real_time'])) if len(whole)>1 and sec(whole[-1]['stats']['real_time'])>sec(whole[0]['stats']['real_time']) else None
  a=[x for x in raw if ri['start']+30<=x['wall']<=end and not x['stats'].get('paused',False)]
  if len(a)>1:
   ds=sec(a[-1]['stats']['sim_time'])-sec(a[0]['stats']['sim_time']);dr=sec(a[-1]['stats']['real_time'])-sec(a[0]['stats']['real_time']);actual=ds/dr if dr>0 else None
   win=[];old=a[0]
   for x in a:
    if x['wall']-old['wall']>=5:
     dreal=sec(x['stats']['real_time'])-sec(old['stats']['real_time']);dsim=sec(x['stats']['sim_time'])-sec(old['stats']['sim_time'])
     if dreal>0 and dsim>=0:win.append(dsim/dreal)
     old=x
  else:actual=None;win=[]
  probes=[json.loads(x) for x in (run/'ros-probe.jsonl').read_text().splitlines() if x] if (run/'ros-probe.jsonl').exists() else []
  status=[x for x in probes if x.get('type')=='status' and ri['start']+30<x['wall']<=end]
  iso={'ros_foreign':sorted({y for x in status for y in x['ros_markers'] if y!=run.name.split('-')[-1]}),'gz_foreign':sorted({y for x in status for y in x['gz_markers'] if y!=run.name.split('-')[-1]}),'clock_publisher_max':max((len(x['clock_publishers']) for x in status),default=0),'controller_managers_max':max((x['controllers'] for x in status),default=0)}
  sensors={}
  for k in ('rgb','gz_rgb','logical','joints'):
   rates={}
   if len(status)>1:
    count=status[-1]['counts'].get(k,0)-status[0]['counts'].get(k,0);dt=status[-1]['wall']-status[0]['wall'];st=status[-1].get('gz_clock',status[-1]['clock'])-status[0].get('gz_clock',status[0]['clock']);rates={'wall_hz':count/dt,'sim_hz':count/st if st>0 else None}
   ages=[x['sim_age'] for x in probes if x.get('type')==k and 'sim_age' in x and ri['start']+30<x['wall']<=end]
   sensors[k]={**rates,'sim_age_samples':len(ages),'sim_age_median':statistics.median(ages) if ages else None,'sim_age_p95':q(ages,.95),'negative_age_samples':sum(v<0 for v in ages)}
  corelines=[]
  for line in log.splitlines():
   tm=re.search(r'\[(\d{10}\.\d+)\]',line)
   if not tm or float(tm[1])<=end:corelines.append(line)
  corelog='\n'.join(corelines)
  if (run/'latency-window.jsonl').exists():
   latency=[json.loads(x) for x in (run/'latency-window.jsonl').read_text().splitlines() if x]
   for kind in ('rgb','joints'):
    vals=[x['sim_age'] for x in latency if x['type']==kind and x['clock']>0]
    sensors[kind]['sim_age_samples']=len(vals);sensors[kind]['sim_age_median']=statistics.median(vals) if vals else None;sensors[kind]['sim_age_p95']=q(vals,.95)
   if (run/'latency-window-summary.json').exists():
    lm=json.loads((run/'latency-window-summary.json').read_text());j=[x for x in latency if x['type']=='joints' and x['clock']>0]
    if len(j)>1:sensors['joints'].update(wall_hz=lm['counts'].get('joints',0)/(lm['end']-lm['start']),sim_hz=(len(j)-1)*10/(j[-1]['stamp']-j[0]['stamp']))
  plan=[];controllers=[];pt=None;ct={}
  for line in corelines:
   t=re.search(r'\[(\d{10}\.\d+)\]',line)
   if not t:continue
   t=float(t[1])
   if 'Calling Planner' in line:pt=t
   if 'Motion plan was computed successfully' in line and pt:plan.append(t-pt);pt=None
   m=re.search(r'\[(arm2_joint_trajectory_controller|joint_trajectory_controller)\]',line)
   if m:
    if 'Received new action goal' in line:ct[m[1]]=t
    if 'Goal reached, success!' in line and m[1] in ct:controllers.append(t-ct.pop(m[1]))
  events=[]
  for line in corelines:
   if '[task_dispatcher]' not in line:continue
   tm=re.search(r'\[(\d{10}\.\d+)\]',line)
   if not tm:continue
   lock=re.search(r'(?:派发任务|并联派发): (part_\d+).*取件区上锁',line);release=re.search(r'取件区释放: (part_\d+)',line)
   if lock:events.append((float(tm[1]),'lock',lock[1]))
   if release:events.append((float(tm[1]),'release',release[1]))
  owner=None;violations=[]
  for tm,kind,part in sorted(events):
   if kind=='lock':
    if owner:violations.append([tm,owner,part])
    owner=part
   elif owner==part:owner=None
   else:violations.append([tm,'unexpected release',part])
  step_values=sorted({sec(x['stats'].get('step_size',{})) for x in a if not x['stats'].get('paused',False)})
  iteration_consistent=all(abs((int(y['stats'].get('iterations',0))-int(x['stats'].get('iterations',0)))*(float(meta['step_ms'])/1000)-(sec(y['stats']['sim_time'])-sec(x['stats']['sim_time'])))<1e-7 for x,y in zip(a,a[1:]))
  spawned=dict(re.findall(r'生成工件 (part_\d+) \((\w+)\): 成功',log));vision=dict(re.findall(r'来料识别完成: (part_\d+) -> (\w+)',log));mismatch=[p for p in vision if p in spawned and vision[p]!=spawned[p]]
  item={'path':str(run),'successful':good,'row_count':len(rows),'results':rows,'completion':completion,'wall_s':completion-ri['start'] if completion else None,'actual_rtf':actual,'rtf_from_first_stats':fullrtf,'rtf_5s_p10':q(win,.1),'rtf_5s_p90':q(win,.9),'rtf_5s_min':min(win) if win else None,'rtf_5s_max':max(win) if win else None,'isolation':iso,'interlock_event_count':len(events),'interlock_violations':violations,'step_values':step_values,'iteration_step_consistent':iteration_consistent,'sensors':sensors,'retries':len(re.findall(r'第 \d+ 次尝试失败',corelog)),'vision_mismatches':mismatch,'planning_n':len(plan),'planning_median_s':statistics.median(plan) if plan else None,'planning_p95_s':q(plan,.95),'controller_n':len(controllers),'controller_median_wall_s':statistics.median(controllers) if controllers else None,'controller_p95_wall_s':q(controllers,.95),'control_errors':re.findall(r'.*(?:ABORTED|PATH_TOLERANCE_VIOLATED|GOAL_TOLERANCE_VIOLATED|Controller.*failed|轨迹.*失败).*',corelog)}
  out['instances'].append(item)
 systems=[json.loads(x) for x in (case/'system.jsonl').read_text().splitlines() if x];end=max(endtimes) if len(endtimes)==meta['instances'] else summary.get('measure_end',systems[-1]['wall']);steady=[x for x in systems if meta['start']+30<x['wall']<=end]
 lifecycle=summary.get('end',end)-meta['start']
 out.update(successful_batches=valid,lifecycle_wall_s=lifecycle,lifecycle_episodes_per_hour=3600*valid/lifecycle if valid else 0,experiment_wall_s=end-meta['start'],episodes_per_hour=3600*valid/(end-meta['start']) if valid else 0,cpu_mean_percent=statistics.mean(x['cpu']['cpu'] for x in steady) if steady else None,cpu_p95_percent=q([x['cpu']['cpu'] for x in steady],.95),min_mem_available_gib=min(x['mem_kib']['MemAvailable']/1048576 for x in systems),swapout_delta=systems[-1]['vm']['pswpout']-systems[0]['vm']['pswpout'],oom_delta=systems[-1]['vm']['oom_kill']-systems[0]['vm']['oom_kill'])
 quality=json.loads((root/'data-quality.json').read_text()) if (root/'data-quality.json').exists() else {}
 out['performance_comparison_valid']=out['label'] not in quality.get('excluded_from_clean_performance_comparison',[])
 out['quality_notes']=quality.get('reason','') if not out['performance_comparison_valid'] else ''
 (case/'analysis.json').write_text(json.dumps(out,indent=2,ensure_ascii=False));return out
if __name__=='__main__':
 cases=[Path(sys.argv[1])] if len(sys.argv)>1 else [p for p in (root/'runs').iterdir() if (p/'metadata.json').exists()]
 out=[analyze(p) for p in cases]
 (root/'comparison.json').write_text(json.dumps(out,indent=2,ensure_ascii=False))
 for x in out:print(x['label'],x['reason'],'success',x['successful_batches'],'time',round(x['experiment_wall_s'],1),'ep/h',round(x['episodes_per_hour'],2),'RTF',[round(i['actual_rtf'],3) if i['actual_rtf'] else None for i in x['instances']],'CPU',round(x['cpu_mean_percent'] or 0,1))
