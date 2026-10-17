import json,csv,re,math,statistics,sys,hashlib
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2')
case=Path(sys.argv[1]);meta=json.loads((case/'metadata.json').read_text());out=[]
for ri in meta['runs']:
 run=Path(ri['run']);log=(run/'launch.log').read_text(errors='replace');metrics=[];targets={}
 cutoff=json.loads((case/'summary.json').read_text()).get('measure_end',1e100)
 runtime=[]
 for line in log.splitlines():
  stamp=re.search(r'\[(\d{10}\.\d+)\]',line)
  if not stamp or float(stamp[1])<=cutoff:runtime.append(line)
 runtime_log='\n'.join(runtime)
 for line in log.splitlines():
  for tag in ('WAIT_METRIC','POSE_TARGET','STARTUP_READY'):
   if tag+' {' not in line:continue
   try:obj=json.loads(line.split(tag+' ',1)[1])
   except ValueError:continue
   tm=re.search(r'\[(\d{10}\.\d+)\]',line);obj['wall_epoch']=float(tm[1]) if tm else None
   if tag=='WAIT_METRIC':metrics.append(obj)
   if tag=='POSE_TARGET':targets[obj['part']]=obj
 phases=[json.loads(s) for s in (run/'ros-probe.jsonl').read_text().splitlines() if s]
 events=[x for x in phases if x.get('type')=='task'];arrivals=[x for x in phases if x.get('type')=='arrival']
 with (run/'phase-events.csv').open('w') as f:
  fields=['wall','stamp','part','task_id','arm','state','detail'];w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(events)
 sums={}
 for label in ('first_feed','dispatch_delay_path','grasp_extra','release_settle','drop_return_branch'):
  a=[x for x in metrics if x['label']==label];vals=[x['wall'] for x in a if 'wall' in x]
  sums[label]={'count':len(a),'wall_sum_s':sum(vals),'wall_median_s':statistics.median(vals) if vals else None,'records':a}
 final=json.loads((run/'tracked-positions.json').read_text()) if (run/'tracked-positions.json').exists() else {}
 reported={}
 for line in log.splitlines():
  match=re.search(r'(part_\d+)\[.*? -> (PLACED|MISPLACED|OUTSIDE|FAILED).*?观测=\(([-\d.]+),\s*([-\d.]+)\)',line)
  if match:reported[match[1]]={'xy':[float(match[3]),float(match[4])],'result':match[2]}
 result_map={row['part_id']:row['result'] for row in csv.DictReader((run/'results.csv').open())} if (run/'results.csv').exists() else {}
 errors=[]
 for part,target in targets.items():
  obs=final.get('parts',{}).get(part)
  if obs:
   xyz=obs['xyz'];source='native last pose before original timed recycle';precision=None
  elif part in reported:
   xyz=[*reported[part]['xy'],None];source='original recorder XY at completion (rounded 0.001m)';precision=.001
  else:continue
  delta=[xyz[i]-target['xyz'][i] if xyz[i] is not None else None for i in range(3)]
  errors.append({'part':part,'task_result':result_map.get(part),'bin':target['bin'],'target_xyz':target['xyz'],'observed_xyz':xyz,'delta_xyz_m':delta,'xy_error_m':math.hypot(*delta[:2]),'xyz_error_m':math.sqrt(sum(x*x for x in delta)) if all(x is not None for x in delta) else None,'source':source,'coordinate_resolution_m':precision})
 (run/'wait-metrics.json').write_text(json.dumps(sums,indent=2));(run/'placement-errors.json').write_text(json.dumps(errors,indent=2))
 nonzero=[x for x in events if x.get('stamp',0)>0];starts=[x['stamp'] for x in nonzero if x['state']=='PICKING'];ends=[x['stamp'] for x in nonzero if x['state'] in ('COMPLETED','FAILED')]
 first_feed_sim=next((x.get('sim') for x in metrics if x['label']=='first_feed'),None)
 out.append({'first_feed_sim':first_feed_sim,'full_batch_sim_s':max(ends)-first_feed_sim if ends and first_feed_sim is not None else None,'world_last_terminal_sim':max(ends) if ends else None,'instance':run.name,'batch_first_picking_sim':min(starts) if starts else None,'batch_last_terminal_sim':max(ends) if ends else None,'batch_last_completed_sim':max((x['stamp'] for x in nonzero if x['state']=='COMPLETED'),default=None),'batch_task_sim_s':max(ends)-min(starts) if starts and ends else None,'waits':{k:{q:v[q] for q in ('count','wall_sum_s','wall_median_s')} for k,v in sums.items()},'placement_count':sum(x['task_result']=='PLACED' for x in errors),'max_xy_error_m':max((x['xy_error_m'] for x in errors if x['task_result']=='PLACED'),default=None),'mean_xy_error_m':statistics.mean(x['xy_error_m'] for x in errors if x['task_result']=='PLACED') if any(x['task_result']=='PLACED' for x in errors) else None,'placement_errors':errors,'rule_explicit_off_logged':'退回谁闲推谁: 没有填写 Q 表路径' in log,'q_policy_load_success':'已加载 Q 表:' in log,'controller_abort_count':len(re.findall(r'\bAborted\b',runtime_log)),'controller_execution_error_lines':len(re.findall(r'控制器未成功|控制器执行超时',runtime_log)),'return_standby_failures':len(re.findall(r'返回待机位失败',runtime_log)),'controller_runtime_messages':[line for line in runtime_log.splitlines() if re.search(r'控制器未成功|控制器执行超时|控制器状态 \d+|\bAborted\b|PATH_TOLERANCE_VIOLATED|GOAL_TOLERANCE_VIOLATED',line)]})
(case/'extra-analysis.json').write_text(json.dumps(out,indent=2));print(case.name,[(x['batch_task_sim_s'],x['placement_count'],x['max_xy_error_m']) for x in out])
