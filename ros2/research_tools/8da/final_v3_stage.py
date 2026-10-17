import os,time,json,subprocess,gzip,shutil
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b')
def run(label,n,contact=False):
 env=os.environ.copy()
 for k in ('PERF_CONTACT','PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','FASTDDS_BUILTIN_TRANSPORTS','LP_NUM_THREADS'):env.pop(k,None)
 env['PERF_LP_THREADS']='2'
 if contact:env['PERF_CONTACT']='1'
 print('BEGIN',label,flush=True)
 subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py {label} 1 true {n} {900 if n>1 or contact else 650}'],env=env,check=True)
 case=Path((root/'latest-case.txt').read_text())
 for name in ('analyze.py','extra_analysis.py'):subprocess.run(['python3',str(root/'tools'/name),str(case)],check=True)
 a=json.loads((case/'analysis.json').read_text());a['extra']=json.loads((case/'extra-analysis.json').read_text())
 if contact:
  r=case/'instance-1';src=r/'contacts.csv';e=a['extra'][0]
  subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(src),str(r/'contacts-all-summary.json')],check=True)
  if e['batch_first_picking_sim'] is not None and e['batch_last_terminal_sim'] is not None:
   subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(src),str(r/'contacts-task-summary.json'),str(e['batch_first_picking_sim']),str(e['batch_last_terminal_sim'])],check=True)
  with src.open('rb') as f,gzip.open(str(src)+'.gz','wb',compresslevel=4) as z:shutil.copyfileobj(f,z)
 print('END',label,flush=True);return a

def good(a):return a['reason']=='completed' and a['successful_batches']==a['n'] and not a['swapout_delta'] and not a['oom_delta'] and not any(i['interlock_violations'] or i['vision_mismatches'] or i['isolation']['ros_foreign'] or i['isolation']['gz_foreign'] for i in a['instances'])
one=run('B3-1ms-event-repeat1',1);assert good(one),'v3 single repeat1 failed'
two=run('B3-1ms-event-repeat2',1);assert good(two),'v3 single repeat2 failed'
contact=run('B3-contact-1ms-event',1,True);assert good(contact),'v3 contact task failed'
co=json.loads((Path(contact['case'])/'instance-1/contacts-task-summary.json').read_text());assert co['bad_rows']==0 and not co['categories'].get('interarm'),'v3 contact safety gate failed'
low_rate=7200/(one['experiment_wall_s']+two['experiment_wall_s'])
dual=run('P2-v3-1ms-event',2);parallel=[]
if good(dual):parallel.append(dual)
eligible=good(dual) and dual['episodes_per_hour']>low_rate and dual['cpu_mean_percent']<85 and dual['cpu_p95_percent']<90 and not any(e['controller_abort_count'] for e in dual['extra'])
(root/'three-instance-gate.json').write_text(json.dumps({'eligible':eligible,'dual_case':dual['case'],'dual_ep_h':dual['episodes_per_hour'],'same_version_single_pooled_ep_h':low_rate,'dual_cpu_mean':dual['cpu_mean_percent'],'dual_cpu_p95':dual['cpu_p95_percent']},indent=2))
if eligible:
 triple=run('P3-v3-1ms-event',3)
 if good(triple):parallel.append(triple)
else:print('THREE SKIPPED: measured resource/reliability/throughput gate not met',flush=True)
if parallel:
 best=max(parallel,key=lambda a:a['episodes_per_hour']);repeat=run(f'P{best["n"]}-v3-repeat-1ms-event',best['n'])
 chosen={'chosen':best['case'],'repeat':repeat['case'],'repeat_valid':good(repeat)}
 if not good(repeat) and best['n']==3 and good(dual):
  fallback=run('P2-v3-repeat-1ms-event',2);chosen['fallback_repeat']=fallback['case'];chosen['fallback_valid']=good(fallback)
 (root/'parallel-repeat-result.json').write_text(json.dumps(chosen,indent=2))
(root/'FINAL_EXPERIMENTS_DONE').touch();print('ALL V3 FINAL EXPERIMENTS FINISHED',flush=True)
