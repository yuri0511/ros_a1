import os,time,json,subprocess
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b')
while not (root/'CONTACT_MATRIX_DONE').exists():time.sleep(5)
bc=next((root/'runs').glob('*B-contact-1ms-event'));ba=json.loads((bc/'analysis.json').read_text());contacts=json.loads((bc/'instance-1/contacts-task-summary.json').read_text())
assert ba['successful_batches']==1 and contacts['bad_rows']==0 and not contacts['categories'].get('interarm'), 'B contact gate did not pass'
print('B CONTACT GATE PASSED; step2 rejected after failures',flush=True)
(root/'parallel-gate.json').write_text(json.dumps({'candidate':'1ms+event','step2_rejected':True,'contact_gate':'B8/8,badrows0,nointerarm; measured baseline-like contact categories','low_matrix':'A/B successful; C1/8,D0/8'},indent=2))
def run(label,n):
 env=os.environ.copy()
 for k in ('PERF_CONTACT','PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','FASTDDS_BUILTIN_TRANSPORTS','LP_NUM_THREADS'):env.pop(k,None)
 env['PERF_LP_THREADS']='2';print('BEGIN',label,flush=True)
 subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py {label} 1 true {n} {900 if n>1 else 650}'],env=env,check=True)
 case=Path((root/'latest-case.txt').read_text())
 subprocess.run(['python3',str(root/'tools/analyze.py'),str(case)],check=True)
 subprocess.run(['python3',str(root/'tools/extra_analysis.py'),str(case)],check=True)
 a=json.loads((case/'analysis.json').read_text());a['extra']=json.loads((case/'extra-analysis.json').read_text());print('END',label,flush=True);return a

def good(a):return a['reason']=='completed' and a['successful_batches']==a['n'] and not a['swapout_delta'] and not a['oom_delta'] and not any(i['interlock_violations'] or i['vision_mismatches'] or i['isolation']['ros_foreign'] or i['isolation']['gz_foreign'] for i in a['instances'])
repeat=run('B-repeat-1ms-event',1)
assert good(repeat),'Single candidate repeat not valid; review instead of expanding'
dual=run('P2-1ms-event',2)
parallel=[]
if good(dual):parallel.append(dual)
eligible=good(dual) and dual['episodes_per_hour']>repeat['episodes_per_hour'] and dual['cpu_mean_percent']<85 and dual['cpu_p95_percent']<90 and not any(x['controller_abort_count'] for x in dual['extra'])
(root/'three-instance-gate.json').write_text(json.dumps({'eligible':eligible,'dual_case':dual['case'],'dual_ep_h':dual['episodes_per_hour'],'single_repeat_ep_h':repeat['episodes_per_hour'],'dual_cpu_mean':dual['cpu_mean_percent'],'dual_cpu_p95':dual['cpu_p95_percent']},indent=2))
if eligible:
 triple=run('P3-1ms-event',3)
 if good(triple):parallel.append(triple)
else:print('THREE SKIPPED: resource/reliability/throughput gate not met',flush=True)
if parallel:
 best=max(parallel,key=lambda a:a['episodes_per_hour'])
 # Recheck highest valid complete parallel setting rather than trusting one run.
 again=run(f'P{best["n"]}-repeat-1ms-event',best['n'])
 (root/'parallel-repeat-result.json').write_text(json.dumps({'chosen':best['case'],'repeat_case':again['case'],'repeat_valid':good(again)},indent=2))
(root/'FINAL_EXPERIMENTS_DONE').touch();print('ALL FINAL EXPERIMENTS FINISHED',flush=True)
