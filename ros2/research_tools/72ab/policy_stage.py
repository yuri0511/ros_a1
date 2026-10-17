import os,time,json,subprocess,gzip,shutil
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');initial=Path((R/'latest-case.txt').read_text())
def analyze(case,contact):
 for name in ('analyze.py','extra_analysis.py','policy_analysis.py'):subprocess.run(['python3',str(R/'tools'/name),str(case)],check=True)
 a=json.loads((case/'analysis.json').read_text());a['extra']=json.loads((case/'extra-analysis.json').read_text());p=json.loads((case/'policy-analysis.json').read_text())[0]
 if contact:
  r=case/'instance-1';src=r/'contacts.csv';e=a['extra'][0]
  for name,win in [('contacts-all-summary.json',[]),('contacts-task-summary.json',[e['batch_first_picking_sim'],e['batch_last_terminal_sim']])]:
   if any(x is None for x in win):continue
   subprocess.run(['python3',str(R/'tools/contact_summary.py'),str(src),str(r/name),*map(str,win)],check=True)
  with src.open('rb') as f,gzip.open(str(src)+'.gz','wb',compresslevel=4) as z:shutil.copyfileobj(f,z)
 a['policy']=p;return a

def good(a,policy):
 return a['reason']=='completed' and a['successful_batches']==1 and not a['oom_delta'] and not a['swapout_delta'] and not any(i['interlock_violations'] or i['vision_mismatches'] or i['isolation']['ros_foreign'] or i['isolation']['gz_foreign'] for i in a['instances']) and (a['policy']['rule_confirmed'] if policy=='rule' else a['policy']['q_confirmed'] and a['policy']['known_state_decisions']>0 and not a['policy']['illegal_decisions'])
def run(label,policy,contact=False):
 env=os.environ.copy()
 for k in ('PERF_CONTACT','PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','PERF_POLICY_PARAMS'):env.pop(k,None)
 env['PERF_POLICY']=policy;env['PERF_LP_THREADS']='2'
 if policy=='q':env['PERF_POLICY_PARAMS']=str(R/'training/q-params.yaml')
 if contact:env['PERF_CONTACT']='1'
 print('BEGIN',label,flush=True)
 subprocess.run(['bash','-c',f'source {R}/activate.sh; exec python {R}/tools/run_case.py {label} 1 true 1 650'],env=env,check=True)
 case=Path((R/'latest-case.txt').read_text());a=analyze(case,contact);print('END',label,flush=True);return a
while not (initial/'summary.json').exists():time.sleep(2)
base=analyze(initial,True);assert good(base,'rule'),'New version rule acceptance failed; stop Q promotion'
co=json.loads((initial/'instance-1/contacts-task-summary.json').read_text());assert co['bad_rows']==0 and not co['categories'].get('interarm'),'Rule contact gate failed'
q=run('q-acceptance-contact','q',True)
assert good(q,'q'),'Q functional acceptance failed; preserve evidence, do not start paired comparison'
co=json.loads((Path(q['case'])/'instance-1/contacts-task-summary.json').read_text());assert co['bad_rows']==0 and not co['categories'].get('interarm'),'Q interarm/recorder gate failed'
# Three pairs, counterbalanced ordering. Each case fresh process/world/private environment.
for pair,order in enumerate([('rule','q'),('q','rule'),('rule','q')],1):
 for policy in order:
  a=run(f'pair{pair}-{policy}',policy)
  # Failed outcomes remain in the comparison, never rerun until a good answer.
  if any(i['interlock_violations'] for i in a['instances']):raise RuntimeError('Unsafe interlock evidence; stop further pairs')
(R/'POLICY_EXPERIMENTS_DONE').touch();print('POLICY EXPERIMENTS FINISHED',flush=True)
