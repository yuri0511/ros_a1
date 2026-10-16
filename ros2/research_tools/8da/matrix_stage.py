import os,json,subprocess
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b')
for step in (1,2):
 case=next((root/'runs').glob(f'*positive-contact-{step}ms'))
 assert json.loads((case/'instance-1/positive-control.json').read_text())['verified']
for label,step,fast in [('A-1ms-original',1,'false'),('B-1ms-event',1,'true'),('C-2ms-original',2,'false'),('D-2ms-event',2,'true')]:
 print('BEGIN',label,flush=True)
 env=os.environ.copy()
 for k in ('PERF_CONTACT','PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','FASTDDS_BUILTIN_TRANSPORTS','LP_NUM_THREADS'):env.pop(k,None)
 env['PERF_LP_THREADS']='2'
 subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py {label} {step} {fast} 1 650'],env=env,check=True)
 case=Path((root/'latest-case.txt').read_text());subprocess.run(['python3',str(root/'tools/analyze.py'),str(case)],check=True)
 a=json.loads((case/'analysis.json').read_text())
 if a['reason'] in ('node_exception','readiness_timeout','extraneous_gazebo_guard'):raise RuntimeError(a['reason'])
 print('END',label,flush=True)
(root/'MATRIX_LOW_DONE').touch()
print('LOW INTERFERENCE MATRIX FINISHED',flush=True)
