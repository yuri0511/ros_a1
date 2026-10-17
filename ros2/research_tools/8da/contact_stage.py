import os,time,json,subprocess,gzip
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b')
while not (root/'MATRIX_LOW_DONE').exists():time.sleep(5)
for case in (root/'runs').iterdir():
 if (case/'summary.json').exists() and not 'positive' in case.name:
  subprocess.run(['python3',str(root/'tools/extra_analysis.py'),str(case)],check=True)
# Historical evidence is summarized separately, never pooled across machines/commits.
for source in (root/'worktree/ros2/measure').glob('contacts*.csv.gz'):
 subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(source),str(root/'audit'/f'yuri-{source.stem}-summary.json')],check=True)
for label,step,fast in [('A-contact-1ms-original',1,'false'),('B-contact-1ms-event',1,'true'),('C-contact-2ms-original-diagnostic',2,'false'),('D-contact-2ms-event-diagnostic',2,'true')]:
 print('BEGIN',label,flush=True)
 env=os.environ.copy()
 for k in ('PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','FASTDDS_BUILTIN_TRANSPORTS','LP_NUM_THREADS'):env.pop(k,None)
 env.update(PERF_LP_THREADS='2',PERF_CONTACT='1')
 subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py {label} {step} {fast} 1 900'],env=env,check=True)
 case=Path((root/'latest-case.txt').read_text())
 subprocess.run(['python3',str(root/'tools/analyze.py'),str(case)],check=True)
 subprocess.run(['python3',str(root/'tools/extra_analysis.py'),str(case)],check=True)
 run=case/'instance-1';source=run/'contacts.csv'
 subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(source),str(run/'contacts-all-summary.json')],check=True)
 extra=json.loads((case/'extra-analysis.json').read_text())[0]
 if extra['batch_first_picking_sim'] is not None and extra['batch_last_terminal_sim'] is not None:
  subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(source),str(run/'contacts-task-summary.json'),str(extra['batch_first_picking_sim']),str(extra['batch_last_terminal_sim'])],check=True)
 # Keep both the untouched CSV and a compressed copy; no deletion of old data.
 with source.open('rb') as f,gzip.open(str(source)+'.gz','wb',compresslevel=4) as z:
  import shutil;shutil.copyfileobj(f,z)
 print('END',label,flush=True)
(root/'CONTACT_MATRIX_DONE').touch();print('CONTACT MATRIX FINISHED',flush=True)
