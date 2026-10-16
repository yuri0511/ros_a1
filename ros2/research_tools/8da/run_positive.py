from pathlib import Path
import json,time,subprocess,os
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b')
for step in (1,2):
 old=(root/'latest-case.txt').read_text() if (root/'latest-case.txt').exists() else ''
 env=os.environ.copy();env.update(PERF_CONTACT='1',PERF_FIXTURE='1')
 p=subprocess.Popen(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py positive-contact-{step}ms {step} false 1 90'],env=env)
 case=None
 while p.poll() is None:
  latest=(root/'latest-case.txt').read_text() if (root/'latest-case.txt').exists() else old
  if latest!=old and (Path(latest)/'metadata.json').exists():case=Path(latest);break
  time.sleep(.2)
 if case is None:raise RuntimeError('fixture failed')
 m=json.loads((case/'metadata.json').read_text());renv=env.copy();renv.update(m['runs'][0]['env']);run=Path(m['runs'][0]['run'])
 try:
  result=subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/positive_contacts.py {run}'],env=renv,stdout=(run/'positive-control.log').open('w'),stderr=subprocess.STDOUT,timeout=50)
  assert result.returncode==0,(run/'positive-control.log').read_text()
 finally:(case/'STOP').touch();p.wait(timeout=30)
 print('POSITIVE PASS',step,case,flush=True)
