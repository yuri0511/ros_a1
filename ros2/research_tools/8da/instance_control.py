"""Use the exact saved domain/partition. World reset does not reset ROS task state."""
import argparse,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('case',type=Path);p.add_argument('instance',type=int);p.add_argument('action',choices=('pause','unpause','reset','stop'));a=p.parse_args()
run=a.case/f'instance-{a.instance}';saved=json.loads((run/'launch-environment.json').read_text())
if a.action=='stop':
 (run/'STOP_INSTANCE').touch();print('Requested isolated instance stop:',run);raise SystemExit()
req={'pause':'pause: true','unpause':'pause: false','reset':'pause: true reset: {all: true}'}[a.action]
env=os.environ.copy();env.update(saved['env'])
cmd=['bash','-c','source /home/yu/yuri_gazebo_performance/fast_sim_8da998b/activate.sh; exec gz service -s /world/sorting_cell/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 4000 --req "$1"','bash',req]
r=subprocess.run(cmd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=8)
with (a.case/'manual-control.jsonl').open('a') as f:f.write(json.dumps({'wall':time.time(),'instance':a.instance,'action':a.action,'command':cmd,'output':r.stdout})+'\n')
print(r.stdout);raise SystemExit(0 if 'data: true' in r.stdout else 1)
