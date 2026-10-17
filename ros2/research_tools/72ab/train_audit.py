import sys,json,time,hashlib,shutil
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');pkg=R/'worktree/ros2/codex_rl';sys.path.insert(0,str(pkg))
from dual_arm_rl.model import load_config,config_hash
from dual_arm_rl.train import train
from dual_arm_rl.qtable import save,load
(R/'training/config').mkdir(parents=True,exist_ok=True);(R/'training/artifacts').mkdir(exist_ok=True);(R/'training/history').mkdir(exist_ok=True)
for src in [pkg/'config/timing.gazebo-v1.json',pkg/'artifacts/gazebo-v1/q_table.json']:shutil.copy2(src,R/'training/history'/src.name)
c=load_config(pkg/'config/timing.gazebo-v1.json');c['allow_prefeed_busy_arm']=False;c['timing_model_version']='gazebo-v1-prefeed-false-unrecalibrated-72ab6f2'
c['measurement_metadata']['limitations']+=['Original duration means retained; NOT recalibrated to local v3 event waits or parallel runs.','Only legal prefeed permission corrected. Pusher return, retries, standby failure and safe zone exit remain incomplete.']
cpath=R/'training/config/timing.prefeed-false.json';cpath.write_text(json.dumps(c,indent=2)+'\n');records=[]
for label,batch in [('general-8',None),('demo-2222',[2,2,2,2])]:
 start=time.monotonic();table=train(c,episodes=50000,batch_size=8,seed=42,batch=batch);path=R/f'training/artifacts/q-{label}.json';save(path,table,c);load(path,c)
 record={'label':label,'episodes':50000,'seed':42,'fixed_batch':batch,'states':len(table['q_values']),'training_wall_s':time.monotonic()-start,'config_sha256':config_hash(c),'config_file_sha256':hashlib.sha256(cpath.read_bytes()).hexdigest(),'table_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'table_path':str(path),'model_only':True};records.append(record);print(json.dumps(record),flush=True)
(R/'training/training-manifest.json').write_text(json.dumps(records,indent=2)+'\n')
