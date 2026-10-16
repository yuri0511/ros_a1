import sys,json,time,hashlib
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');sys.path.insert(0,str(R/'worktree/ros2/codex_rl_v2'))
from dual_arm_rl.model import load_config,config_hash,Model,run
from dual_arm_rl.train import train
from dual_arm_rl.qtable import save,load
c=load_config(R/'training/config.v2.json');audit=[]
for name,batch in [('demo',[2,2,2,2]),('general',None)]:
 start=time.monotonic();t=train(c,episodes=50000,batch_size=8,seed=42,batch=batch);f=R/f'training/q-v2-{name}.json';save(f,t,c);load(f,c)
 d={'name':name,'episodes':50000,'seed':42,'states':len(t['q_values']),'wall_s':time.monotonic()-start,'model_only':True,'config_sha256':config_hash(c),'config_file_sha256':hashlib.sha256((R/'training/config.v2.json').read_bytes()).hexdigest(),'table_sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'hyperparameters':t['training']};audit.append(d);print(json.dumps(d),flush=True)
(R/'training/TRAINING_MANIFEST.json').write_text(json.dumps(audit,indent=2))
