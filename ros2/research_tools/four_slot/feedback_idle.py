"""One isolated idle Gazebo launch; no gripper/arm/task/fixture commands."""
import os,time,json,sys,subprocess,signal,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1];ROOT=WS.parents[1]
sys.path.insert(0,str(HERE.parent/'voice_four'))
from run_four import stop_group
from shutdown import stop_requests
from task_io import atomic_json

def main():
 run=ROOT/'feedback_fix'/('idle-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]);run.mkdir();(run/'home').mkdir();print(run,flush=True)
 env=os.environ.copy()
 for k in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY'):env.pop(k,None)
 env.update(HOME=str(run/'home'),ROS_DOMAIN_ID='95',GZ_PARTITION=run.name,ROS_LOG_DIR=str(run/'ros'),LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
 cmd=['ros2','launch','a1_cell_sim','cell_bringup.launch.py','dual:=true','spawn_mode:=service','gazebo_gui:=false','gpu_offload:=false','world_file:=four_slot_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml','fast_sim_waits:=true']
 atomic_json(run/'metadata.json',dict(command=cmd,environment={k:env[k] for k in ['ROS_DOMAIN_ID','GZ_PARTITION','LP_NUM_THREADS']},commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip(),source_diff=subprocess.check_output(['git','diff'],cwd=WS,text=True),scope='user approved one idle feedback comparison; no commands'))
 children=[];handles=[];interrupted=[]
 for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *x:interrupted.append(True))
 def start(name,cmd):
  f=(run/(name+'.log')).open('w');handles.append(f);p=subprocess.Popen(cmd,env=env,cwd=WS,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
 obs=start('observer',[sys.executable,str(HERE.parent/'voice_four/observe_four.py'),str(run),'--no-submit','--calibration']);scene=start('scene',cmd);probe=None;begin=time.monotonic();outcome='wall_bound'
 try:
  while time.monotonic()-begin<65:
   if interrupted:raise RuntimeError('operator_stop')
   if (run/'guard-fault.json').exists():raise RuntimeError('guard_fault')
   if obs.poll() is not None or scene.poll() is not None:raise RuntimeError('child_exit')
   if time.monotonic()-begin>22 and probe is None:probe=start('probe',[sys.executable,str(HERE/'feedback_probe.py'),str(run),'--seconds','25'])
   if probe is not None and probe.poll() is not None:
    if probe.returncode:raise RuntimeError('probe_exit')
    outcome='idle_comparison_complete';break
   time.sleep(.02)
 except Exception as e:outcome=str(e)
 finally:
  (run/'SHUTDOWN_INTENT').touch();steps=stop_requests(run,env)
  if probe is not None:
   try:probe.wait(timeout=5)
   except subprocess.TimeoutExpired:stop_group(probe)
  stop_group(scene);(run/'SUPERVISOR_STOP').touch()
  try:obs.wait(timeout=8)
  except subprocess.TimeoutExpired:stop_group(obs)
  for f in handles:f.close()
  atomic_json(run/'summary.json',dict(outcome=outcome,observer_exit=obs.poll(),scene_exit=scene.poll(),probe_exit=probe.poll() if probe else None,stop=steps,task_sent=False,arm_goals_enabled=False))
 print(outcome,flush=True)
if __name__=='__main__':main()
