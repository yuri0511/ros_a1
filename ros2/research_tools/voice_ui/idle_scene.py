"""Bounded idle GUI/stop test: simulation layer ONLY, no executor or task sender."""
import json, os, signal, subprocess, sys, time, uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent; WS=HERE.parents[1]; ROOT=WS.parents[1]
sys.path.insert(0,str(HERE.parent/'voice_four'))
from run_four import stop_group,verify_profile
from shutdown import stop_requests
from task_io import atomic_json

def main():
    run=ROOT/'ui_idle_runs'/('idle-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]);run.mkdir(parents=True);(run/'home').mkdir()
    env=os.environ.copy()
    for k in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY'):env.pop(k,None)
    env.update(HOME=str(run/'home'),ROS_DOMAIN_ID='89',GZ_PARTITION=run.name,ROS_LOG_DIR=str(run/'ros'),LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    atomic_json(run/'approved-task.json',json.loads((ROOT/'tasks/four-part-text.json').read_text())) # readonly observer, NOT permission
    cmd=['ros2','launch','a1_cell_sim','cell_bringup.launch.py','dual:=true','spawn_mode:=service','gazebo_gui:=false','gpu_offload:=false','world_file:=fast_sim_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml','fast_sim_waits:=true']
    atomic_json(run/'metadata.json',{'command':cmd,'environment':{k:env[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION','DISPLAY','LP_NUM_THREADS','LIBGL_ALWAYS_SOFTWARE')},'profile':verify_profile(),'task_submission':False,'executor':False,'paid_calls':0})
    (ROOT/'latest-idle-run.txt').write_text(str(run)); print(str(run),flush=True)
    children=[]; handles=[]; gui=None; dispatcher=None; interrupted=[]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *x:interrupted.append(True))
    def start(name,c):
        f=(run/(name+'.log')).open('w');handles.append(f);p=subprocess.Popen(c,env=env,cwd=WS,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    obs=start('observer',[sys.executable,str(HERE.parent/'voice_four/observe_four.py'),str(run),'--no-submit'])
    scene=start('scene',cmd); startwall=time.time(); phases=[]
    try:
        for phase,seconds in [('server_warmup',18),('server_only',12),('visible_gui',25)]:
            if phase=='visible_gui':gui=start('gui',['gz','sim','-g','--gui-config',str(HERE/'idle_gui.config'),'-v','2'])
            begin=time.time(); phases.append({'phase':phase,'start':begin})
            while time.time()-begin<seconds and not interrupted:
                with (run/'resources.txt').open('a') as f:
                    f.write('WALL '+str(time.time())+' PHASE '+phase+'\n');subprocess.run(['ps','-eo','pid,ppid,pcpu,rss,comm'],stdout=f)
                if scene.poll() is not None or obs.poll() is not None:raise RuntimeError('scene/observer exited early')
                time.sleep(1)
            phases[-1]['end']=time.time()
            if phase=='visible_gui':
                subprocess.run([sys.executable,str(HERE/'screenshot.py'),str(run/'desktop.png')],env=env,timeout=10)
        dispatcher=start('dispatcher',['ros2','run','a1_sorting_core','task_dispatcher_node','--ros-args','-p','voice_batch_mode:=true','-p','auto_start:=false','-p','demand_spawn:=true','-p','belt_control:=true','-p','fast_sim_waits:=true','-p','arm2_colors:=blue,green','-p','use_sim_time:=true'])
        time.sleep(2) # only WAITING_FOR_PLAN; no executor or submit client exists
        (run/'SHUTDOWN_INTENT').touch()
        stopping=stop_requests(run,env) # Real dispatcher stop RPC, no task ever submitted
        # Observe real native paused=true and frozen sim_time for 3 wall seconds.
        time.sleep(3)
        atomic_json(run/'pause-observation.json',{'observed_wall_end':time.time(),'steps':stopping})
    finally:
        (run/'SHUTDOWN_INTENT').touch()
        if not (run/'stop-timeline.jsonl').exists():stop_requests(run,env)
        if dispatcher:stop_group(dispatcher)
        if gui:stop_group(gui)
        stop_group(scene)
        (run/'SUPERVISOR_STOP').touch()
        try:obs.wait(timeout=8)
        except subprocess.TimeoutExpired:stop_group(obs)
        for f in handles:f.close()
        atomic_json(run/'summary.json',{'phases':phases,'start':startwall,'end':time.time(),'observer_exit':obs.poll(),'scene_exit':scene.poll(),'gui_exit':gui.poll() if gui else None,'no_task_sent':True,'ROS_stop_service_expected_absent':False,'dispatcher_exit':dispatcher.poll() if dispatcher else None})
if __name__=='__main__':main()
