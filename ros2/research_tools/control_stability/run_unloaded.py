"""One isolated gain trial. Stops at first fault; never auto-retries a failed trial."""
import os,sys,time,json,subprocess,signal,datetime,hashlib
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/control_stability');W=R/'worktree/ros2';gain=sys.argv[1];tag={'2.0':'2','1.0':'1','0.5':'0p5'}[gain]
C=R/'runs'/(datetime.datetime.now().strftime('%Y%m%dT%H%M%S')+'-empty-gain'+tag);C.mkdir();I=C/'instance-1';I.mkdir();(I/'home').mkdir();(R/'latest-empty-case.txt').write_text(str(C));env=os.environ.copy()
for k in ('WAYLAND_DISPLAY','__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME','ROS_DOMAIN_ID','GZ_PARTITION'):env.pop(k,None)
env.update(HOME=str(I/'home'),ROS_DOMAIN_ID='91',GZ_PARTITION=C.name,ROS_LOG_DIR=str(I/'ros'),LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',QT_QPA_PLATFORM='xcb',LP_NUM_THREADS='2',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',CONTROL_NATIVE_CSV=str(I/'native-joints.csv'))
children=[];handles=[];start=time.time();reason='exception';meta={'gain':float(gain),'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=W,text=True).strip(),'start':start,'env':{k:v for k,v in env.items() if k in ('HOME','ROS_DOMAIN_ID','GZ_PARTITION','ROS_LOG_DIR','CONTROL_NATIVE_CSV','LP_NUM_THREADS','DISPLAY','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER')},'commands':[]}
def launch(command,name):
 h=(I/(name+'.log')).open('w');handles.append(h);p=subprocess.Popen(['bash','-c',f'source {R}/activate.sh; source {W}/install/setup.bash; export PATH={R}/tools/bin:$PATH; '+command],cwd=W,env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append((name,p));meta['commands'].append({'name':name,'command':command,'pid':p.pid});return p
def pause():
 x=subprocess.run(['bash','-c',f'source {R}/activate.sh; exec gz service -s /world/sorting_cell/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 4000 --req "pause: true"'],env=env,capture_output=True,text=True,timeout=7);meta['pause']={'wall':time.time(),'output':x.stdout+x.stderr}
def guard():
 if (I/'guard-fault.json').exists() or (C/'STOP').exists():pause();raise RuntimeError('safety_fault_or_requested_stop')
 for name,p in children:
  if name in ('observe','probe','contact','launch') and p.poll() is not None:pause();raise RuntimeError(name+' exited')
 if time.time()-start>180:pause();raise TimeoutError('trial bound')
def wait(p):
 while p.poll() is None:guard();time.sleep(.05)
 if p.returncode:raise RuntimeError('child failed '+str(p.returncode))
try:
 launch(f'exec python {R}/tools/observe.py {I}','observe');launch(f'exec python {R}/tools/record_contacts.py {I}/contacts.csv','contact');launch(f'exec python {R}/tools/probe.py {I} empty','probe')
 p=launch(f'exec ros2 launch {W}/research_tools/control_stability/unloaded.launch.py controllers_file:=ur5e_cell_controllers_stability_gain{tag}.yaml','launch')
 ready_deadline=time.monotonic()+45
 while True:
  guard();text=(I/'launch.log').read_text()
  if all(x in text for x in ('Configured and activated arm2_joint_trajectory_controller','Configured and activated gripper_controller','Configured and activated joint_state_broadcaster')):break
  if time.monotonic()>ready_deadline:raise TimeoutError('controller startup barrier')
  time.sleep(.1)
 auditor=launch(f'exec python -u {R}/tools/runtime_audit.py {I}/runtime-parameters.json','audit');wait(auditor)
 data=json.loads((I/'runtime-parameters.json').read_text());assert data['parameters']['/gz_ros_control']['position_proportional_gain']==float(gain);assert data['parameters']['/controller_manager']['update_rate']==500
 # Prove exact binaries loaded in the actual server, rather than just installed.
 maps={}
 for proc in Path('/proc').iterdir():
  if not proc.name.isdigit():continue
  try:
   content=(proc/'maps').read_text()
   found=sorted({x.split()[-1] for x in content.splitlines() if 'libgz_ros2_control' in x or 'libjoint_trace.so' in x})
   if found:maps[proc.name]=found
  except (OSError,IndexError):pass
 meta['loaded_libraries']=maps
 for arm,node in [('arm1','pick_place_executor'),('arm2','pick_place_relay')]:
  motion=launch(f'exec python {R}/tools/unloaded_motion.py {I} --ros-args -r __node:={node} --params-file {W}/src/a1_sorting_core/config/sorting_params.yaml --params-file {W}/src/a1_sorting_core/config/sorting_params_pipeline.yaml -p use_sim_time:=true -p fast_sim_waits:=true',arm)
  wait(motion);guard();result=json.loads((I/(arm+'-result.json')).read_text());assert result['passed'],result['fault']
 reason='both_empty_arms_passed'
except Exception as e:
 meta['error']=repr(e);reason='safety_stop_or_failure';print(repr(e),flush=True)
 if 'pause' not in meta:
  try:pause()
  except Exception as stop_error:meta['pause_error']=repr(stop_error)
finally:
 (I/'PROBE_STOP').touch();(I/'CONTACT_STOP').touch()
 for name,p in reversed(children):
  if p.poll() is None:
   try:os.killpg(p.pid,signal.SIGINT)
   except ProcessLookupError:pass
 for name,p in reversed(children):
  try:p.wait(timeout=8 if name=='launch' else 3)
  except subprocess.TimeoutExpired:
   try:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=2)
   except (ProcessLookupError,subprocess.TimeoutExpired):
    try:os.killpg(p.pid,signal.SIGKILL)
    except ProcessLookupError:pass
 meta.update(end=time.time(),reason=reason,exits={name:p.poll() for name,p in children});(C/'summary.json').write_text(json.dumps(meta,indent=2));(C/'metadata.json').write_text(json.dumps(meta,indent=2))
 for h in handles:h.close()
 print(str(C),reason,flush=True)
