import os,sys,time,json,csv,signal,subprocess,datetime,statistics
from pathlib import Path
ROOT=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2'); WS=ROOT/'worktree/ros2'
label,step,fast,num,limit=sys.argv[1:6];rate='4';num=int(num);limit=float(limit)
fixture=os.environ.get('PERF_FIXTURE','0')
if step != '1' or fast not in ('true','false'):raise SystemExit('Only audited 1/2 ms and explicit wait policy accepted')
contact_mode=os.environ.get('PERF_CONTACT','0')
stop_on_failure=(contact_mode=='1' and step=='2' and fixture!='1')
for proc in Path('/proc').iterdir():
 if not proc.name.isdigit():continue
 try:c=(proc/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')
 except OSError:continue
 if c.startswith('gz sim '):raise SystemExit('Existing Gazebo server/GUI: '+str(proc)+' '+c)
if not 1<=num<=3:raise SystemExit('Only 1..3 complete instances are permitted')
if rate not in ('1','2','4','0'):raise SystemExit('Unsupported audited world variant')
threads=os.environ.get('PERF_LP_THREADS','2')
case=ROOT/'runs'/(datetime.datetime.now().strftime('%Y%m%dT%H%M%S')+'-'+label)
case.mkdir();(ROOT/'latest-case.txt').write_text(str(case))
start=time.time(); children=[];files=[];meta={'label':label,'rate':rate,'step_ms':step,'fast_waits':fast,'contact_mode':contact_mode,'stop_on_failure':stop_on_failure,'commit':'72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd','instances':num,'limit_s':limit,'start':start,'threads':threads,'rmw':os.environ.get('PERF_RMW','rmw_fastrtps_cpp'),'egl':os.environ.get('PERF_EGL','0'),'transport':os.environ.get('FASTDDS_BUILTIN_TRANSPORTS','default'),'isolation_only':os.environ.get('PERF_ISOLATION_ONLY','0'),'probe_mode':'light-with-10s-latency-window','runs':[]}
baseenv=os.environ.copy()
for key in ('LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','MESA_LOADER_DRIVER_OVERRIDE','QT_OPENGL','QT_QUICK_BACKEND','__GLX_VENDOR_LIBRARY_NAME','__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','DRI_PRIME','WAYLAND_DISPLAY','LIBGL_DEBUG','LIBGL_DRI3_DISABLE','MESA_GL_VERSION_OVERRIDE','MESA_EXTENSION_OVERRIDE','ROS_DOMAIN_ID','GZ_PARTITION'):
 baseenv.pop(key,None)
for i in range(num):
 run=case/f'instance-{i+1}';run.mkdir();(run/'home').mkdir()
 env=baseenv.copy();env.update(HOME=str(run/'home'),ROS_DOMAIN_ID=str(81+i),GZ_PARTITION=case.name+f'-{i+1}',ROS_LOG_DIR=str(run/'ros'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1')
 if threads!='default':env['LP_NUM_THREADS']=threads
 if os.environ.get('PERF_RMW'):env['RMW_IMPLEMENTATION']=os.environ['PERF_RMW']
 feed_policy=os.environ.get('PERF_POLICY','rule');policy_params=os.environ.get('PERF_POLICY_PARAMS','')
 if feed_policy=='rule' and policy_params:raise ValueError('Rule runs must not load a Q overlay')
 if feed_policy not in ('rule','q'):raise ValueError('Explicit rule/q required')
 inner=f'source {ROOT}/activate.sh; source {WS}/install/setup.bash; export PATH={ROOT}/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=fast_sim_{step}ms.sdf fast_sim_waits:={fast} result_csv:={run}/results.csv feed_policy:={feed_policy}'
 if policy_params:inner+=' policy_params:='+policy_params
 if os.environ.get('PERF_ISOLATION_ONLY')=='1':inner+=' auto_start:=false'
 if fixture=='1':
  inner=f'source {ROOT}/activate.sh; source {WS}/install/setup.bash; export PATH={ROOT}/tools/bin:$PATH; exec ros2 launch a1_cell_sim cell_bringup.launch.py dual:=true gpu_offload:=false gazebo_gui:=false spawn_mode:=service world_file:=fast_sim_{step}ms.sdf' 
 cmd=['bash','-c',inner];f=(run/'launch.log').open('w');files.append(f)
 contact=None
 if contact_mode=='1':
  cf=(run/'contact-recorder.log').open('w')
  contact=subprocess.Popen(['bash','-c',f'source {ROOT}/activate.sh; exec python {ROOT}/tools/record_contacts.py {run}/contacts.csv'],cwd=WS,env=env,stdout=cf,stderr=subprocess.STDOUT,start_new_session=True)
  deadline=time.monotonic()+10
  while not (run/'CONTACT_READY').exists() and time.monotonic()<deadline and contact.poll() is None:time.sleep(.1)
  if not (run/'CONTACT_READY').exists():raise RuntimeError('Contact recorder not ready before Gazebo')
 p=subprocess.Popen(cmd,cwd=WS,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
 probe=subprocess.Popen(['bash','-c',f'source {ROOT}/activate.sh; source {WS}/install/setup.bash; exec python {ROOT}/tools/probe.py {run} {i+1}'],cwd=WS,env=env,stdout=(run/'probe.log').open('w'),stderr=subprocess.STDOUT,start_new_session=True)
 info={'run':str(run),'pid':p.pid,'probe_pid':probe.pid,'contact_pid':contact.pid if contact else None,'env':{k:v for k,v in env.items() if k in ('HOME','ROS_DOMAIN_ID','GZ_PARTITION','ROS_LOG_DIR','ROS_AUTOMATIC_DISCOVERY_RANGE','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER','QT_QPA_PLATFORM','LP_NUM_THREADS','RMW_IMPLEMENTATION','FASTDDS_BUILTIN_TRANSPORTS','PERF_EGL','DISPLAY','XAUTHORITY')},'command':cmd,'start':time.time()};meta['runs'].append(info);children.append((p,probe,run,env));(run/'launch-environment.json').write_text(json.dumps(info,indent=2))
 if i<num-1:time.sleep(3)
meta['fixture']=fixture
meta['feed_policy']=os.environ.get('PERF_POLICY','rule')
meta['policy_params']=os.environ.get('PERF_POLICY_PARAMS','')
meta['source_hashes']=__import__('hashlib').sha256(b''.join(p.read_bytes() for p in sorted((WS/'src/a1_sorting_core/a1_sorting_core').glob('*.py')))).hexdigest()
(case/'metadata.json').write_text(json.dumps(meta,indent=2));print(case,flush=True)
def snapshot_cpu():
 out={}
 for line in Path('/proc/stat').read_text().splitlines():
  f=line.split()
  if f and f[0].startswith('cpu'):
   v=list(map(int,f[1:]));out[f[0]]=(sum(v[:8]),v[3]+v[4])
 return out
prev=snapshot_cpu();prevproc={};prevtime=time.monotonic();hz=os.sysconf('SC_CLK_TCK');page=os.sysconf('SC_PAGE_SIZE');done={};failed={};capacity=[];seen=set();latency_jobs=[];latency_started=False;saturated_consecutive=0
syslog=(case/'system.jsonl').open('w',buffering=1);procfile=(case/'processes.csv').open('w',buffering=1);writer=csv.writer(procfile);writer.writerow(['wall','instance','pid','cpu_one_core_pct','rss_mib','threads','command'])
def read_status(run):
 for attempt in range(10):
  try:return json.loads((run/'probe-status.json').read_text())
  except (OSError,ValueError):time.sleep(.1)
 raise RuntimeError('probe status unavailable')
def control(i,request):
 p,probe,run,env=children[i]
 command=['bash','-c',f'source {ROOT}/activate.sh; exec gz service -s /world/sorting_cell/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 4000 --req "$1"','bash',request]
 result=subprocess.run(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=8)
 with (case/'isolation-control.jsonl').open('a') as f:f.write(json.dumps({'wall':time.time(),'instance':i+1,'request':request,'command':command,'output':result.stdout})+'\n')
 if 'data: true' not in result.stdout:raise RuntimeError(result.stdout)
def isolation_test():
 first=[read_status(x[2]) for x in children]
 (case/'isolation-proof.json').write_text(json.dumps({'verified':False,'initial':first},indent=2))
 assert all(x['ros_markers']==[str(i+1)] and x['gz_markers']==[str(i+1)] and len(x['clock_publishers'])==1 and x['controllers']==1 for i,x in enumerate(first)),first
 assert first[0]['clock_publishers'][0]['gid']!=first[1]['clock_publishers'][0]['gid']
 control(0,'pause: true');time.sleep(2);paused=[read_status(x[2]) for x in children];time.sleep(2);later=[read_status(x[2]) for x in children]
 assert abs(later[0]['clock']-paused[0]['clock'])<.01,(paused,later)
 assert later[1]['clock']-paused[1]['clock']>.2,(paused,later)
 assert later[0]['counts'].get('gz_rgb',0)==paused[0]['counts'].get('gz_rgb',0),(paused,later)
 assert later[1]['counts'].get('gz_rgb',0)>paused[1]['counts'].get('gz_rgb',0) and later[1]['counts'].get('rgb',0)>paused[1]['counts'].get('rgb',0),(paused,later)
 (case/'isolation-proof.json').write_text(json.dumps({'verified':False,'initial':first,'paused':paused,'pause_check':later},indent=2))
 control(0,'pause: true reset: {all: true}');time.sleep(3);reset=[read_status(x[2]) for x in children]
 assert reset[0]['clock']<1 and reset[1]['clock']>later[1]['clock'],reset
 children[0][2].joinpath('PROBE_STOP').touch();os.killpg(children[0][0].pid,signal.SIGINT);time.sleep(4);after=read_status(children[1][2])
 assert after['clock']>reset[1]['clock'] and after['ros_markers']==['2'] and after['gz_markers']==['2'] and len(after['clock_publishers'])==1,after
 (case/'isolation-proof.json').write_text(json.dumps({'verified':True,'initial':first,'paused':paused,'pause_check':later,'reset_check':reset,'second_after_first_stopped':after},indent=2))
reason='timeout'
initial_vm={x.split()[0]:int(x.split()[1]) for x in Path('/proc/vmstat').read_text().splitlines() if x.startswith(('pswpin ','pswpout ','oom_kill '))}
swap_events=[]
try:
 while time.time()-start<limit:
  now=time.monotonic();wall=time.time();cpu=snapshot_cpu();usage={k:100*(1-(cpu[k][1]-prev[k][1])/(cpu[k][0]-prev[k][0])) for k in cpu if k in prev and cpu[k][0]>prev[k][0]};prev=cpu
  mem={x.split(':')[0]:int(x.split()[1]) for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith(('MemAvailable:','MemTotal:','SwapTotal:','SwapFree:'))};vm={x.split()[0]:int(x.split()[1]) for x in Path('/proc/vmstat').read_text().splitlines() if x.startswith(('pswpin ','pswpout ','oom_kill '))};syslog.write(json.dumps({'wall':wall,'cpu':usage,'mem_kib':mem,'vm':vm})+'\n');capacity.append(usage.get('cpu',0))
  for idx,(p,probe,run,env) in enumerate(children):
   text=(run/'launch.log').read_text(errors='replace')
   if 'STARTUP_READINESS_TIMEOUT' in text:reason='readiness_timeout';raise RuntimeError('readiness gate timed out without feeding')
   if 'Traceback (most recent call last)' in text:reason='node_exception';raise RuntimeError('runtime Python exception; inspect launch.log')
   try:
    rows=list(csv.DictReader((run/'results.csv').open()))
    if stop_on_failure and any(x.get('result')=='FAILED' for x in rows):reason='irrecoverable_task_failure';raise RuntimeError('2-ms contact diagnostic stops at first permanent failure; configuration already failed low-interference full batch')
    if len(rows)>=8 and idx not in done and idx not in failed:
     log=(run/'launch.log').read_text(errors='replace');import re
     stamps=re.findall(r'\[(\d{10}\.\d+)\] \[result_recorder\]: part_\d+\[.*? -> ',log)
     done[idx]={'completion':float(stamps[-1]) if stamps else wall,'rows':rows};print('finished',idx+1,[x['result'] for x in rows],flush=True)
   except (OSError,ValueError):pass
   stop_requested=(run/'STOP_INSTANCE').exists()
   if stop_requested and not (run/'STOP_ACK').exists():
    (run/'STOP_ACK').touch();(run/'PROBE_STOP').touch()
    try:os.killpg(p.pid,signal.SIGINT)
    except ProcessLookupError:pass
   if idx not in done and idx not in failed and (p.poll() is not None or stop_requested):
    failed[idx]={'wall':wall,'reason':'requested_instance_stop' if stop_requested else 'early_exit','exit_code':p.poll()}
    (run/'PROBE_STOP').touch()
    try:os.killpg(p.pid,signal.SIGINT)
    except ProcessLookupError:pass
    print('isolated failure/stop',idx+1,failed[idx],flush=True)
  for path in Path('/proc').iterdir():
   if not path.name.isdigit():continue
   try:
    stat=(path/'stat').read_text().rsplit(')',1)[1].split();session=int(stat[3]);idx=next((i for i,(p,_,_,_) in enumerate(children) if p.pid==session),None)
    if idx is None:
     foreign=(path/'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')
     if foreign.startswith('gz sim '):reason='extraneous_gazebo_guard';raise RuntimeError('Unowned Gazebo PID '+path.name)
     continue
    cmd=(path/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip();ticks=int(stat[11])+int(stat[12]);pid=int(path.name);pcpu=(ticks-prevproc[pid])/hz/(now-prevtime)*100 if pid in prevproc else '';prevproc[pid]=ticks
    writer.writerow([wall,idx+1,pid,pcpu,int(stat[21])*page/1048576,stat[17],cmd])
    if wall-start>30 and pid not in seen:
     seen.add(pid);dest=case/f'instance-{idx+1}'/'process-env';dest.mkdir(exist_ok=True)
     vals=(path/'environ').read_bytes().decode(errors='replace').split('\0');keys=('HOME=','ROS_DOMAIN_ID=','GZ_PARTITION=','ROS_LOG_DIR=','LIBGL_ALWAYS_SOFTWARE=','GALLIUM_DRIVER=','LP_NUM_THREADS=','ROS_AUTOMATIC_DISCOVERY_RANGE=','RMW_IMPLEMENTATION=','FASTDDS_BUILTIN_TRANSPORTS=')
     (dest/f'{pid}.json').write_text(json.dumps({'command':cmd,'environment':[v for v in vals if v.startswith(keys)]},indent=2))
   except (OSError,ValueError,IndexError):pass
  prevtime=now
  status={'wall':wall,'elapsed':wall-start,'completed':len(done),'cpu_total_percent':usage.get('cpu'),'mem_available_gib':mem['MemAvailable']/1048576,'reason':'running'};(case/'status.json').write_text(json.dumps(status))
  saturated_consecutive=saturated_consecutive+1 if wall-start>30 and usage.get('cpu',0)>90 else 0
  if num>=3 and saturated_consecutive>=10:reason='cpu_expansion_guard';break
  if mem['MemAvailable']<1500000 or vm['oom_kill']>initial_vm['oom_kill']:reason='memory_or_oom_guard';break
  swap_events.append(vm['pswpout'])
  if len(swap_events)>10 and swap_events[-1]>swap_events[-10]+1024:reason='sustained_swap_guard';break
  if wall-start>30 and not (case/'network-sockets.txt').exists():
   try:
    net=subprocess.run(['ss','-utapn'],capture_output=True,text=True,timeout=2)
    (case/'network-sockets.txt').write_text(net.stdout+net.stderr)
   except (OSError,subprocess.TimeoutExpired) as ex:(case/'network-sockets.txt').write_text(repr(ex))
  if not latency_started and wall-start>40 and os.environ.get('PERF_ISOLATION_ONLY')!='1':
   latency_started=True
   for lp,lprobe,lrun,lenv in children:
    lf=(lrun/'latency-probe.log').open('w')
    latency_jobs.append(subprocess.Popen(['bash','-c',f'source {ROOT}/activate.sh; source {WS}/install/setup.bash; exec python {ROOT}/tools/latency_probe.py {lrun}'],env=lenv,stdout=lf,stderr=subprocess.STDOUT,start_new_session=True))
  if (case/'STOP').exists():reason='requested_stop';break
  if os.environ.get('PERF_ISOLATION_ONLY')=='1' and wall-start>45:
   isolation_test();reason='isolation_verified';break
  if len(done)+len(failed)==num:reason='completed_with_failures' if failed else 'completed';break
  time.sleep(2)
except Exception as e:
 meta['error']=repr(e)
finally:
 # Collect settled final positions after all measurement timestamps, then stop.
 for p,probe,run,env in children:
  if (run/'results.csv').exists():
   try:subprocess.run(['bash','-c',f'source {ROOT}/activate.sh; exec python {ROOT}/tools/positions.py {run}'],env=env,stdout=subprocess.DEVNULL,stderr=(run/'positions-errors.log').open('w'),timeout=5)
   except subprocess.TimeoutExpired:pass
 # Stop only these sessions. Preserve application-level result files and all old experiments.
 meta['done']=done;meta['failed']=failed;meta['reason']=reason;meta['measure_end']=max((x['completion'] for x in done.values()),default=time.time()) if len(done)==num else time.time();meta['capacity_saturation_samples']=sum(x>90 for x in capacity);meta['cpu_mean_percent']=statistics.mean(capacity[2:]) if len(capacity)>2 else 0
 for p,probe,run,env in children:
  (run/'PROBE_STOP').touch()
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 deadline=time.monotonic()+12
 while time.monotonic()<deadline and any(p.poll() is None for p,_,_,_ in children):time.sleep(.2)
 for p,probe,run,env in children:
  if p.poll() is None:
   try:os.killpg(p.pid,signal.SIGTERM)
   except ProcessLookupError:pass
  try:probe.wait(timeout=3)
  except subprocess.TimeoutExpired:os.killpg(probe.pid,signal.SIGTERM)
 time.sleep(1)
 for p,probe,run,env in children:
  try:os.killpg(p.pid,signal.SIGTERM)
  except ProcessLookupError:pass
  try:p.wait(timeout=3)
  except subprocess.TimeoutExpired:
   try:os.killpg(p.pid,signal.SIGKILL)
   except ProcessLookupError:pass
 # A terminated launch parent does not prove its entire session has exited.
 meta['residual_session_cleanup']=[]
 for p,probe,run,env in children:
  live=[]
  for proc in Path('/proc').iterdir():
   if not proc.name.isdigit():continue
   try:
    fields=(proc/'stat').read_text().rsplit(')',1)[1].split()
    if int(fields[3])==p.pid and fields[0]!='Z':live.append(int(proc.name))
   except (OSError,ValueError,IndexError):pass
  if live:
   meta['residual_session_cleanup'].append({'session':p.pid,'pids':live})
   try:os.killpg(p.pid,signal.SIGKILL)
   except ProcessLookupError:pass
 for info in meta['runs']:
  run=Path(info['run']);(run/'CONTACT_STOP').touch()
  if info.get('contact_pid'):
   try:os.killpg(info['contact_pid'],signal.SIGINT)
   except ProcessLookupError:pass
 deadline=time.monotonic()+3
 while time.monotonic()<deadline:
  alive=[]
  for info in meta['runs']:
   if not info.get('contact_pid'):continue
   try:
    result=os.waitpid(info['contact_pid'],os.WNOHANG)
    if result[0]==0:alive.append(info['contact_pid'])
   except ChildProcessError:pass
  if not alive:break
  time.sleep(.1)
 for pid in alive:
  try:os.killpg(pid,signal.SIGKILL)
  except ProcessLookupError:pass
 for f in files:f.close()
 meta['end']=time.time();(case/'summary.json').write_text(json.dumps(meta,indent=2));(case/'status.json').write_text(json.dumps({'reason':reason,'completed':len(done),'wall':time.time()}));print(reason,flush=True)
