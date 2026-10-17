"""Bounded Rule demo supervisor. All safety decisions remain in the READY protocol."""
import argparse,collections,csv,datetime,hashlib,json,os,signal,subprocess,time,uuid
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-root',type=Path,default=Path.home()/'yuri_final_runs')
    p.add_argument('--timeout',type=float,default=420)
    args=p.parse_args()
    if not 30<=args.timeout<=600:p.error('timeout must be 30..600 wall seconds')
    if not os.environ.get('DISPLAY'):p.error('X11 DISPLAY is required for the validated sensor rendering path')
    ws=Path(__file__).resolve().parents[2]
    # Prevent accidental concurrent use of this demonstration entry. Independent lab
    # partitions belong to the separately documented throughput experiment.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:cmd=(proc/'cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError,UnicodeError):continue
        if cmd.startswith('gz sim '):raise SystemExit('Existing Gazebo process; stop its owning launcher first')
    run=args.output_root/(datetime.datetime.now().strftime('%Y%m%dT%H%M%S')+'-gain1-rule-'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True);(run/'home').mkdir()
    env=os.environ.copy()
    for key in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY','MESA_LOADER_DRIVER_OVERRIDE'):env.pop(key,None)
    env.update(HOME=str(run/'home'),ROS_LOG_DIR=str(run/'ros'),ROS_DOMAIN_ID='82',GZ_PARTITION=run.name,ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',QT_QPA_PLATFORM='xcb')
    config=ws/'src/a1_cell_sim/config/ur5e_cell_controllers_stability_gain1.yaml'
    assert hashlib.sha256(config.read_bytes()).hexdigest()=='c96a32325ac2d66b958cb4c75ac82b794e1fe64802e25394c717436121b9a5b9'
    cmd=['ros2','launch','a1_sorting_core','sorting_demo.launch.py','gpu_offload:=false','gazebo_gui:=false','launch_rviz:=false','world_file:=fast_sim_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml','fast_sim_waits:=true','feed_policy:=rule','demo_parts:=8','result_csv:='+str(run/'results.csv')]
    record={'command':cmd,'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ws,text=True).strip(),'environment':{k:env[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION','ROS_LOG_DIR','LP_NUM_THREADS','DISPLAY','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER')},'start_wall':time.time()}
    (run/'metadata.json').write_text(json.dumps(record,indent=2));print(run,flush=True)
    interrupted=[]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda s,f:interrupted.append(s))
    reason='timeout';ready=set();begin=time.monotonic()
    with (run/'launch.log').open('w') as log:
        child=subprocess.Popen(cmd,cwd=ws,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while time.monotonic()-begin<args.timeout:
                text=(run/'launch.log').read_text(errors='replace')
                for line in text.splitlines():
                    if 'CONTROLLER_RESULT ' in line and not json.loads(line.split('CONTROLLER_RESULT ',1)[1]).get('success'):reason='controller_failure';break
                    if 'ARM_SLOT_RELEASE ' in line:
                        o=json.loads(line.split('ARM_SLOT_RELEASE ',1)[1])
                        if o.get('trigger')!='ARM_READY':reason='invalid_slot_release';break
                        ready.add(o.get('part_id',o.get('part')))
                if reason not in ('timeout',):break
                if any(x in text for x in ('ARM_FAULT','PART_REJECTED','STARTUP_READINESS_TIMEOUT','Traceback (most recent call last)')):reason='fault';break
                if interrupted or (run/'STOP').exists():reason='requested_stop';break
                if len(ready)>=8:
                    try:rows=list(csv.DictReader((run/'results.csv').open()))
                    except OSError:rows=[]
                    if len(rows)==8:
                        bins=collections.Counter(r['final_bin'] for r in rows)
                        correct=all(r['result']=='PLACED' and r['target_bin']==r['final_bin'] for r in rows) and bins=={b:2 for b in ('bin_blue','bin_yellow','bin_red','bin_green')}
                        reason='completed' if correct else 'result_failure';break
                if child.poll() is not None:reason='early_exit';break
                time.sleep(.2)
        finally:
            if reason!='completed':
                try:subprocess.run(['gz','service','-s','/world/sorting_cell/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],env=env,capture_output=True,timeout=4)
                except (OSError,subprocess.TimeoutExpired):pass
            for sig,delay in ((signal.SIGINT,12),(signal.SIGTERM,3),(signal.SIGKILL,1)):
                try:os.killpg(child.pid,sig)
                except ProcessLookupError:break
                try:child.wait(timeout=delay)
                except subprocess.TimeoutExpired:continue
                # The parent may exit before all processes in its session.
                time.sleep(.5)
            record.update(reason=reason,ready_releases=len(ready),end_wall=time.time(),launch_returncode=child.poll())
            (run/'summary.json').write_text(json.dumps(record,indent=2));print(reason,run,flush=True)
    return 0 if reason=='completed' else 1

if __name__=='__main__':raise SystemExit(main())
