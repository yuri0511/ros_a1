"""One human-confirmed run; isolated world, bounded supervision, no cloud calls."""
import argparse,csv,hashlib,json,os,signal,subprocess,sys,time,uuid
from pathlib import Path
from a1_sorting_core.voice_batch_plan import PlanRejected
from a1_sorting_core.voice_batch_results import validate_batch_results
from a1_sorting_core.voice_demo_audit import BatchEventAudit
from task_io import load_task,atomic_json
from shutdown import stop_requests
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1];ROOT=WS.parents[1]
ENTRY_SCOPE='one-four-part-gazebo-diagnostic'
OBSERVER_SCRIPT=HERE/'observe_four.py'
LIVE_PREREQUISITE=lambda run,env: None

def commands(run):
    return ['ros2','launch','a1_sorting_core','sorting_demo.launch.py',
        'gpu_offload:=false','gazebo_gui:=false','launch_rviz:=false',
        'world_file:=fast_sim_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml',
        'pipeline:=true','dual_arm:=true','fast_sim_waits:=true','feed_policy:=rule','voice_batch_mode:=true','auto_start:=false',
        'demo_parts:=4','result_csv:='+str(run/'results.csv')]

def verify_profile():
    import yaml,xml.etree.ElementTree as ET
    cfg=WS/'src/a1_cell_sim/config/ur5e_cell_controllers_stability_gain1.yaml'
    d=yaml.safe_load(cfg.read_text())
    if d['/**']['ros__parameters']['position_proportional_gain']!=1. or d['controller_manager']['ros__parameters']['update_rate']!=500:raise PlanRejected('controller_profile_changed')
    world=WS/'src/a1_cell_sim/worlds/fast_sim_1ms.sdf';s=ET.parse(world).find('.//physics/max_step_size')
    if s is None or float(s.text)!=.001:raise PlanRejected('physics_step_changed')
    return {str(p.relative_to(WS)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (cfg,world)}

def stop_group(child):
    for sig,wait in ((signal.SIGINT,12),(signal.SIGTERM,3),(signal.SIGKILL,1)):
        try:os.killpg(child.pid,sig)
        except ProcessLookupError:break
        try:child.wait(timeout=wait)
        except subprocess.TimeoutExpired:continue
        # Also signal any children left in the original group, never global pkill.
        time.sleep(.2)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,default=ROOT/'tasks/four-part-text.json')
    p.add_argument('--output-root',type=Path,default=ROOT/'runs');p.add_argument('--timeout',type=float,default=420)
    mode=p.add_mutually_exclusive_group();mode.add_argument('--check',action='store_true');mode.add_argument('--interactive',action='store_true');mode.add_argument('--approval',type=Path)
    args=p.parse_args();record,plan=load_task(args.candidate);hashes=verify_profile()
    if not 60<=args.timeout<=600:p.error('timeout must be 60..600 wall seconds')
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip()
    proposed={'task':record,'commit':commit,'profile_sha256':hashes,'command':commands(Path('RUN_DIRECTORY')),
        'contact_policy':'Any inter-arm or arm-body/bin contact stops. Any positive finger/bin contact pauses for review; no numeric penetration allowance.',
        'no_api_calls':True,'check_only':args.check}
    print(json.dumps(proposed,ensure_ascii=False,indent=2),flush=True)
    if args.check:return 0
    if not os.environ.get('DISPLAY'):raise PlanRejected('validated_X11_sensor_path_requires_DISPLAY')
    if args.interactive:
        if not sys.stdin.isatty():raise PlanRejected('human_TTY_required')
        if input('本轮会启动仿真并运动、送料上述4件。确认请输入 CONFIRM：').strip()!='CONFIRM':raise PlanRejected('human_not_confirmed')
        approval=dict(approved=True,scope=ENTRY_SCOPE,request_id=record['request_id'],plan_sha256=plan.sha256,commit=commit,expires_wall=time.time()+600,source='local_human_TTY_CONFIRM')
    elif args.approval:
        approval=json.loads(args.approval.read_text())
    else:raise PlanRejected('human_confirmation_required_before_Gazebo_start')
    if not (approval.get('approved') is True and approval.get('scope')==ENTRY_SCOPE and approval.get('request_id')==record['request_id'] and approval.get('plan_sha256')==plan.sha256 and approval.get('commit')==commit and time.time()<approval.get('expires_wall',0)<=time.time()+86400):raise PlanRejected('approval_missing_stale_or_wrong_task_commit')
    if subprocess.check_output(['git','status','--porcelain','--','ros2/src','ros2/research_tools/voice_four'],cwd=WS,text=True).strip():raise PlanRejected('runtime_sources_dirty_after_tested_commit')
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:c=(proc/'cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError,UnicodeError):continue
        if c.startswith('gz sim '):raise PlanRejected('existing_Gazebo_stop_owner_first')
    claims=ROOT/'operator';claims.mkdir(exist_ok=True)
    with (claims/('consumed-'+record['request_id']+'.json')).open('x') as f:json.dump(approval,f)
    run=args.output_root/('four-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6]);run.mkdir(parents=True);(run/'home').mkdir()
    atomic_json(run/'approved-task.json',{**record,'human_confirmed':True,'approval':approval});atomic_json(ROOT/'latest-run.json',{'run':str(run)})
    env=os.environ.copy()
    for k in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY','MESA_LOADER_DRIVER_OVERRIDE'):env.pop(k,None)
    env.update(HOME=str(run/'home'),ROS_DOMAIN_ID='88',GZ_PARTITION=run.name,ROS_LOG_DIR=str(run/'ros'),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1')
    cmd=commands(run);meta={**proposed,'command':cmd,'run':str(run),'start_wall':time.time(),'environment':{k:env.get(k) for k in ('HOME','ROS_DOMAIN_ID','GZ_PARTITION','ROS_LOG_DIR','DISPLAY','XAUTHORITY','LP_NUM_THREADS','LIBGL_ALWAYS_SOFTWARE','GALLIUM_DRIVER')},'source_status':subprocess.check_output(['git','status','--porcelain'],cwd=WS,text=True)}
    atomic_json(run/'metadata.json',meta);print('RUN_DIRECTORY '+str(run),flush=True)
    interrupted=[]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda s,f:interrupted.append(s))
    children=[];handles=[];reason='timeout';detail='';audit=BatchEventAudit(plan,record['request_id']);actual_releases=set();start=time.monotonic();result=None
    try:
        for name,command in [('observer',[sys.executable,str(OBSERVER_SCRIPT),str(run)]),('launch',cmd)]:
            log=(run/(name+'.log')).open('w');handles.append(log)
            children.append(subprocess.Popen(command,cwd=WS,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            if name=='observer':
                deadline=time.monotonic()+10
                while not (run/'OBSERVER_READY').exists() and time.monotonic()<deadline and children[-1].poll() is None:time.sleep(.05)
                if not (run/'OBSERVER_READY').exists():raise PlanRejected('observer_not_ready')
        with (run/'launch.log').open() as log:
            while time.monotonic()-start<args.timeout:
                LIVE_PREREQUISITE(run,env)
                if interrupted or (run/'STOP').exists():reason='operator_stop';break
                if (run/'guard-fault.json').exists():raise PlanRejected((run/'guard-fault.json').read_text())
                if any(c.poll() is not None for c in children):raise PlanRejected('owned_child_exited')
                health=json.loads((run/'observer-health.json').read_text()) if (run/'observer-health.json').exists() else {}
                if time.monotonic()-start>10 and time.time()-health.get('wall',0)>3:raise PlanRejected('observer_stale')
                while True:
                    pos=log.tell();line=log.readline()
                    if not line:break
                    if not line.endswith('\n'):log.seek(pos);break
                    if 'VOICE_BATCH ' in line:audit.event(json.loads(line.split('VOICE_BATCH ',1)[1]))
                    if 'CONTROLLER_RESULT ' in line and not json.loads(line.split('CONTROLLER_RESULT ',1)[1]).get('success'):raise PlanRejected('controller_result_failure')
                    if 'ARM_SLOT_RELEASE ' in line:
                        e=json.loads(line.split('ARM_SLOT_RELEASE ',1)[1]);part=e.get('part');arm=e.get('arm')
                        if e.get('trigger') not in ('ARM_READY','PART_PLACED') or part in actual_releases or part not in audit.parts or not audit.parts[part]['ready'] or not audit.parts[part]['placed'] or e.get('slot_'+str(arm)) is not None:raise PlanRejected('invalid_slot_release')
                        actual_releases.add(part)
                    if any(x in line for x in ('ARM_FAULT','PART_REJECTED','STARTUP_READINESS_TIMEOUT','Traceback (most recent call last)')):raise PlanRejected('runtime_fault:'+line.strip())
                raw=json.loads((run/'observed-status.json').read_text()) if (run/'observed-status.json').exists() else {}
                audit.ready_claims=set(raw.get('ready_parts',[]))
                if audit.completed:
                    if set(audit.parts)!=audit.ready_claims or set(audit.parts)!=set(raw.get('placed_parts',[])):
                        time.sleep(.05);continue
                    if actual_releases!=set(audit.parts) or raw.get('spawn_requests')!=list(plan.order) or [r['color'] for r in raw.get('arrivals',[])]!=list(plan.order) or {r['part'] for r in raw.get('arrivals',[])}!=set(audit.parts):raise PlanRejected('raw_spawn_vision_slot_evidence_mismatch')
                    if any(raw.get(k,0)<=0 for k in ('rgb_frames','logical_frames','joint_messages','controller_messages','stats_messages','contact_points')):raise PlanRejected('missing_live_sensor_controller_native_stats_or_contact_evidence')
                    result={'csv':validate_batch_results(run/'results.csv',plan),'events':audit.finish(),'observations':raw};reason='completed';break
                if (run/'results.csv').exists():
                    data=(run/'results.csv').read_text()
                    if data and not data.endswith('\n'):continue
                    rows=list(csv.DictReader(data.splitlines()))
                    if len(rows)>plan.total or any(r.get('result')!='PLACED' or r.get('target_bin')!=r.get('final_bin') for r in rows):raise PlanRejected('partial_csv_failure')
                time.sleep(.05)
    except (PlanRejected,ValueError,OSError,KeyError) as exc:
        reason='safety_stop';detail=str(exc)
    finally:
        (run/'SHUTDOWN_INTENT').touch()
        # Keep evidence recording until after world pause and simulator teardown.
        # In 62987d0 the ROS stop RPC ran first and the observer stopped too early.
        stopping=stop_requests(run,env)
        for child in reversed(children[1:]):stop_group(child)
        (run/'SUPERVISOR_STOP').touch()
        if children:
            try:children[0].wait(timeout=3)
            except subprocess.TimeoutExpired:stop_group(children[0])
        for handle in handles:handle.close()
        cleanup_ok=all(s['acknowledged'] for s in stopping) and all(c.poll()==0 for c in children)
        if reason=='completed' and not cleanup_ok:reason='completed_with_cleanup_fault'
        summary={'reason':reason,'detail':detail,'result':result,'end_wall':time.time(),'wall_seconds':time.monotonic()-start,'returncodes':[c.poll() for c in children],'stop_requests':stopping,'cleanup_ok':cleanup_ok,'simulation_test_only':True,'cloud_calls':0}
        atomic_json(run/'summary.json',summary);print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)
    return 0 if reason=='completed' else 1

if __name__=='__main__':
    try:raise SystemExit(main())
    except (PlanRejected,FileExistsError) as e:raise SystemExit('REFUSED: '+str(e)) from None
