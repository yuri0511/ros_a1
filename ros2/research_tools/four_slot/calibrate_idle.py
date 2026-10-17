"""Gate-2 evidence only. Simulation layer; no dispatcher, executor or task client."""
import json, os, signal, subprocess, sys, time, uuid, hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parent
WS=HERE.parents[1]
ROOT=WS.parents[1]
sys.path.insert(0,str(HERE.parent/'voice_four'))
from run_four import stop_group
from shutdown import stop_requests
from task_io import atomic_json


def instrument_fixture(sdf):
    """Add a read-only contact sensor; no geometry, mass or pose edits."""
    import xml.etree.ElementTree as ET
    tree=ET.fromstring(sdf);link=tree.find('model/link')
    sensor=ET.SubElement(link,'sensor',name='calibration_support_contact',type='contact')
    ET.SubElement(sensor,'always_on').text='1';ET.SubElement(sensor,'update_rate').text='1000'
    contact=ET.SubElement(sensor,'contact');ET.SubElement(contact,'collision').text='collision';ET.SubElement(contact,'topic').text='/a1_calibration/fixture_contacts'
    return ET.tostring(tree,encoding='unicode')

def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    for flag in ('fingers','arm-probe','empty-slot','release-fixture','lift-fixture'):
        parser.add_argument('--'+flag,action='store_true')
    parser.add_argument('--approval-file',type=Path)
    parser.add_argument('--gui-precheck',action='store_true')
    args=parser.parse_args()
    stage='B' if '--fingers' in sys.argv else 'A'
    arm_probe='--arm-probe' in sys.argv
    empty_slot='--empty-slot' in sys.argv
    release_fixture='--release-fixture' in sys.argv
    lift_fixture='--lift-fixture' in sys.argv
    if arm_probe and stage!='B':raise ValueError('arm-probe requires --fingers')
    if release_fixture and not empty_slot:raise ValueError('release requires --empty-slot')
    if lift_fixture and not release_fixture:raise ValueError('lift requires --release-fixture')
    if empty_slot:
        if not arm_probe:raise ValueError('empty-slot requires planner')
        stage='C'
    approval=None
    if lift_fixture:
        if args.approval_file is None:raise ValueError('a new one-run human approval file is required')
        approval=json.loads(args.approval_file.read_text())
        sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip()
        if approval.get('approved') is not True or approval.get('scope')!='red4-support-cycle-once' or approval.get('commit')!=sha:
            raise ValueError('approval does not match this support-cycle revision')
        if subprocess.check_output(['git','status','--porcelain'],cwd=WS,text=True).strip():raise ValueError('commit code before approving a run')
        uuid.UUID(approval['request_id'])
        token=args.approval_file.with_name('CONSUMED-'+approval['request_id'])
        with token.open('x') as f:json.dump(dict(commit=sha,wall=time.time()),f)
    run=ROOT/'calibration_runs'/(stage+'-'+time.strftime('%Y%m%dT%H%M%S')+'-'+uuid.uuid4().hex[:6])
    run.mkdir(parents=True);(run/'home').mkdir()
    env=os.environ.copy()
    for k in ('__NV_PRIME_RENDER_OFFLOAD','__NV_PRIME_RENDER_OFFLOAD_PROVIDER','__GLX_VENDOR_LIBRARY_NAME','DRI_PRIME','WAYLAND_DISPLAY'):env.pop(k,None)
    env.update(HOME=str(run/'home'),ROS_DOMAIN_ID='95',GZ_PARTITION=run.name,ROS_LOG_DIR=str(run/'ros'),LIBGL_ALWAYS_SOFTWARE='1',GALLIUM_DRIVER='llvmpipe',LP_NUM_THREADS='2',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    cmd=['ros2','launch','a1_cell_sim','cell_bringup.launch.py','dual:=true','spawn_mode:=service',('gazebo_gui:=true' if args.gui_precheck else 'gazebo_gui:=false'),'gpu_offload:=false','world_file:=four_slot_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml','fast_sim_waits:=true']
    files=[HERE/'gui_evidence.py',HERE/'joint_receiver.py',HERE/'lock_window.py',HERE/'support_contact.py',HERE/'live_support_cycle.py',HERE.parent/'voice_four/support_evidence.py',HERE/'arm_probe.py',WS/'src/a1_sorting_core/a1_sorting_core/four_slot_grip_control.py',HERE/'calibrate_fingers.py',HERE/'geometry.py',WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf',WS/'src/a1_cell_sim/config/ur5e_cell_controllers_stability_gain1.yaml',Path(__file__),HERE.parent/'voice_four/observe_four.py']
    atomic_json(run/'metadata.json',dict(stage=stage,scope='isolated_calibration',fingers=(stage!='A'),arm_probe=arm_probe,fixture=empty_slot,release_fixture=release_fixture,lift_fixture=lift_fixture,gate2='BLOCKED',task_authorized=False,reference_task_only=False,command=cmd,environment={k:env[k] for k in ('ROS_DOMAIN_ID','GZ_PARTITION','DISPLAY','LP_NUM_THREADS','LIBGL_ALWAYS_SOFTWARE')},sha256={str(p.relative_to(WS)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip()))
    if approval is not None:atomic_json(run/'human-approval.json',approval)
    (run/'source_snapshot').mkdir()
    for p in files:(run/'source_snapshot'/p.name).write_bytes(p.read_bytes())
    (run/'source.patch').write_text(subprocess.check_output(['git','diff'],cwd=WS,text=True))
    (ROOT/'latest-calibration-run.txt').write_text(str(run));print(run,flush=True)
    handles=[];children=[];interrupted=[]
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *x:interrupted.append(True))
    def start(name,c):
        f=(run/(name+'.log')).open('w');handles.append(f)
        p=subprocess.Popen(c,env=env,cwd=WS,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
    obs=start('observer',[sys.executable,str(HERE.parent/'voice_four/observe_four.py'),str(run),'--no-submit','--calibration'])
    scene=start('scene',cmd)
    planner=start('planner',['ros2','launch',str(HERE/'plan_only.launch.py')]) if arm_probe else None
    outcome='bounded_static_observation';begin=time.monotonic();finger=None;fixture=False
    try:
        while time.monotonic()-begin<(180 if empty_slot else (110 if arm_probe else 50)):
            if interrupted:raise RuntimeError('interrupted')
            if scene.poll() is not None or obs.poll() is not None:raise RuntimeError('scene_or_observer_exited')
            if (run/'guard-fault.json').exists():raise RuntimeError('guard_fault')
            if time.monotonic()-begin>25:
                health=run/'observer-health.json'
                if not health.exists() or time.time()-json.loads(health.read_text())['wall']>2:raise RuntimeError('observer_feedback_loss')
            if empty_slot and time.monotonic()-begin>18 and not fixture:
                import importlib.util
                spec=importlib.util.spec_from_file_location('fixture_source',WS/'src/a1_cell_sim/scripts/spawn_parts.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
                sdf=mod.part_sdf(0,'red').replace('part_0','calibration_red4')
                sdf=instrument_fixture(sdf);(run/'fixture.sdf').write_text(sdf)
                command=['gz','service','-s','/world/sorting_cell/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','2000','--req',f'sdf_filename: "{run}/fixture.sdf" name: "calibration_red4" pose: {{position: {{x: 0.295 y: -0.235 z: 0.747}} orientation: {{w: 1}}}}']
                response=subprocess.run(command,env=env,capture_output=True,text=True,timeout=4)
                atomic_json(run/'fixture-spawn.json',dict(command=command,stdout=response.stdout,stderr=response.stderr,returncode=response.returncode,purpose='50mm 0.2kg calibration fixture; not fed, grasped or sorted'))
                if response.returncode or 'data: true' not in response.stdout:raise RuntimeError('fixture_spawn_failed')
                fixture=True
            if args.gui_precheck and finger is None and time.monotonic()-begin>15:
                from gui_evidence import confirm
                if not confirm(run,env['GZ_PARTITION']):
                    if time.monotonic()-begin>45:raise RuntimeError('GUI_camera_precheck_failed_no_motion')
                    time.sleep(.1);continue
            if stage in ('B','C') and time.monotonic()-begin>22 and finger is None:
                finger=start('fingers',[sys.executable,str(HERE/'calibrate_fingers.py'),str(run)]+(['--arm-probe'] if arm_probe else [])+(['--empty-slot'] if empty_slot else [])+(['--release-fixture'] if '--release-fixture' in sys.argv else [])+(['--lift-fixture'] if '--lift-fixture' in sys.argv else []))
            if finger is not None and finger.poll() is not None:
                if not (run/'FINGER_DONE').exists():raise RuntimeError('finger_test_not_passed')
                outcome=('fixture_cycle_pass' if empty_slot else ('unloaded_arm_probe_pass' if arm_probe else 'unloaded_fingers_pass'));break
            time.sleep(.05)
        if stage!='A' and not (run/'FINGER_DONE').exists():raise RuntimeError('calibration_wall_bound')
    except Exception as e:outcome=str(e)
    finally:
        (run/'SHUTDOWN_INTENT').touch()
        steps=stop_requests(run,env)
        time.sleep(2)
        if finger is not None:stop_group(finger)
        if planner is not None:stop_group(planner)
        stop_group(scene)
        (run/'SUPERVISOR_STOP').touch()
        try:obs.wait(timeout=8)
        except subprocess.TimeoutExpired:stop_group(obs)
        for f in handles:f.close()
        atomic_json(run/'summary.json',dict(outcome=outcome,observer_exit=obs.poll(),scene_exit=scene.poll(),stop=steps,task_sent=False,arm_goals_enabled=arm_probe,gate2='BLOCKED'))
        print(outcome,flush=True)
if __name__=='__main__':main()
