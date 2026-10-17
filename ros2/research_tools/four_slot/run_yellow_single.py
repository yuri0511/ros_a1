"""Real feeder/RGB/executor/recorder wiring; refuses absent calibration evidence.

No synthetic part IDs or completion results. Existing supervisor and raw sensor,
contact, controller, physical READY/slot/CSV audit are reused unchanged.
"""
import argparse,sys,json,hashlib,uuid,time,os
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1];ROOT=WS.parents[1]
sys.path.insert(0,str(HERE.parent/'voice_four'))
import run_four
from yellow_task import load_task
from a1_sorting_core.voice_batch_plan import validate_candidate
from a1_sorting_core.four_slot_trial import load_trial

def task_record():
    p=validate_candidate(json.dumps({'schema_version':1,'action':'sort_batch','counts':dict(blue=0,yellow=1,red=0,green=0),'priority_first':'yellow','proposed_order':['yellow']}))
    return dict(request_id=uuid.uuid4().hex,source='local single-yellow acceptance text; no ASR/API',transcript_unmodified='开始分拣，一个黄色，先完成黄色。',plan=p.as_dict(),plan_sha256=p.sha256)

def commands(run,permit):
    import yaml
    d=yaml.safe_load((WS/'src/a1_sorting_core/config/four_slot_overlay.yaml').read_text())
    candidate=WS/'src/a1_sorting_core/config/four_slot_candidate.json'
    for name in ['/task_dispatcher','/pick_place_executor','/scene_publisher']:
        v=d.setdefault(name,{}).setdefault('ros__parameters',{});v.update(four_slot_trial_permit=str(permit),four_slot_config=str(candidate))
    d['/pick_place_executor']['ros__parameters'].update(four_slot_inventory=str(run/'arm1_inventory.json'),split_plan_execute=True)
    d['/scene_publisher']['ros__parameters']['four_slot_world']=str(WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
    d['/pick_place_relay']['ros__parameters']['four_slot_mode']=False
    overlay=run/'single_overlay.yaml'
    if run.is_dir():overlay.write_text(yaml.safe_dump(d,allow_unicode=True))
    return ['ros2','launch','a1_sorting_core','sorting_demo.launch.py','gpu_offload:=false','gazebo_gui:=true','launch_rviz:=false','world_file:=four_slot_1ms.sdf','controllers_file:=ur5e_cell_controllers_stability_gain1.yaml','pipeline:=true','dual_arm:=true','fast_sim_waits:=true','feed_policy:=rule','voice_batch_mode:=true','auto_start:=false','demo_parts:=1','experiment_params:='+str(overlay),'result_csv:='+str(run/'results.csv')]

def main():
    p=argparse.ArgumentParser();p.add_argument('--parent-approval',type=Path,required=True);p.add_argument('--calibration-evidence',type=Path,required=True);p.add_argument('--permit',type=Path,required=True);a=p.parse_args()
    from concentrated_acceptance import calibration_pass
    sha=run_four.subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip();approval=json.loads(a.parent_approval.read_text())
    if not (approval.get('approved') is True and approval.get('scope')=='red4-then-yellow-single-once' and approval.get('commit')==sha):raise SystemExit('REFUSED: scope/commit approval mismatch')
    if not calibration_pass(a.calibration_evidence):raise SystemExit('REFUSED: calibration did not pass')
    load_trial(a.permit,WS/'src/a1_sorting_core/config/four_slot_candidate.json')
    prep=ROOT/'acceptance_preparation'/('single-'+uuid.uuid4().hex);prep.mkdir(parents=True)
    record=task_record();candidate=prep/'task.json';candidate.write_text(json.dumps(record,ensure_ascii=False,indent=2))
    child=dict(approved=True,scope='one-yellow-single-gazebo-diagnostic',request_id=record['request_id'],plan_sha256=record['plan_sha256'],commit=sha,expires_wall=time.time()+600);ap=prep/'approval.json';ap.write_text(json.dumps(child))
    original=run_four.verify_profile
    def profile():
        profiles=original();import xml.etree.ElementTree as E
        f=WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf'
        if float(E.parse(f).find('.//physics/max_step_size').text)!=.001:raise RuntimeError('physics_step_changed')
        return {**profiles,str(f):hashlib.sha256(f.read_bytes()).hexdigest()}
    from gui_evidence import confirm
    checked=[0.]
    def live(run,env):
        if time.monotonic()-checked[0]>.5:
            confirm(run,env['GZ_PARTITION']);checked[0]=time.monotonic()
    run_four.LIVE_PREREQUISITE=live
    run_four.load_task=load_task;run_four.verify_profile=profile;run_four.ENTRY_SCOPE='one-yellow-single-gazebo-diagnostic';run_four.OBSERVER_SCRIPT=HERE/'observe_yellow.py';run_four.commands=lambda run:commands(run,a.permit)
    sys.argv=[__file__,'--candidate',str(candidate),'--approval',str(ap),'--output-root',str(ROOT/'single_runs'),'--timeout','300']
    return run_four.main()
if __name__=='__main__':raise SystemExit(main())
