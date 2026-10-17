"""One future approval, no retries: red4 cycle -> yellow preflight -> single sort.

Default --check does not start ROS. --approval-file requires a NEW human approval
bound to this commit; the idle-feedback authorization cannot authorize this tool.
Existing yellow runtime inhibits remain authoritative, never edited by this tool.
"""
import argparse,json,subprocess,sys,time,uuid
from pathlib import Path
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1];ROOT=WS.parents[1]

def yellow_preflight():
    cfg=json.loads((WS/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
    source=(WS/'src/a1_sorting_core/a1_sorting_core/pick_place_executor_node.py').read_text()
    blockers=[]
    if cfg.get('motion_validated') is not True:blockers.append('four_slot_candidate motion_validated=false; cannot change flag to bypass evidence')
    if "raise FatalGraspError('four_slot_exit_clearance_not_calibrated')" in source:blockers.append('executor deliberately inhibits post-release exit; red4 calibration alone cannot validate yellow path')
    # Explicit, fixed yellow-only runner contract, never fall back to old 4/8-part entry.
    if not (HERE/'run_yellow_single.py').exists():blockers.append('yellow-only feeder/RGB/placement/READY runner not yet qualified; old entry requires exactly four parts')
    return blockers

def calibration_pass(run):
    try:
        s=json.loads((run/'summary.json').read_text());rows=json.loads((run/'support-held-cycle.json').read_text())
        if s['outcome']!='fixture_cycle_pass' or s['observer_exit']!=0 or s['scene_exit']!=0:return False
        if not (run/'FINGER_DONE').exists() or (run/'guard-fault.json').exists():return False
        expected=['depart_32mm','exit_above_bin','return_above_support','land_original_pose']
        if [r['stage'] for r in rows]!=expected:return False
        return all(r['action']['status']==4 and r['action']['result']['error_code']==0 and r.get('post_action_stability') and r.get('actual_native') and r.get('independent_checks') for r in rows) and (run/'support-contact-permit.json').exists() and json.loads((run/'support-contact-permit.json').read_text())['phase']=='normal'
    except (FileNotFoundError,KeyError,ValueError):return False

def main():
    p=argparse.ArgumentParser();g=p.add_mutually_exclusive_group(required=True);g.add_argument('--check',action='store_true');g.add_argument('--approval-file',type=Path);a=p.parse_args()
    if a.check:
        print(json.dumps({'no_execution':True,'stage1':'one red4 loaded calibration, max180s','stage2':'fresh world, yellow-only; conditional','yellow_blockers':yellow_preflight()},ensure_ascii=False,indent=2));return 0
    approval=json.loads(a.approval_file.read_text());sha=subprocess.check_output(['git','rev-parse','HEAD'],cwd=WS,text=True).strip()
    if approval.get('approved') is not True or approval.get('scope')!='red4-then-yellow-single-once' or approval.get('commit')!=sha:raise SystemExit('REFUSED: new explicit two-stage authorization bound to current commit required')
    if subprocess.check_output(['git','status','--porcelain'],cwd=WS,text=True).strip():raise SystemExit('REFUSED: dirty sources')
    rid=str(uuid.UUID(approval['request_id']));claim=a.approval_file.with_name('CONSUMED-'+rid)
    with claim.open('x') as f:json.dump({'commit':sha,'wall':time.time()},f)
    root=ROOT/'feedback_fix'/('concentrated-'+rid);root.mkdir()
    child=dict(approval,scope='red4-support-cycle-once',request_id=str(uuid.uuid4()))
    token=root/'stage1-approval.json';token.write_text(json.dumps(child))
    result=subprocess.run([sys.executable,str(HERE/'calibrate_idle.py'),'--fingers','--arm-probe','--empty-slot','--release-fixture','--lift-fixture','--approval-file',str(token)],cwd=WS)
    run=Path((ROOT/'latest-calibration-run.txt').read_text().strip())
    if result.returncode or not calibration_pass(run):raise SystemExit('STOPPED: stage1 not fully proven; no stage2, no retry')
    blockers=yellow_preflight()
    if blockers:
        (root/'stage2-blocked.json').write_text(json.dumps(blockers,ensure_ascii=False,indent=2));raise SystemExit('BLOCKED before stage2: '+ '; '.join(blockers))
    # A qualified runner must itself verify the exact new-world, one-yellow scope.
    raise SystemExit(subprocess.call([sys.executable,str(HERE/'run_yellow_single.py'),'--parent-approval',str(a.approval_file),'--calibration-evidence',str(run)],cwd=WS))
if __name__=='__main__':main()
