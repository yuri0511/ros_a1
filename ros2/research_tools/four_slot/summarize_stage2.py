"""Summarize recorded evidence, never promote the dynamic gate automatically."""
from pathlib import Path
import csv,json,hashlib
ROOT=Path(__file__).resolve().parents[4];D=ROOT/'audit/stage2';W=ROOT/'worktree/ros2'
plans=json.loads((D/'plans.json').read_text());sweep=json.loads((D/'dense-sweep.json').read_text());cfg=json.loads((W/'src/a1_sorting_core/config/four_slot_candidate.json').read_text())
with (D/'PATH_CLEARANCE.csv').open('w') as f:
 writer=csv.writer(f,lineterminator="\n");writer.writerow(['slot','stage','samples','sat_separation_m','object_a','object_b','trajectory_time_s','hits','physical_validated'])
 for row in sweep['segments']:
  w=row['minimum_witness'];writer.writerow([row['slot'],row['stage'],row['samples'],row['minimum_separating_gap'],w['a'],w['b'],w['time'],row['hits'],False])
with (D/'SLOT_STAGE_STATUS.csv').open('w') as f:
 writer=csv.writer(f,lineterminator="\n");writer.writerow(['slot','historical_carry_clear_cases','plan_successes','conditional_sweep_hits','release_feedback_new_world','physical_exit_clearance','status'])
 for bn,b in cfg['bins'].items():
  for slot in b['slots']:
   sid=slot['id'];rp=[r for r in plans if r['slot']==sid];rs=[r for r in sweep['segments'] if r['slot']==sid]
   writer.writerow([sid,4,sum(r['response']['motion_plan_response']['error_code']['val']==1 for r in rp),sum(r['hits'] for r in rs),'NOT_MEASURED','NOT_CALIBRATED','BLOCKED'])
witness=min(sweep['segments'],key=lambda r:r['minimum_separating_gap']);(D/'MINIMUM_WITNESS.json').write_text(json.dumps(witness,indent=2))
inputs=[ROOT/'audit/robot-static.urdf',W/'src/a1_cell_sim/worlds/four_slot_1ms.sdf',W/'src/a1_sorting_core/config/four_slot_candidate.json',W/'src/a1_cell_sim/config/ur5e_cell_controllers_stability_gain1.yaml',D/'historical-carry-clear.json',D/'plans.json',D/'dense-sweep.json']
(D/'INPUT_SHA256.txt').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p)+'\n' for p in inputs))
summary={'stage':2,'dynamic_execution_allowed':False,'status':'BLOCKED','planning_success':sum(r['response']['motion_plan_response']['error_code']['val']==1 for r in plans),'planning_total':len(plans),'dense_samples':sweep['samples'],'dense_hits':sum(r['hits'] for r in sweep['segments']),'minimum_sat_projection_gap_m':witness['minimum_separating_gap'],'runtime_scene_service_test':'PASS with synthetic observations, no Gazebo','blockers':['New release/opening physical response and complete exit clearance not calibrated; continuous evidence guard is not proof of safe stopping distance.','Held object actual 6D pose, deposited pose uncertainty, geometry error and sampling/tracking error lack certified bounds.','Historical carry_clear starts rounded to 0.0001 rad; new-world start distribution not measured.','Other arm standby only; no new-world simultaneous-arm or native camera pose validation.','Command spline sampling plus planner collision checks do not certify continuous physical trajectories.'],'gazebo_runs':0,'paid_api_calls':0}
(D/'gate-status.json').write_text(json.dumps(summary,indent=2));print(summary)
