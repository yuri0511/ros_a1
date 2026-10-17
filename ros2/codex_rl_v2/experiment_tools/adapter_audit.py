import json,sys,hashlib
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';P=R/'worktree/ros2/codex_rl_v2';sys.path.insert(0,str(P));sys.path.insert(0,str(R/'worktree/ros2/src/a1_sorting_core'))
from dual_arm_rl.model import Model
from dual_arm_rl.qtable import choose,state_key
from a1_sorting_core.q_feed import load_q_policy,resolve_q_paths,decide_feed
package,cfg,table=resolve_q_paths(str(P),str(R/'training/config.v2.json'),str(R/'training/q-v2-demo.json'));c,q=load_q_policy(package,cfg,table);rows=[]
for case in sorted((I/'runs').iterdir()):
 audit=json.loads((case/'policy-analysis.json').read_text())[0]
 for d in audit['decision_audit']:
  m=Model(c,d['remaining']);m.time=d['sim']
  for arm,obs in zip(m.arms,d['arms']):arm.phase=obs['phase'];arm.color=obs['color'];arm.phase_start=obs['phase_start']
  if any(a.phase!='READY' for a in m.arms):m._schedule(1.,'ARM_READY',0)
  a=choose(q,m);choice,stats=decide_feed(c,q,d['remaining'],d['arms'],d['sim'],str(P));expected=None if a==4 else ('arm'+str(c['color_to_arm'][a]+1),c['color_order'][a]);assert choice==expected
  rows.append({'case':case.name,'old_key':d['state_key'],'v2_key':state_key(m.state()),'same_key':d['state_key']==state_key(m.state()),'legal':m.legal_actions(),'v2_action_on_old_observation':a,'unknown':stats.get('unknown_state_fallback_steps',0),'not_a_new_policy_rollout':True})
result={'observations':len(rows),'changed_eta_keys':sum(not x['same_key'] for x in rows),'unknown_on_recorded_observations':sum(x['unknown'] for x in rows),'native_unchanged_q_feed_sha256':hashlib.sha256((R/'worktree/ros2/src/a1_sorting_core/a1_sorting_core/q_feed.py').read_bytes()).hexdigest(),'schema2_loader_via_package_path_passed':True,'rows':rows,'warning':'Semantic adapter compatibility only. No Gazebo deployment or fault-ready/safe-clear certification. Existing dummy1s event is used for legal WAIT availability only, never stepped by ROS.'};(R/'audit/ADAPTER_AUDIT.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='rows'})
