import json,statistics,subprocess,math,hashlib
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';data=json.loads((R/'data/batches.json').read_text());rows=json.loads((R/'data/parts.json').read_text());ev=json.loads((R/'training/EVALUATION_V2.json').read_text())
code="""import sys,json
from pathlib import Path
I=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');R=I.parent/'model_calibration_v2';sys.path.insert(0,str(I/'worktree/ros2/codex_rl'))
from dual_arm_rl.model import Model,load_config,COLORS,WAIT
from dual_arm_rl.qtable import load
c=load_config(I/'training/config/timing.prefeed-false.json');load(I/'training/artifacts/q-demo-2222.json',c);out={}
for d in json.loads((R/'data/batches.json').read_text()):
 if d['split']=='diagnostic_contact_only':continue
 q=list(map(COLORS.index,d['feed_order']));m=Model(c,[2]*4)
 while not m.done:
  a=q.pop(0) if q[0] in m.legal_actions() else WAIT;m.step(a)
 out[d['case']]=m.time
print(json.dumps(out))
"""
old=json.loads(subprocess.run(['python3','-c',code],capture_output=True,text=True,check=True).stdout)
for d in ev['holdout_predictions']:d['predictions']['old_v1']=old[d['case']];d['errors']['old_v1']=old[d['case']]-d['observed_sim_s']
metrics={}
for split in ('calibration','holdout'):
 g=[x for x in ev['holdout_predictions'] if x['split']==split];metrics[split]={mode:{'n':len(g),'mae_sim_s':statistics.mean(abs(x['errors'][mode]) for x in g),'bias_sim_s':statistics.mean(x['errors'][mode] for x in g),'max_abs_error_sim_s':max(abs(x['errors'][mode]) for x in g)} for mode in ('old_v1','nominal','mean')}
low=[x for x in data if x['split']!='diagnostic_contact_only'];decomp={}
for clock in ('wall','sim'):
 decomp[clock]={k:statistics.mean(x[clock][k] for x in low if x['policy']=='q')-statistics.mean(x[clock][k] for x in low if x['policy']=='rule') for k in low[0][clock]}
for k in ('startup_wall_s','recorder_tail_wall_s','csv_measurement_wall_s'):decomp[k]=statistics.mean(x[k] for x in low if x['policy']=='q')-statistics.mean(x[k] for x in low if x['policy']=='rule')
orders=json.loads((I/'training/model-evaluation.json').read_text())['policies'];expected={'rule':orders['rule']['feed_order'],'q':orders['new_q_demo']['feed_order']};matches={d['case']:d['feed_order']==expected[d['policy']] for d in data}
reasons={}
for x in rows:
 if x['split']=='diagnostic_contact_only':continue
 for reason in x['retry_reasons'].split('|'):
  if reason:reasons[(x['policy'],x['color'],reason)]=reasons.get((x['policy'],x['color'],reason),0)+1
# Pilot planning uses three observed complete paired differences; not an effect estimate guarantee.
deltas=[]
for pair in ('pair1','pair2','pair3'):
 g={x['policy']:x for x in low if pair in x['case']};deltas.append(g['q']['csv_measurement_wall_s']-g['rule']['csv_measurement_wall_s'])
sd=statistics.stdev(deltas);rmean=statistics.mean(x['csv_measurement_wall_s'] for x in low if x['policy']=='rule');target=rmean*(1-122.32/126.49);normal_n=math.ceil(((1.96+.8416)*sd/target)**2)
a={'validation':metrics,'predictions':ev['holdout_predictions'],'critical_chain_q_minus_rule':decomp,'orders_match_old_model':matches,'retry_reasons':[{'policy':p,'color':c,'reason':reason,'count':n} for (p,c,reason),n in reasons.items()],'pilot_paired_differences_wall_s':deltas,'pilot_sd_wall_s':sd,'planning_target_effect_wall_s':target,'normal_approx_pairs_for_80pct_power':normal_n,'prospective_recommendation_pairs':12,'warning':'Only two heldout rounds, outcome history already public. Mean empirical model is not a validated failure probability model.'}
(R/'data/ASSESSMENT.json').write_text(json.dumps(a,indent=2));print(json.dumps(metrics,indent=2));print('Planning',sd,target,normal_n);print('Order matches',all(matches.values()))
