import sys,json,random,statistics,copy,math,itertools,time
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2'
sys.path.insert(0,str(R/'worktree/ros2/codex_rl_v2'))
sys.path.insert(0,str(R/'worktree/ros2/src/a1_sorting_core'))
from dual_arm_rl.model import Model,load_config,config_hash,COLORS,WAIT
from dual_arm_rl.qtable import load,choose,state_key
from a1_sorting_core.pipeline_logic import choose_idle_feed
def key(m):
 r=lambda t:round(t-m.time,9)
 # Sequence ordinals preserve co-timed event order without absolute IDs.
 ev=tuple((r(t),kind,arm) for t,seq,kind,arm in sorted(m.events))
 return (tuple(m.remaining),tuple((a.phase,a.color,r(a.phase_start) if a.phase!='READY' else 0) for a in m.arms),ev,None if m.slot is None else (m.slot[0],m.slot[2]),m.pick_owner,m.completed,m.requested,tuple(m._ordinal))
class Solver:
 def __init__(self,objective='reward'):self.memo={};self.objective=objective;self.nodes=0;self.wait_edges=0
 def solve(self,m):
  if m.done:return (0.,None)
  k=key(m)
  if k in self.memo:return self.memo[k]
  best=(math.inf,None);self.nodes+=1
  for a in m.legal_actions():
   n=copy.deepcopy(m);n.trace=[];t=n.time;_,reward,_=n.step(a)
   self.wait_edges+=a==WAIT
   edge=-reward if self.objective=='reward' else n.time-t
   total=edge+self.solve(n)[0]
   if total<best[0]-1e-9:best=(total,a)
  if best[1] is None:raise RuntimeError('Nonterminal deadlock')
  self.memo[k]=best;return best
 def policy(self,m):return self.solve(m)[1]
def ros_rule(m):
 legal=m.legal_actions();idle=[f'arm{i+1}' for i,a in enumerate(m.arms) if a.phase=='READY' and any(c in legal for c in range(4) if m.config['color_to_arm'][c]==i)]
 counts=[m.initial[i]-m.remaining[i] for i in range(4)];last=next((f"arm{m.config['color_to_arm'][e['color']]+1}" for e in reversed(m.trace) if e['event']=='FEED_RESERVED'),None)
 choice=choose_idle_feed(idle,last,dict(zip(COLORS,counts)))
 if choice is None:return WAIT
 a=COLORS.index(choice[1])
 if a not in legal:
  xs=[i for i in legal if i!=WAIT and m.config['color_to_arm'][i]==int(choice[0][-1])-1]
  return min(xs,key=lambda i:(counts[i],i)) if xs else WAIT
 return a

def run(c,b,policy,table=None,seed=2027):
 m=Model(c,b,rng=random.Random(seed));stats={};actions=[]
 while not m.done:
  action=choose(table,m,stats) if table is not None else policy(m)
  assert action in m.legal_actions();actions.append({'sim':m.time,'state':m.state(),'action':action});m.step(action)
  assert len(actions)<1000
 return {'time':m.time,'idle':m.idle,'cost':m.time+c['idle_weight']*m.idle,'completed':m.completed,'actions':actions,'order':[COLORS[x['color']] for x in m.trace if x['event']=='FEED_RESERVED'],'trace':m.trace,'stats':stats}

def summary(xs):
 s=sorted(xs)
 def q(p):idx=(len(s)-1)*p;lo=math.floor(idx);hi=math.ceil(idx);return s[lo]*(hi-idx)+s[hi]*(idx-lo) if lo!=hi else s[lo]
 return {'n':len(s),'mean':statistics.mean(s),'sd':statistics.stdev(s) if len(s)>1 else 0,'p05':q(.05),'p50':q(.5),'p95':q(.95),'min':s[0],'max':s[-1]}

def main():
 c=load_config(R/'training/config.v2.json');tables={'q_v1_prefeed_false':json.loads((I/'training/artifacts/q-demo-2222.json').read_text()),'q_yuri_history':json.loads((I/'training/history/q_table.json').read_text()),'q_v2_demo':load(R/'training/q-v2-demo.json',c),'q_v2_general':load(R/'training/q-v2-general.json',c)}
 policies={'rule':None,**tables};out={'config_sha256':config_hash(c),'mean_models':{},'empirical_common_scenarios':{},'empirical_independent_scenarios':{},'holdout_predictions':[],'coverage':[],'tests':[]}
 for mode in ('nominal','mean'):
  cfg=dict(c,sampling_mode=mode);group={}
  for name,table in policies.items():group[name]=run(cfg,[2]*4,ros_rule,table)
  for obj in ('reward','makespan'):
   solver=Solver(obj);solver.solve(Model(cfg,[2]*4));ev=run(cfg,[2]*4,solver.policy);ev.update(dp_nodes=solver.nodes,wait_edges=solver.wait_edges);group['dp_'+obj]=ev
  for name,v in group.items():v['gap_makespan']=v['time']-group['dp_makespan']['time'];v['gap_weighted_cost']=v['cost']-group['dp_reward']['cost']
  out['mean_models'][mode]=group
 # Independent seed family never used in training. Each common scenario reuses
 # the same color/ordinal profile bank, but Q cannot observe future draws.
 mc={name:[] for name in policies};ind={name:[] for name in policies};deltas={};unknown={name:[0,0] for name in tables}
 for k in range(200):
  seed=202700+k
  for name,table in policies.items():
   result=run(c,[2]*4,ros_rule,table,seed);mc[name].append(result['time']);ind[name].append(run(c,[2]*4,ros_rule,table,303700+k+(0 if name=='rule' else 10000))['time'])
   if table is not None:unknown[name][0]+=result['stats'].get('unknown_state_fallback_steps',0);unknown[name][1]+=result['stats'].get('decision_steps',0)
 for name in policies:
  out['empirical_common_scenarios'][name]=summary(mc[name]);out['empirical_independent_scenarios'][name]=summary(ind[name])
  if name!='rule':
   d=[a-b for a,b in zip(mc[name],mc['rule'])];di=[a-b for a,b in zip(ind[name],ind['rule'])];out['empirical_common_scenarios'][name]['q_minus_rule']=summary(d);out['empirical_independent_scenarios'][name]['q_minus_rule']=summary(di);out['empirical_independent_scenarios'][name]['slower_fraction']=sum(x>0 for x in di)/len(di)
   out['empirical_common_scenarios'][name]['unknown_fallbacks'],out['empirical_common_scenarios'][name]['decisions']=unknown[name]
 # Clairvoyant oracle knows a fixed scenario's future color/ordinal samples.
 oracles=[]
 for seed in range(202700,202720):
  solver=Solver('makespan');value=solver.solve(Model(c,[2]*4,rng=random.Random(seed)))[0]
  q=run(c,[2]*4,ros_rule,tables['q_v2_demo'],seed);oracles.append({'seed':seed,'clairvoyant_min_time':value,'q_time':q['time'],'gap':q['time']-value,'nodes':solver.nodes,'wait_edges':solver.wait_edges})
 out['clairvoyant_oracles']=oracles
 def prescribed(order):
  queue=list(map(COLORS.index,order))
  def policy(m):
   a=queue[0]
   if a in m.legal_actions():queue.pop(0);return a
   return WAIT
  return policy
 batches=json.loads((R/'data/batches.json').read_text())
 for real in batches:
  if real['split']=='diagnostic_contact_only':continue
  pred={mode:run(dict(c,sampling_mode=mode),[2]*4,prescribed(real['feed_order']))['time'] for mode in ('nominal','mean')}
  out['holdout_predictions'].append({'case':real['case'],'split':real['split'],'policy':real['policy'],'conditioned_on_observed_feed_order':True,'observed_sim_s':real['sim']['total'],'predictions':pred,'errors':{k:v-real['sim']['total'] for k,v in pred.items()},'note':'No heldout retry outcomes used in parameters. Mean predicts expected empirical burden, not this round actual retries.'})
 # Teacher-forced trace replay checks event bookkeeping, not heldout forecasting.
 rows=json.loads((R/'data/parts.json').read_text());replay=[]
 for real in batches:
  if real['split']=='diagnostic_contact_only':continue
  bank={color:[] for color in COLORS}
  for row in rows:
   if row['case']!=real['case']:continue
   bank[row['color']].append({'feed':row['feed_to_pick_sim_s'],'advance_stop':row['request_spawn_sim_s']+row['advance_sim_s'],'home':row['request_spawn_sim_s']+row['advance_sim_s']+row['retract_sim_s'],'pick_nominal':row['pick_nominal_sim_s'],'retry_prefix':row['retry_prefix_sim_s'],'place':row['place_stage_sim_s'],'return_tail':row['return_tail_sim_s'],'post':row['post_sim_s']})
  m=Model(c,[2]*4);m._banks={i:bank[color] for i,color in enumerate(COLORS)};pol=prescribed(real['feed_order'])
  while not m.done:m.step(pol(m))
  replay.append({'case':real['case'],'teacher_forced_sim':m.time,'observed_sim':real['sim']['total'],'residual':m.time-real['sim']['total'],'not_a_forecast':True})
 out['teacher_forced_replay']=replay
 for x,y,z in itertools.product(range(9),repeat=3):
  if x+y+z>8:continue
  b=[x,y,z,8-x-y-z]
  for name in ('q_v2_demo','q_v2_general'):
   ev=run(c,b,ros_rule,tables[name],404700+x*81+y*9+z);out['coverage'].append({'name':name,'batch':b,'completed':ev['completed'],**ev['stats']})
 # No future samples appear in the observable state or legality.
 a=Model(c,[2]*4,rng=random.Random(1));b=Model(c,[2]*4,rng=random.Random(999));assert a.state()==b.state() and a.legal_actions()==b.legal_actions();out['tests'].append('future_bank_hidden_at_start')
 a.step(0);b.step(0);assert a.state()==b.state();out['tests'].append('sampled_post_time_not_leaked_in_eta')
 a=Model(dict(c,sampling_mode='mean'),[1,0,1,0]);a.step(0);t=a.time;post=a.config['post_pick_seconds']['blue'];a.step(WAIT);assert a.arms[0].phase=='READY' and abs(a.time-t-post)<1e-8;out['tests'].append('WAIT_skips_placement_and_wakes_on_real_ARM_READY')
 def brute(m):
  if m.done:return 0.
  values=[]
  for action in m.legal_actions():
   n=copy.deepcopy(m);_,reward,_=n.step(action);values.append(-reward+brute(n))
  return min(values)
 for batch in ([1,0,1,0],[1,1,1,0],[0,0,0,2]):
  cfg=dict(c,sampling_mode='mean');solver=Solver();m=Model(cfg,batch);assert abs(solver.solve(m)[0]-brute(m))<1e-7
 out['tests'].append('DP_matches_independent_exhaustive_small_batches_including_WAIT')
 (R/'training/EVALUATION_V2.json').write_text(json.dumps(out,indent=2)+'\n')
 print(json.dumps({mode:{name:{k:ev[k] for k in ('time','cost','gap_makespan')} for name,ev in group.items()} for mode,group in out['mean_models'].items()},indent=2),flush=True)
 print('ALL MODEL TESTS PASSED',out['tests'],flush=True)
if __name__=='__main__':main()
