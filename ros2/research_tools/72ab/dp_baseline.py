"""Exact full-event-state DP. Uses the unchanged upstream Model.step/legal_actions.

The tabular Q observation is aliased (ETA bins); DP may inspect exact remaining
internal event times. This is an explicit model-information upper bound.
"""
import sys,copy,json,time,math,csv,hashlib,itertools
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');sys.path.insert(0,str(R/'worktree/ros2/codex_rl'))
from dual_arm_rl.model import Model,load_config,run,rule_action,config_hash,COLORS,WAIT
from dual_arm_rl.qtable import load,choose,state_key
sys.path.insert(0,str(R/'worktree/ros2/src/a1_sorting_core'))
from a1_sorting_core.pipeline_logic import choose_idle_feed

def key(m):
 r=lambda t:round(t-m.time,9)
 # Sequence ordinals preserve co-timed event order without absolute IDs.
 ev=tuple((r(t),kind,arm) for t,seq,kind,arm in sorted(m.events))
 return (tuple(m.remaining),tuple((a.phase,a.color,r(a.phase_start) if a.phase!='READY' else 0) for a in m.arms),ev,None if m.slot is None else (m.slot[0],m.slot[2]),m.pick_owner,m.completed,m.requested)
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
    # Upstream ROS rule balances per-color counts and alternates idle arms.
    legal=m.legal_actions();idle=[f'arm{i+1}' for i,a in enumerate(m.arms) if a.phase=='READY' and any(c in legal for c in range(4) if m.config['color_to_arm'][c]==i)]
    requested=[m.initial[i]-m.remaining[i] for i in range(4)]
    last=next((f"arm{m.config['color_to_arm'][e['color']]+1}" for e in reversed(m.trace) if e['event']=='FEED_RESERVED'),None)
    choice=choose_idle_feed(idle,last,dict(zip(COLORS,requested)))
    if choice is None:return WAIT
    action=COLORS.index(choice[1])
    if action not in legal:
        # General unequal demand: skip exhausted colors using same count tie order.
        colors=[i for i in legal if i!=WAIT and m.config['color_to_arm'][i]==int(choice[0][-1])-1]
        if not colors:return WAIT
        action=min(colors,key=lambda i:(requested[i],i))
    return action

def evaluate_policy(c,b,policy,table=None):
 stats={}
 def wrapped(m):return choose(table,m,stats)
 m=run(c,b,wrapped if table is not None else policy)
 return {'makespan_model_s':m.time,'demand_idle_arm_s':m.idle,'weighted_cost':m.time+c['idle_weight']*m.idle,'completed':m.completed,'first_color':COLORS[next(x['color'] for x in m.trace if x['event']=='FEED_RESERVED')],'feed_order':[COLORS[x['color']] for x in m.trace if x['event']=='FEED_RESERVED'],**stats,'trace':m.trace}
def brute(m,objective):
 if m.done:return 0.
 vals=[]
 for a in m.legal_actions():
  n=copy.deepcopy(m);t=n.time;_,rew,_=n.step(a);vals.append((-rew if objective=='reward' else n.time-t)+brute(n,objective))
 return min(vals)
def main():
 c=load_config(R/'training/config/timing.prefeed-false.json');oldc=load_config(R/'training/history/timing.gazebo-v1.json');old=load(R/'training/history/q_table.json',oldc);new=load(R/'training/artifacts/q-demo-2222.json',c);general=load(R/'training/artifacts/q-general-8.json',c)
 # Independent exhaustive check includes WAIT branches and both objectives.
 checks=[]
 for b in ([1,0,1,0],[1,1,1,0],[0,0,0,2],[0,0,0,0]):
  for obj in ('reward','makespan'):
   dp=Solver(obj);x=dp.solve(Model(c,b))[0];y=brute(Model(c,b),obj);assert abs(x-y)<1e-8;(checks.append({'batch':b,'objective':obj,'dp':x,'exhaustive':y}))
 b=[2,2,2,2];outputs={};start=time.monotonic()
 outputs['rule']=evaluate_policy(c,b,ros_rule);outputs['upstream_model_rule']=evaluate_policy(c,b,'rule');outputs['old_q_legal_filtered_historical']=evaluate_policy(c,b,None,old);outputs['new_q_demo']=evaluate_policy(c,b,None,new);outputs['new_q_general']=evaluate_policy(c,b,None,general)
 for obj in ('reward','makespan'):
  solver=Solver(obj);value,_=solver.solve(Model(c,b));ev=evaluate_policy(c,b,solver.policy);ev.update(dp_value=value,states=solver.nodes,wait_edges=solver.wait_edges,full_internal_event_state=True);outputs['dp_'+obj]=ev
 for name,v in outputs.items():
  v['gap_to_dp_weighted_cost']=v['weighted_cost']-outputs['dp_reward']['weighted_cost'];v['gap_to_dp_makespan_s']=v['makespan_model_s']-outputs['dp_makespan']['makespan_model_s']
 # All 165 eight-piece multisets; same corrected config, separate histories.
 grid=[]
 for counts in itertools.product(range(9),repeat=3):
  if sum(counts)>8:continue
  batch=[*counts,8-sum(counts)]
  for name,table in [('new_q_demo',new),('new_q_general',general),('old_q_legal_filtered_historical',old)]:
   ev=evaluate_policy(c,batch,None,table);ev.pop('trace');grid.append({'policy':name,'batch':batch,**ev})
 result={'base_commit':'72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd','config_sha256':config_hash(c),'batch':b,'model_only':True,'ros_rule_boundary':'Default2222 uses exact upstream choose_idle_feed balancing/idle alternation with Model resources; model rule_action is reported separately. Unequal demand adapter skips exhausted colors.', 'historical_adapter':'Old table validated against original config; immutable q_values evaluated on corrected Model using legal-action filtering. Not falsely relabeled load-compatible.','dp_information_limit':'DP exact internal event times; Q uses binned observation, so exact DP is a model upper bound, not proof a stationary binned Q can represent it.','offline_wall_s':time.monotonic()-start,'small_exhaustive_checks':checks,'policies':outputs,'other_batches':grid}
 (R/'training/model-evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({k:{x:v[x] for x in ('makespan_model_s','weighted_cost','first_color','gap_to_dp_makespan_s','gap_to_dp_weighted_cost')} for k,v in outputs.items()},indent=2),flush=True)
if __name__=='__main__':main()
