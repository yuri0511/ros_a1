#!/usr/bin/env python3
"""Read-only artifact/adapter/legality checks. Never trains or launches Gazebo."""
import hashlib,json,random,sys
from pathlib import Path
WS=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WS/'codex_rl_v2'),str(WS/'src/a1_sorting_core')]
from dual_arm_rl.model import Model,config_hash,WAIT
from dual_arm_rl.qtable import choose
from a1_sorting_core.q_feed import resolve_q_paths,load_q_policy,decide_feed
p,c,t=resolve_q_paths('codex_rl_v2','codex_rl_v2/config/timing.v2.json','codex_rl_v2/artifacts/v2/q-v2-general.json',anchors=[WS])
cfg,q=load_q_policy(p,c,t)
assert cfg['schema_version']==q['schema_version']==2
assert not cfg['allow_prefeed_busy_arm'] and not cfg['overlap_validated'] and not q['deployment_ready']
assert config_hash(cfg)==q['config_sha256']=='ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b'
assert hashlib.sha256(Path(t).read_bytes()).hexdigest()=='18c4b343b0c359b2c740d9d6eeabef273be3d93e4e573ee28a32b53216977e2c'
n=unknown=steps=0
for x in range(9):
 for y in range(9-x):
  for z in range(9-x-y):
   m=Model(cfg,[x,y,z,8-x-y-z],rng=random.Random(202700));stats={};local=0
   while not m.done:
    a=choose(q,m,stats);assert a in m.legal_actions();m.step(a);local+=1;assert local<200
   assert m.completed==8;n+=1;unknown+=stats.get('unknown_state_fallback_steps',0);steps+=local
assert n==165
# Hidden future profiles must not change observable states or ETA.
a=Model(cfg,[2]*4,rng=random.Random(1));b=Model(cfg,[2]*4,rng=random.Random(999));assert a.state()==b.state() and a.legal_actions()==b.legal_actions();a.step(0);b.step(0);assert a.state()==b.state()
a=Model(dict(cfg,sampling_mode='mean'),[1,0,1,0]);a.step(0);now=a.time;post=cfg['post_pick_seconds']['blue'];a.step(WAIT);assert a.arms[0].phase=='READY' and abs(a.time-now-post)<1e-8
choice,stats=decide_feed(cfg,q,[2]*4,[{'phase':'READY','color':-1,'phase_start':0}]*2,0,p);assert choice and not stats.get('unknown_state_fallback_steps',0)
print(json.dumps({'read_only':True,'no_gazebo':True,'no_training':True,'schema':2,'package':p,'config_sha256':config_hash(cfg),'all_165_batches_complete_and_legal':n,'unknown_fallbacks':unknown,'model_decisions':steps,'future_hidden_and_eta_and_wait_tests':True,'initial_adapter_choice_not_forced':choice,'deployment_ready':q['deployment_ready']},indent=2))
