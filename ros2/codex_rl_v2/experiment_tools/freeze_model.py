import json,statistics,shutil,hashlib
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';P=R/'worktree/ros2/codex_rl_v2';shutil.copytree(I/'worktree/ros2/codex_rl/dual_arm_rl',P/'dual_arm_rl',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
rows=json.loads((R/'data/parts.json').read_text());train=[x for x in rows if x['split']=='calibration'];assert len(train)==32
c=json.loads((I/'training/config/timing.prefeed-false.json').read_text());c.update(schema_version=2,timing_model_version='local72ab-1ms-v3-empirical-timing-v2',request_delay_seconds=0.0,sampling_mode='empirical_profiles',calibration_scope='aggregate observed dispatch gates; incomplete safety/failure-probability calibration')
c['measurement_metadata']={'source':'data/SOURCE_MANIFEST.json and CALIBRATION_DATA.csv','clock':'native stats interpolated at source log wall times; exact ROS observed stamps separate','training_rounds':sorted({x['case'] for x in train}),'holdout_rounds':'pair3 rule/q; never used in parameter fitting','limitations':['Gate-home is position/stop-command evidence, not measured zero velocity.','READY means dispatcher task slot available after return attempt, not independently verified physical standby.','Zero retries in a color sample is not proof of zero probability.','Uniform resampling of observed whole-part profiles is an empirical sensitivity model, not calibrated physical failure probabilities.','Placement occurrence >2, alternate policy correlations and faults are uncalibrated.','Sim seconds only; wall seconds require measured load/RTF and startup/recorder costs.']}
c['profiles']={};c['stage_summary']={}
for color in c['color_order']:
 group=[x for x in train if x['color']==color];profiles=[]
 for x in group:
  profiles.append({'source_case':x['case'],'source_part':x['part'],'color':color,'feed':x['feed_to_pick_sim_s'],'advance_stop':x['request_spawn_sim_s']+x['advance_sim_s'],'home':x['request_spawn_sim_s']+x['advance_sim_s']+x['retract_sim_s'],'pick_nominal':x['pick_nominal_sim_s'],'retry_prefix':x['retry_prefix_sim_s'],'place':x['place_stage_sim_s'],'return_tail':x['return_tail_sim_s'],'post':x['post_sim_s'],'retries':x['retries'],'retry_reasons':x['retry_reasons']})
  assert 0<profiles[-1]['advance_stop']<=profiles[-1]['home']<=profiles[-1]['feed'] and all(profiles[-1][k]>=0 for k in ('pick_nominal','retry_prefix','place','return_tail','post'))
 c['profiles'][color]=profiles
 c['feed_seconds'][color]=statistics.mean(p['feed'] for p in profiles)
 c['pick_seconds'][color]=statistics.mean(p['pick_nominal']+p['retry_prefix'] for p in profiles)
 c['post_pick_seconds'][color]=statistics.mean(p['post'] for p in profiles)
 c['stage_summary'][color]={k:{'n':len(profiles),'mean':statistics.mean(p[k] for p in profiles),'min':min(p[k] for p in profiles),'max':max(p[k] for p in profiles)} for k in ('feed','advance_stop','home','pick_nominal','retry_prefix','place','return_tail','post')}
c['data_manifest_sha256']=hashlib.sha256((R/'data/SOURCE_MANIFEST.json').read_bytes()).hexdigest();(R/'training/config.v2.json').write_text(json.dumps(c,indent=2)+'\n');(P/'config').mkdir(exist_ok=True);shutil.copy2(R/'training/config.v2.json',P/'config/timing.v2.json')
# Preserve the upstream methods/legality and replace only timing/event observation hooks.
m=P/'dual_arm_rl/model.py';s=m.read_text();s=s.replace('import math','import math\nimport random')
s=s.replace('c.get("schema_version") != 1','c.get("schema_version") != 2')
s=s.replace('def __init__(self, config, batch):','def __init__(self, config, batch, rng=None):')
s=s.replace('        self.config = config','        self.config = config\n        self._rng = rng or random.Random(0)\n        self._profile = {}\n        self._ordinal = [0]*4\n        self._banks = {}\n        for color in range(4):\n            pool=config["profiles"][COLORS[color]]\n            self._banks[color]=[dict(self._rng.choice(pool)) for _ in range(batch[color])]')
s=s.replace('        return self.config[field][COLORS[color]]','''        p=self._profile.get(color)
        if p is None:return self.config[field][COLORS[color]]
        return p[{'feed_seconds':'feed','pick_seconds':'pick_total','post_pick_seconds':'post'}[field]]''')
s=s.replace('estimate = self._duration("post_pick_seconds", arm.color)','estimate = self.config["post_pick_seconds"][COLORS[arm.color]]').replace('estimate += self._duration("pick_seconds", arm.color)','estimate += self.config["pick_seconds"][COLORS[arm.color]]')
s=s.replace('        self._schedule(self._duration("pick_seconds", color), "PICK_EXIT", i)','''        retry=self._profile[color]['retry_prefix']
        if retry>0:self._schedule(retry,'RETRY_PREFIX_END',i)
        self._schedule(self._duration("pick_seconds", color), "PICK_EXIT", i)''')
s=s.replace('                self._schedule(self._duration("feed_seconds", color), "FEED_READY", None)','''                p=self._profile[color]
                self._schedule(p['advance_stop'],'RETRACT_START',None)
                self._schedule(p['home'],'HOME_GATE',None)
                self._schedule(self._duration("feed_seconds", color), "FEED_READY", None)''')
s=s.replace('            elif event == "FEED_READY":','''            elif event in ('RETRACT_START','HOME_GATE'):
                self._log(event,None,self.slot[0],self.slot[1])
            elif event in ('RETRY_PREFIX_END','PLACE_FINISHED'):
                a=self.arms[i];self._log(event,i,a.color,a.part)
            elif event == "FEED_READY":''')
s=s.replace('                self._schedule(self._duration("post_pick_seconds", arm.color), "ARM_READY", i)','''                self._schedule(self._profile[arm.color]['place'],'PLACE_FINISHED',i)
                self._schedule(self._duration("post_pick_seconds", arm.color), "ARM_READY", i)''')
s=s.replace('            self.remaining[action] -= 1','''            self._profile[action]=self._select_profile(action)
            self.remaining[action] -= 1''')
insert='''    def _select_profile(self,color):
        pool=self.config['profiles'][COLORS[color]];mode=self.config.get('sampling_mode','empirical_profiles')
        if mode in ('mean','nominal'):
            p={k:sum(row[k] for row in pool)/len(pool) for k in ('feed','advance_stop','home','pick_nominal','retry_prefix','place','return_tail','post')}
            if mode=='nominal':p['retry_prefix']=0.
        else:p=dict(self._banks[color][self._ordinal[color]])
        self._ordinal[color]+=1
        p['pick_total']=p['pick_nominal']+p['retry_prefix']
        return p

'''
s=s.replace('        if action == WAIT:\n            self._next_events()', '        if action == WAIT:\n            completed_before = self.completed\n            # ROS queries again on a real terminal/zone event, not an internal\n            # placement checkpoint nor the observer adapter dummy 1s event.\n            while self.events and self.completed == completed_before:\n                self._next_events()')
s=s.replace('    def _duration(self, field, color):',insert+'    def _duration(self, field, color):');compile(s,str(m),'exec');m.write_text(s)
q=P/'dual_arm_rl/qtable.py';s=q.read_text().replace('"schema_version": 1','"schema_version": 2');q.write_text(s)
t=P/'dual_arm_rl/train.py';s=t.read_text().replace('Model(config, counts)','Model(config, counts, rng=rng)');t.write_text(s)
print('Frozen V2 config:',c['data_manifest_sha256'],'32 train pieces, 16 held out,16 diagnostic excluded')
