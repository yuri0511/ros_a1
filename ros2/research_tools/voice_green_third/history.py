from pathlib import Path
import json,gzip,csv,hashlib
from geometry import fk
R=Path(__file__).resolve().parents[1]
D=Path('/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1')
samples=[];controllers=[];last=-1
with gzip.open(D/'high-rate.jsonl.gz','rt') as f:
    for line in f:
        d=json.loads(line)
        if not 1792178298<=d.get('wall',0)<=1792178320:continue
        if d['kind']=='joint_state' and d['stamp']>last+.009:
            last=d['stamp'];v=dict(zip(d['names'],d['positions']))
            samples.append({'wall':d['wall'],'stamp':d['stamp'],'positions':v,
                'tool0_xyz':fk('arm2_tool0',v)[:3,3].tolist()})
        if d['kind']=='controller_state' and d.get('controller')=='/arm2_joint_trajectory_controller':
            if not controllers or d['wall']-controllers[-1]['wall']>.02:controllers.append(d)
(R/'audit/history-green1-joints.json').write_text(json.dumps(samples))
(R/'audit/history-green1-controller.json').write_text(json.dumps(controllers))
lo=min(s['stamp'] for s in samples);hi=max(s['stamp'] for s in samples)
contacts=[]
with (D/'contacts.csv').open(newline='') as f:
    for d in csv.DictReader(f):
        if lo<=float(d['sim_sec'])<=hi and 'bin_green' in d['collision1']+d['collision2']:contacts.append(d)
with (R/'audit/history-green1-bin-contacts.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(contacts[0]) if contacts else ['sim_sec']);w.writeheader();w.writerows(contacts)
summary={'source':str(D),'sample_count':len(samples),'sim_window':[lo,hi],
 'minimum_tool0_z':min(s['tool0_xyz'][2] for s in samples),
 'bin_contact_points':len(contacts),'max_bin_depth_m':max([float(c['depth']) for c in contacts],default=0),
 'contact_pairs':sorted(set((c['collision1'],c['collision2']) for c in contacts))}
(R/'audit/history-green1-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
