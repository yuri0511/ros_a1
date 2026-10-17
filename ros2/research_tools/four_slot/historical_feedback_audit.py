"""Read existing physical evidence only; do not treat old paths as new validation."""
import gzip,json,re,hashlib
import numpy as np
from geometry import fk
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];out=ROOT/'audit/stage2'
base=Path('/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1')
windows=[];sources=[]
for path in sorted((base/'ros').glob('*.log')):
    text=path.read_text();pending=None;part=None;arm=None
    for line in text.splitlines():
        t=re.search(r'\[([0-9]+\.[0-9]+)\]',line)
        if not t:continue
        if '开始: part_' in line:
            part=re.search(r'开始: (part_\d+)',line)[1];arm='arm2' if '[pick_place_relay]' in line else 'arm1'
        if '已解除附着' in line and part:pending=float(t[1])
        if pending and 'CONTROLLER_RESULT' in line and '"label": "settle_rise"' in line:
            windows.append({'part':part,'arm':arm,'start_wall':pending,'end_wall':float(t[1]),'samples':0,'violating_samples':0,'minimum':[1e9,1e9],'maximum':[-1e9,-1e9],'max_speed':[0.,0.],'max_joint_tracking_error_rad':0.,'controller_samples':0,'max_tool_reference_feedback_m':0.,'source':str(path)});pending=None
    if '已解除附着' in text:sources.append({'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
p=base/'high-rate.jsonl.gz';kinds={}
with gzip.open(p,'rt') as f:
    for line in f:
        if not line.strip():continue
        r=json.loads(line);kind=r.get('kind');kinds[kind]=kinds.get(kind,0)+1
        for w in windows:
            if not w['start_wall']<=r['wall']<=w['end_wall']:continue
            if kind=='joint_state':
                names=r['names'];pre='' if w['arm']=='arm1' else 'arm2_';indices=[names.index(pre+'gripper_finger_'+side+'_joint') for side in ('left','right')]
                q=[r['positions'][i] for i in indices];v=[r['velocities'][i] for i in indices];w['samples']+=1
                w['violating_samples']+=int(any(x<.008 or x>.014 for x in q))
                for i in range(2):w['minimum'][i]=min(w['minimum'][i],q[i]);w['maximum'][i]=max(w['maximum'][i],q[i]);w['max_speed'][i]=max(w['max_speed'][i],abs(v[i]))
            elif kind=='controller_state' and ('arm2' in r['controller'])==(w['arm']=='arm2'):
                m=r['message'];error=max(abs(a-b) for a,b in zip(m['reference']['positions'],m['feedback']['positions']));w['max_joint_tracking_error_rad']=max(w['max_joint_tracking_error_rad'],error);w['controller_samples']+=1
                tool=('arm2_' if w['arm']=='arm2' else '')+'tool0'
                a=fk(tool,dict(zip(m['joint_names'],m['reference']['positions'])))
                b=fk(tool,dict(zip(m['joint_names'],m['feedback']['positions'])))
                w['max_tool_reference_feedback_m']=max(w['max_tool_reference_feedback_m'],float(np.linalg.norm(a[:3,3]-b[:3,3])))
sources.append({'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(out/'historical-release-feedback.json').write_text(json.dumps({'sources':sources,'windows':windows,'kinds':kinds,'scope':'old world and old release command; sample maxima NOT calibrated future bounds'},ensure_ascii=False,indent=2))
print(json.dumps(windows,ensure_ascii=False,indent=2))
