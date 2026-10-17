import csv,gzip,json
from pathlib import Path
D=Path('/home/yu/yuri_gazebo_performance/voice_batch_four'); R=D/'runs/four-20261017T081850-4d18c0'; out={}
rows=[r for r in csv.DictReader((R/'contacts.csv').open()) if 'bin_yellow' in r['collision1']+r['collision2']]
out['contact']={'points':len(rows),'first':rows[0],'last':rows[-1],'max':max(float(r['depth']) for r in rows),'depths_first_last':[float(rows[0]['depth']),float(rows[-1]['depth'])]}
near=[];controllers=[]
for l in gzip.open(R/'raw-ros.jsonl.gz','rt'):
    d=json.loads(l); m=d['message']; stamp=m.get('header',{}).get('stamp',{});t=stamp.get('sec',0)+stamp.get('nanosec',0)*1e-9
    if d['kind']=='joint_state' and 30.04<=t<=30.12:
        near.append({'stamp':t,'joints':{n:{'position':m['position'][i],'velocity':m['velocity'][i]} for i,n in enumerate(m['name']) if 'gripper' in n and not n.startswith('arm2_')}})
    if d['kind']=='controller_state' and 30.04<=t<=30.12 and 'arm2_' not in str(m.get('joint_names')):controllers.append(m)
out['max_tracking_error_rad_in_window']=max(abs(x) for m in controllers for x in m['error']['positions']);out['joints_at_contact']=near;out['controller_first_last']=controllers[::max(1,len(controllers)-1)]
logical=[]
for l in (R/'events.jsonl').open():
    d=json.loads(l)
    if d.get('kind')=='logical' and 29.9<=d.get('stamp',0)<=30.12:logical.append(d)
out['logical_near_contact']=logical
old=Path('/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/contacts.csv')
with old.open() as f:
    rd=csv.DictReader(f);out['historical_columns']=rd.fieldnames; matches=[]
    for r in rd:
        if 'bin_yellow' in str(r) and 'finger' in str(r):matches.append(r)
out['historical_yellow_finger_count']=len(matches);out['historical_yellow_finger_examples']=matches[:2]+matches[-2:]
if matches:
    depthkey=next((k for k in matches[0] if 'depth' in k),None)
    if depthkey:out['historical_yellow_max_depth']=max(float(r[depthkey]) for r in matches)
(D/'audit/contact-review/analysis.json').write_text(json.dumps(out,indent=2)); print(json.dumps({k:v for k,v in out.items() if k not in ('joints_at_contact','controller_first_last','logical_near_contact')},indent=2));print('joints',near[:1]); print('controllers',controllers[:1])
