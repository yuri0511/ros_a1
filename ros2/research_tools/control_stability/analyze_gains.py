"""Saved native ECM / ROS evidence analysis; never starts simulation."""
from pathlib import Path
import csv,json,collections,math,sys,gzip
import numpy as np
R=Path('/home/yu/yuri_gazebo_performance/control_stability');summary=[];details=[]
for C in sorted(list((R/'runs').glob('*empty-gain*'))+list((R/'runs').glob('*commandprobe-gain*'))):
 I=C/'instance-1'
 if not (C/'summary.json').exists():continue
 meta=json.loads((C/'summary.json').read_text());windows=[]
 for arm in ('arm1','arm2'):
  p=I/(arm+'-result.json')
  if not p.exists():continue
  obj=json.loads(p.read_text());begins={};last_trajectory=None
  config=next(x for x in obj['records'] if x['kind']=='configuration');names=config['joint_names']
  for x in obj['records']:
   if x['kind']=='planned_trajectory':last_trajectory=x['trajectory']
   if x['kind']=='window_begin':
    begins[x['label']]=dict(x,jtc_target=dict(zip(last_trajectory['joint_names'],last_trajectory['points'][-1]['positions'])) if last_trajectory else {})
   if x['kind']=='window_end':
    b=begins[x['label']];windows.append({'arm':arm,'label':x['label'],'start':b['sim'],'end':b['sim']+2.,'target':dict(zip(names,b['target'])),'jtc_target':b['jtc_target'],'gate':x['evidence'],'controller':x['controller'],'native':collections.defaultdict(list)})
 p=I/'native-joints.csv'
 if p.exists():
  with p.open() as f:
   for row in csv.DictReader(f):
    t=float(row['sim_sec']);name=row['joint']
    for w in windows:
     if name in w['target'] and w['start']+1<=t<=w['end']:
      w['native'][name].append((int(row['iteration']),t,float(row['q']),float(row['v']),float(row['velocity_cmd_pre']) if row.get('velocity_cmd_pre') else float('nan')))
 for w in windows:
  for name,rows in w.pop('native').items():
   a=np.array(rows);it=a[:,0].astype(int);t,q,v,cmd=a[:,1],a[:,2],a[:,3],a[:,4];target=w['target'][name];jtc_target=w['jtc_target'].get(name,target);dt=float(np.median(np.diff(t))) if len(t)>1 else None
   freq=None;phase_deg=None
   if len(t)>10 and np.std(v)>1e-8:
    vf=np.fft.rfft(v-np.mean(v));qf=np.fft.rfft(q-np.mean(q));index=np.argmax(abs(vf)[1:])+1;freq=float(np.fft.rfftfreq(len(v),dt)[index]);phase_deg=float(np.angle(vf[index]/qf[index],deg=True)) if abs(qf[index])>1e-12 else None
   fits=[];lookup=dict(zip(it,q))
   for parity in (0,1):
    obs=[];pred=[]
    for index,command in zip(it,cmd):
     prev=index-1 if (index-1)%2==parity else index-2
     if prev in lookup:obs.append(command);pred.append(meta['gain']*500*(jtc_target-lookup[prev]))
    if obs and np.all(np.isfinite(obs)):fits.append({'read_iteration_parity':parity,'command_rmse':float(np.sqrt(np.mean((np.array(obs)-pred)**2)))})
   best=min(fits,key=lambda x:x['command_rmse']) if fits else None
   row={'case':C.name,'gain':meta['gain'],'arm':w['arm'],'phase':w['label'],'joint':name,'samples':len(t),'window_start_sim':w['start']+1,'window_end_sim':w['end'],'native_dt_median':dt,'max_position_error_rad':float(np.max(abs(q-target))),'velocity_peak_rad_s':float(np.max(abs(v))),'velocity_rms_rad_s':float(np.sqrt(np.mean(v*v))),'velocity_cmd_peak_rad_s':float(np.max(abs(cmd))) if np.all(np.isfinite(cmd)) else None,'dominant_velocity_hz':freq,'velocity_vs_position_phase_deg':phase_deg,'jtc_target':jtc_target,'velocity_command_correlation':float(np.corrcoef(v,cmd)[0,1]) if np.all(np.isfinite(cmd)) and np.std(v)>1e-8 and np.std(cmd)>1e-8 else None,'best_sample_hold_fit':best,'possible_sample_hold_phase_fits':fits,'gate_ready':w['gate']['ready'],'action_success':bool(w['controller'].get('success')),'command_observation':'PreUpdate priority100' if np.all(np.isfinite(cmd)) else 'unavailable (PostUpdate-cleared field not a command measurement)'};summary.append(row)
  details.append({'case':C.name,'gain':meta['gain'],**w})
# Every case, including incomplete startup attempts, remains visible.
cases=[{'case':c.name,**json.loads((c/'summary.json').read_text())} for c in sorted(list((R/'runs').glob('*empty-gain*'))+list((R/'runs').glob('*commandprobe-gain*'))) if (c/'summary.json').exists()]
(R/'audit/native-response-analysis.json').write_text(json.dumps({'metric_window':'last one sim second of fixed two-sim-second post-Action observation','fft_noise_floor_analysis_only':1e-8,'case_inventory':cases,'windows':details,'joint_metrics':summary},indent=2))
cols=['case','gain','arm','phase','joint','samples','window_start_sim','window_end_sim','native_dt_median','max_position_error_rad','velocity_peak_rad_s','velocity_rms_rad_s','velocity_cmd_peak_rad_s','dominant_velocity_hz','velocity_command_correlation','gate_ready','action_success','command_observation']
with (R/'GAIN_COMPARISON.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=cols,extrasaction='ignore');writer.writeheader();writer.writerows(summary)
for row in summary:
 if row['phase']=='return_standby' and 'wrist_3' in row['joint']:print(json.dumps(row))
