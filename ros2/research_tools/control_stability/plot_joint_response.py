"""Publication-style static diagnostic, from saved native samples only."""
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/home/yu/yuri_gazebo_performance/control_stability');a=json.loads((R/'audit/native-response-analysis.json').read_text());cases=['20261017T030406-empty-gain2','20261017T030503-empty-gain1','20261017T030615-empty-gain0p5','20261017T030953-commandprobe-gain2'];fig,axes=plt.subplots(2,2,figsize=(11,6.8),layout='constrained')
for ax,case in zip(axes.flat,cases):
 phase='empty_carry' if 'commandprobe' in case else 'return_standby';w=next(x for x in a['windows'] if x['case']==case and x['arm']=='arm1' and x['label']==phase);start=w['start']+1.;target=w['target']['wrist_3_joint'];rows=[]
 with (R/'runs'/case/'instance-1/native-joints.csv').open() as f:
  for x in csv.DictReader(f):
   t=float(x['sim_sec'])
   if x['joint']=='wrist_3_joint' and start<=t<=start+.04:rows.append((t-start,float(x['q']),float(x['v']),float(x['velocity_cmd_pre']) if x.get('velocity_cmd_pre') else np.nan))
 data=np.array(rows);ax.plot(data[:,0]*1000,data[:,2],'o-',ms=2.5,lw=1,label='Native velocity')
 if np.all(np.isfinite(data[:,3])):ax.plot(data[:,0]*1000,data[:,3],'x--',ms=4,lw=1,label='Pre-physics velocity command')
 ax.axhline(.3,color='red',ls=':',lw=.8,label='READY speed threshold');ax.axhline(-.3,color='red',ls=':',lw=.8);ax.set_title('gain '+str(w['gain'])+' / '+phase.replace('_',' '));ax.set_xlabel('Time within saved window (ms)');ax.set_ylabel('rad/s');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='upper right')
fig.suptitle('Gazebo native wrist-3 response; unchanged 1 ms physics / 500 Hz control',fontsize=12)
fig.savefig(R/'JOINT_RESPONSE.png',dpi=180);fig.savefig(R/'JOINT_RESPONSE.pdf');print('Wrote native-response figure')
