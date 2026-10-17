"""Index every attempt, including stopped/failed runs. No simulation commands."""
import csv,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];rows=[];manifest=[]
for r in sorted((ROOT/'calibration_runs').glob('[ABC]-*')):
 summary=json.loads((r/'summary.json').read_text()) if (r/'summary.json').exists() else {}
 fault=json.loads((r/'guard-fault.json').read_text()) if (r/'guard-fault.json').exists() else None
 phases=[]
 if (r/'finger-response.jsonl').exists():
  for line in (r/'finger-response.jsonl').open():
   e=json.loads(line)
   if 'phase' in e:phases.append({k:v for k,v in e.items() if k not in ('rows','ack')})
 contacts=[]
 if (r/'contacts.csv').exists():contacts=list(csv.DictReader((r/'contacts.csv').open()))
 pause=[json.loads(l)['stats'] for l in (r/'stats.jsonl').open() if json.loads(l)['stats'].get('paused')] if (r/'stats.jsonl').exists() else []
 plans=json.loads((r/'arm-probe.json').read_text()) if (r/'arm-probe.json').exists() else []
 successes=[{'arm':p['arm'],'stage':p['stage'],'status':p['action']['status'],'error_code':p['action']['result']['error_code']} for p in plans if p.get('action')]
 rows.append(dict(run=r.name,outcome=summary.get('outcome'),fault=fault,phases=phases,actions=successes,contact_points=len(contacts),collision_pairs=sorted(set((c['collision1'],c['collision2']) for c in contacts)),max_recorded_depth_m=max([float(c['depth']) for c in contacts if c['depth']] or [0]),observer_exit=summary.get('observer_exit'),pause_ack=summary.get('stop',[{}])[0].get('acknowledged'),pause_duration_s=summary.get('stop',[{}])[0].get('duration_monotonic_s'),paused_stats=len(pause)))
 for f in sorted(r.rglob('*')):
  if not f.is_file() or 'home' in f.relative_to(r).parts or 'ros' in f.relative_to(r).parts or 'replay-ros' in f.relative_to(r).parts:continue
  h=hashlib.sha256()
  with f.open('rb') as stream:
   for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
  manifest.append(h.hexdigest()+'  '+str(f.relative_to(ROOT)))
(ROOT/'calibration_runs/RUN_INDEX.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
(ROOT/'calibration_runs/MANIFEST_SHA256.txt').write_text('\n'.join(manifest)+'\n')
for r in rows:print(r['run'],r['outcome'],(r['fault'] or {}).get('reason'),len(r['actions']),r['contact_points'])
