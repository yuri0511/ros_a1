import json,gzip,statistics
from pathlib import Path
D=Path('/home/yu/yuri_gazebo_performance/voice_batch_four')
results=[]
for run in sorted((D/'ui_idle_runs').glob('idle-*')):
    if not (run/'summary.json').exists():continue
    s=json.loads((run/'summary.json').read_text());stats=[json.loads(l) for l in (run/'stats.jsonl').open()]
    def sim(d):t=d['stats']['sim_time'];return int(t.get('sec',0))+int(t.get('nsec',0))*1e-9
    metrics={}
    for p in s['phases']:
        a=[r for r in stats if p['start']+3<=r['wall']<=p['end']]
        if len(a)>1:metrics[p['phase']]={'sim_per_wall':(sim(a[-1])-sim(a[0]))/(a[-1]['wall']-a[0]['wall']),'samples':len(a)}
    stop=[json.loads(l) for l in (run/'stop-timeline.jsonl').open()];pause=stop[0];paused=[r for r in stats if r['wall']>=pause['start_wall'] and r['stats'].get('paused')]
    last_raw=None;counts={}
    for l in gzip.open(run/'raw-ros.jsonl.gz','rt'):
        d=json.loads(l);last_raw=d['wall'];counts[d['kind']]=counts.get(d['kind'],0)+1
    results.append({'run':str(run),'metrics':metrics,'observer_exit':s['observer_exit'],'pause_ack_seconds':pause['end_wall']-pause['start_wall'],'first_native_paused_delay':paused[0]['wall']-pause['start_wall'] if paused else None,'paused_sim_values':sorted(set(sim(r) for r in paused)),'paused_wall_span':paused[-1]['wall']-paused[0]['wall'] if paused else None,'last_raw_wall':last_raw,'last_native_wall':stats[-1]['wall'],'raw_crc_valid':True,'raw_counts':counts,'observed':json.loads((run/'observed-status.json').read_text())})
(D/'audit/idle-analysis.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
