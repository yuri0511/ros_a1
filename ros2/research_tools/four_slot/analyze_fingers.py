import gzip,json,sys
from pathlib import Path

def main():
 r=Path(sys.argv[1]);events=[json.loads(l) for l in (r/'finger-response.jsonl').open()];rows=[]
 begin=events[0]['wall'];end=events[-1]['wall']
 for l in gzip.open(r/'raw-ros.jsonl.gz','rt'):
  d=json.loads(l)
  if d['kind']=='joint_state' and begin-.1<=d['wall']<=end+1:
   m=d['message'];q=dict(zip(m['name'],m['position']));v=dict(zip(m['name'],m['velocity']));rows.append(dict(wall=d['wall'],stamp=m['header']['stamp'],q={k:x for k,x in q.items() if 'finger' in k},v={k:x for k,x in v.items() if 'finger' in k}))
 (r/'finger-feedback-window.json').write_text(json.dumps(rows,indent=2))
 out={'events':events,'samples':len(rows),'joint_ranges':{k:{'min_m':min(x['q'][k] for x in rows),'max_m':max(x['q'][k] for x in rows),'peak_velocity_m_s':max(abs(x['v'][k]) for x in rows)} for k in rows[0]['q']} if rows else {}}
 (r/'finger-analysis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
if __name__=='__main__':main()
