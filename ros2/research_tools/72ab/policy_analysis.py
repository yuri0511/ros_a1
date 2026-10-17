import json,re,sys,csv
from pathlib import Path
case=Path(sys.argv[1]);meta=json.loads((case/'metadata.json').read_text());analysis=json.loads((case/'analysis.json').read_text());out=[]
for ri,a in zip(meta['runs'],analysis['instances']):
 r=Path(ri['run']);text=(r/'launch.log').read_text();dec=[];loads=[]
 for line in text.splitlines():
  for tag,target in [('Q_DECISION',dec),('Q_AUDIT_LOAD',loads)]:
   if tag+' {' in line:target.append(json.loads(line.split(tag+' ',1)[1]))
 first=re.search(r'已请求 1/8\([^<]+<- (\w+)',text)
 actionlines=re.findall(r'按需来料: 已请求.*',text)
 known=sum(not d['stats'].get('unknown_state_fallback_steps',0) for d in dec);unknown=sum(d['stats'].get('unknown_state_fallback_steps',0) for d in dec)
 v={'policy':meta.get('feed_policy'),'first_color':first[1] if first else None,'q_loads':loads,'q_decisions':len(dec),'known_state_decisions':known,'unknown_state_fallbacks':unknown,'unknown_state_fallback_rate':unknown/len(dec) if dec else None,'wait_decisions':sum(d['action']==4 for d in dec),'illegal_decisions':[d for d in dec if d['action'] not in d['legal_actions']],'decision_audit':dec,'feed_request_log':actionlines,'rule_confirmed':'退回谁闲推谁: 没有填写 Q 表路径' in text,'q_confirmed':'已加载 Q 表:' in text,'q_missing_timestamp_fallback_lines':sum('Q表缺离区时刻' in l for l in text.splitlines()),'q_busy_rejected_lines':sum('Q表选了忙着的臂' in l for l in text.splitlines())}
 if meta.get('feed_policy')=='q' and loads:
  expected=json.loads((case.parents[1]/'training/training-manifest.json').read_text())[-1]
  v['matches_new_training_hashes']=(loads[0]['config_sha256']==expected['config_sha256'] and loads[0]['table_file_sha256']==expected['table_sha256'])
  table=json.loads(Path(loads[0]['table_path']).read_text());v['q_audit_inconsistencies']=[]
  for d in dec:
   row=table['q_values'].get(d['state_key']);unknown=d['stats'].get('unknown_state_fallback_steps',0)>0
   if (row is None)!=unknown or row is not None and abs(row[d['action']]-max(row[a] for a in d['legal_actions']))>1e-9:v['q_audit_inconsistencies'].append(d)
 (r/'policy-audit.json').write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n');out.append(v)
(case/'policy-analysis.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n');print(case.name,[(d['policy'],d['first_color'],d['q_decisions'],d['unknown_state_fallbacks'],d['wait_decisions']) for d in out])
