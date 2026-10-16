import json,hashlib,shutil,subprocess,difflib,zipfile,csv,datetime
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';W=R/'worktree';OUT=Path('/home/yu/Documents/Codex/2026-10-16/files-pasted-by-the-user-ubuntu/outputs/model_calibration_v2')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def git(*args,cwd=W):return subprocess.run(['git',*args],cwd=cwd,capture_output=True,text=True,check=True).stdout.strip()
# Archive immutable input copies and exact source identifiers, never replace originals.
raw=R/'raw';mapping={}
for case in sorted((I/'runs').iterdir()):
 dest=raw/case.name/'instance-1';dest.mkdir(parents=True,exist_ok=True)
 for name in ('launch.log','stats.jsonl','phase-events.csv','results.csv','ros-probe.jsonl','contacts.csv.gz','contacts-task-summary.json','tracked-positions.json','policy-audit.json'):
  src=case/'instance-1'/name
  if src.exists():shutil.copy2(src,dest/name);mapping[str(src)]=str((dest/name).relative_to(R))
 for name in ('metadata.json','analysis.json','extra-analysis.json','policy-analysis.json','configuration-manifest.json'):
  if (case/name).exists():shutil.copy2(case/name,raw/case.name/name)
(R/'data/ARCHIVE_MAPPING.json').write_text(json.dumps(mapping,indent=2))
H=R/'training/history';H.mkdir(exist_ok=True)
for name,src in {'q-yuri-prefeed-true.json':I/'training/history/q_table.json','config-yuri.json':I/'training/history/timing.gazebo-v1.json','q-v1-prefeed-false-demo.json':I/'training/artifacts/q-demo-2222.json','config-v1-prefeed-false.json':I/'training/config/timing.prefeed-false.json'}.items():shutil.copy2(src,H/name)
# Complete code diff includes experimental drivers as well as the model.
shutil.copytree(R/'tools',W/'ros2/codex_rl_v2/experiment_tools',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
codehash={str(p.relative_to(W)):sha(p) for p in (W/'ros2/codex_rl_v2').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
(R/'training/SOURCE_CODE_HASHES.json').write_text(json.dumps(codehash,indent=2))
train=json.loads((R/'training/TRAINING_MANIFEST.json').read_text())
for x in train:
 assert sha(R/f'training/q-v2-{x["name"]}.json')==x['table_sha256']
 assert json.loads((R/f'training/q-v2-{x["name"]}.json').read_text())['deployment_ready'] is False
 x['model_source_sha256']=sha(W/'ros2/codex_rl_v2/dual_arm_rl/model.py');x['trainer_source_sha256']=sha(W/'ros2/codex_rl_v2/dual_arm_rl/train.py')
(R/'training/TRAINING_MANIFEST.json').write_text(json.dumps(train,indent=2))
protected=json.loads((R/'audit/protected-integration-files.json').read_text());changed=[p for p,h in protected.items() if not (I/p).exists() or sha(I/p)!=h]
old=R.parent/'fast_sim_8da998b';oldhash=json.loads((I/'audit/protected-8da-inventory.json').read_text());oldchanged=[p for p,h in oldhash.items() if not (old/p).exists() or sha(old/p)!=h]
original=subprocess.run(['sha256sum','-c',str(R.parent/'original-tracked-files.sha256')],cwd='/home/yu/ros_a1',capture_output=True,text=True);(R/'audit/original-sha-final.log').write_text(original.stdout+original.stderr)
live=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:args=(p/'cmdline').read_bytes().split(b'\0');s=args[0].decode(errors='replace')
 except OSError:continue
 if s.startswith('gz sim ') or Path(s).name in ('move_group','rviz2'):live.append({'pid':p.name,'command':s})
basefiles=git('ls-files').splitlines();inherited_changes=[p for p in basefiles if (W/p).is_file() and (I/'worktree'/p).is_file() and sha(W/p)!=sha(I/'worktree'/p)]
flags={}
for p in [R/'training/config.v2.json',R/'training/q-v2-demo.json',R/'training/q-v2-general.json']:
 d=json.loads(p.read_text());flags[p.name]={k:d.get(k) for k in ('schema_version','allow_prefeed_busy_arm','overlap_validated','deployment_ready')}
original_csv_ok=sha(Path('/home/yu/.ros/a1_sorting_results.csv'))=='9d1a577e9b0aa7086af45c07268eabbdfd84373c921351078fe77d4fa83d5799'
audit={'time':datetime.datetime.now().isoformat(),'integration_protected_count':len(protected),'integration_changed':changed,'8da_protected_count':len(oldhash),'8da_changed':oldchanged,'original_135_sha_ok':original.returncode==0,'original_csv_unchanged':original_csv_ok,'original_status':git('status','--porcelain',cwd=Path('/home/yu/ros_a1')),'head':git('rev-parse','HEAD'),'branch':git('branch','--show-current'),'new_tracked_source_vs_integration_differences':inherited_changes,'new_gazebo_runs':0,'remaining_sim_processes':live,'flags':flags}
(R/'audit/FINAL_INTEGRITY.json').write_text(json.dumps(audit,indent=2));assert not changed and not oldchanged and original.returncode==0 and original_csv_ok and not audit['original_status'] and not inherited_changes and not live,audit
patch=subprocess.run(['git','diff','--binary'],cwd=W,capture_output=True,text=True,check=True).stdout
for rel in git('ls-files','--others','--exclude-standard').splitlines():
 p=W/rel
 if p.is_file() and '__pycache__' not in p.parts:patch+=''.join(difflib.unified_diff([],p.read_text().splitlines(True),fromfile='/dev/null',tofile=rel))
(R/'SOURCE_CHANGES.diff').write_text(patch)
subprocess.run(['python3',str(R/'tools/write_reports.py')],check=True)
reports=['REAL_TRACE_ANALYSIS.md','MODEL_MISMATCH_REPORT.md','CALIBRATION_DATA.csv','MODEL_V2_REPORT.md','TRAINING_V2_REPORT.md','DP_V2_REPORT.md','NEXT_GAZEBO_TEST_PLAN.md','REPRO_COMMANDS.txt','SOURCE_CHANGES.diff']
OUT.mkdir(parents=True,exist_ok=True)
for name in reports:shutil.copy2(R/name,OUT/name)
with zipfile.ZipFile(OUT/'EVIDENCE.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=4) as z:
 for top in ('audit','data','training','raw','tools'):
  for p in sorted((R/top).rglob('*')):
   if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts:z.write(p,str(p.relative_to(R)))
 for p in sorted((W/'ros2/codex_rl_v2').rglob('*')):
  if p.is_file() and '__pycache__' not in p.parts:z.write(p,'model_source/'+str(p.relative_to(W/'ros2')))
 for name in reports:z.write(R/name,name)
assert zipfile.ZipFile(OUT/'EVIDENCE.zip').testzip() is None
(OUT/'DELIVERY.json').write_text(json.dumps({'source_root':str(R),'reports':reports,'evidence_zip_sha256':sha(OUT/'EVIDENCE.zip'),'original_inputs_unchanged':True,'gazebo_runs_this_stage':0},indent=2));print('DELIVERED',OUT)
