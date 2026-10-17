import json,hashlib,subprocess,shutil,csv,statistics,difflib,zipfile,datetime,shlex
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');W=R/'worktree';OLD=R.parent/'fast_sim_8da998b';OUT=Path('/home/yu/Documents/Codex/2026-10-16/files-pasted-by-the-user-ubuntu/outputs/integration_72ab6f2')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for c in iter(lambda:f.read(1048576),b''):h.update(c)
 return h.hexdigest()
def git(*args,cwd=W):return subprocess.run(['git',*args],cwd=cwd,capture_output=True,text=True,check=True).stdout.strip()
assert (R/'POLICY_EXPERIMENTS_DONE').exists(),'Experiment coordinator is not finished'
commands=[];hashes=[]
for c in sorted((R/'runs').iterdir()):
 if not (c/'summary.json').exists():continue
 for tool in ('analyze.py','extra_analysis.py','policy_analysis.py'):subprocess.run(['python3',str(R/'tools'/tool),str(c)],check=True)
 m=json.loads((c/'metadata.json').read_text());p=json.loads((c/'policy-analysis.json').read_text())[0]
 if m['feed_policy']=='q':assert p['matches_new_training_hashes'] and not p['q_audit_inconsistencies']
 manifest={'base_commit':m['commit'],'core_source_sha256':m['source_hashes'],'physics_ms':m['step_ms'],'fast_waits':m['fast_waits'],'policy':m['feed_policy'],'contact_mode':m['contact_mode'],'lp_threads':m['threads'],'random_seed':'Not explicitly overridden; OS scheduling nondeterministic','world_sha256':sha(W/'ros2/src/a1_cell_sim/worlds/fast_sim_1ms.sdf'),'controller_sha256':sha(W/'ros2/src/a1_cell_sim/config/ur5e_cell_controllers.yaml'),'pipeline_yaml_sha256':sha(W/'ros2/src/a1_sorting_core/config/sorting_params_pipeline.yaml'),'q_load_audit':p['q_loads']}
 manifest['configuration_sha256']=hashlib.sha256(json.dumps(manifest,sort_keys=True).encode()).hexdigest();(c/'configuration-manifest.json').write_text(json.dumps(manifest,indent=2));hashes.append(m['source_hashes'])
 for ri in m['runs']:commands+=['# '+c.name,'env '+' '.join(shlex.quote(k+'='+v) for k,v in ri['env'].items())+' '+shlex.join(ri['command'])]
assert len(set(hashes))==1,'Unrecorded source version change during paired experiment'
(R/'RUN_COMMANDS.txt').write_text('\n\n'.join(commands)+'\n\n# Model/train commands: python3 tools/train_audit.py; python3 tools/dp_baseline.py\n# Manual: ./run_policy.sh rule 1; ./run_policy.sh q 1\n# Historical migrate.py is provenance, do not rerun against the already-patched tree.\n')
protected=json.loads((R/'audit/protected-8da-inventory.json').read_text());bad=[p for p,h in protected.items() if not (OLD/p).exists() or sha(OLD/p)!=h]
original=subprocess.run(['sha256sum','-c',str(R.parent/'original-tracked-files.sha256')],cwd='/home/yu/ros_a1',capture_output=True,text=True);(R/'audit/original-sha-final.log').write_text(original.stdout+original.stderr)
oldstatus=git('status','--porcelain',cwd=OLD/'worktree');originalstatus=git('status','--porcelain',cwd=Path('/home/yu/ros_a1'))
keep=['ros2/codex_rl','ros2/src/a1_sorting_core/a1_sorting_core/q_feed.py','ros2/src/a1_sorting_core/config/sorting_params_pipeline.yaml','ros2/measure','ros2/pixi.lock'];unchanged={k:not git('diff','--',k) for k in keep}
live=[]
for pr in Path('/proc').iterdir():
 if not pr.name.isdigit():continue
 try:argv=(pr/'cmdline').read_bytes().split(b'\0');arg0=argv[0].decode(errors='replace');names=[Path(a.decode(errors='replace')).name for a in argv[:3]]
 except OSError:continue
 if arg0.startswith('gz sim ') or any(n in ('move_group','rviz2','parameter_bridge','task_dispatcher_node','pick_place_executor_node','probe.py','record_contacts.py','run_case.py') for n in names):live.append({'pid':pr.name,'argv':[a.decode(errors='replace') for a in argv]})
integrity={'time':datetime.datetime.now().isoformat(),'old_8da_inventory_count':len(protected),'old_8da_changed_files':bad,'old_worktree_status':oldstatus,'original_status':originalstatus,'original_135_sha_ok':original.returncode==0,'original_commit':git('rev-parse','HEAD',cwd=Path('/home/yu/ros_a1')),'current_commit':git('rev-parse','HEAD'),'current_branch':git('branch','--show-current'),'upstream_untouched':unchanged,'remaining_application_processes':live}
(R/'audit/final-integrity.json').write_text(json.dumps(integrity,indent=2));assert not bad and original.returncode==0 and not originalstatus and all(unchanged.values()) and not live,integrity
patch=subprocess.run(['git','diff','--binary'],cwd=W,capture_output=True,text=True,check=True).stdout
for rel in git('ls-files','--others','--exclude-standard').splitlines():
 if '__pycache__' in rel:continue
 p=W/rel
 if p.is_file():patch+=''.join(difflib.unified_diff([],p.read_text().splitlines(True),fromfile='/dev/null',tofile=rel))
(R/'SOURCE_CHANGES.diff').write_text(patch)
subprocess.run(['python3',str(R/'tools/reports.py')],check=True)
OUT.mkdir(parents=True,exist_ok=True)
reports=['INTEGRATION_REPORT.md','TRAINING_AUDIT.md','DP_BASELINE_REPORT.md','GAZEBO_POLICY_COMPARISON.md','DATA_COMPATIBILITY.md','GAZEBO_RESULTS.csv','SOURCE_CHANGES.diff','RUN_COMMANDS.txt']
for name in reports:shutil.copy2(R/name,OUT/name)
with zipfile.ZipFile(OUT/'EVIDENCE.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=4) as z:
 for top in ('audit','training','runs','tools'):
  for p in sorted((R/top).rglob('*')):
   if not p.is_file() or p.is_symlink() or '__pycache__' in p.parts:continue
   if p.name=='contacts.csv' and Path(str(p)+'.gz').exists():continue
   z.write(p,str(p.relative_to(R)))
 for top in ('src','codex_rl','measure','docs'):
  for p in sorted((W/'ros2'/top).rglob('*')):
   if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts:z.write(p,'experimental_source/ros2/'+str(p.relative_to(W/'ros2')))
 for f in reports+['activate.sh','run_policy.sh']:z.write(R/f,f)
assert zipfile.ZipFile(OUT/'EVIDENCE.zip').testzip() is None
(OUT/'DELIVERY.json').write_text(json.dumps({'root':str(R),'evidence_zip_sha256':sha(OUT/'EVIDENCE.zip'),'reports':reports,'raw_contacts':'Original CSVs preserved at root; lossless gzip in ZIP'},indent=2))
print('FINAL DELIVERED',OUT)
