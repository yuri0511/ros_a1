"""Run after the experiment coordinator exits; preserve all raw evidence."""
import json,hashlib,csv,statistics,subprocess,shlex,difflib,zipfile,shutil,datetime,os
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');W=R/'worktree';O=Path('/home/yu/ros_a1');OUT=Path('/home/yu/Documents/Codex/2026-10-16/files-pasted-by-the-user-ubuntu/outputs/fast_sim_8da998b')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(*args,cwd=W):return subprocess.run(['git',*args],cwd=cwd,capture_output=True,text=True,check=True).stdout.strip()
def dump(p,data):p.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
assert (R/'FINAL_EXPERIMENTS_DONE').exists(),'Do not finalize while experiments remain active'
for case in sorted((R/'runs').iterdir()):
 if not (case/'summary.json').exists():continue
 for tool in ('analyze.py','extra_analysis.py'):subprocess.run(['python3',str(R/'tools'/tool),str(case)],check=True)
# Recompute derived contact summaries with measured stamp spans, never assumed Hz.
for case in sorted((R/'runs').iterdir()):
 for run in sorted(case.glob('instance-*')):
  src=run/'contacts.csv'
  if not src.exists():continue
  subprocess.run(['python3',str(R/'tools/contact_summary.py'),str(src),str(run/'contacts-all-summary.json')],check=True)
  extra=case/'extra-analysis.json'
  if extra.exists():
   ex=json.loads(extra.read_text())[int(run.name.split('-')[-1])-1]
   if ex['batch_first_picking_sim'] is not None and ex['batch_last_terminal_sim'] is not None:
    subprocess.run(['python3',str(R/'tools/contact_summary.py'),str(src),str(run/'contacts-task-summary.json'),str(ex['batch_first_picking_sim']),str(ex['batch_last_terminal_sim'])],check=True)
for summary in (R/'audit').glob('yuri-*-summary.json'):
 d=json.loads(summary.read_text());src=Path(d['source'])
 subprocess.run(['python3',str(R/'tools/contact_summary.py'),str(src),str(summary)],check=True)
current={str(p.relative_to(W)):sha(p) for p in (W/'ros2/src').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
dump(R/'audit/source-hashes-v3-final.json',current)
v1=json.loads((R/'audit/source-hashes-v1.json').read_text())
changed_since_v1={k:{'v1':v,'v3':current.get(k)} for k,v in v1.items() if current.get(k)!=v}
dump(R/'audit/changes-since-v1.json',changed_since_v1)
assert set(changed_since_v1)<= {'ros2/src/a1_sorting_core/a1_sorting_core/task_dispatcher_node.py'},changed_since_v1
config_paths=sorted(p for p in (W/'ros2/src').rglob('*') if p.is_file() and p.suffix in ('.yaml','.sdf','.urdf','.xacro','.srdf') and 'build' not in p.parts)
configs={str(p.relative_to(W)):sha(p) for p in config_paths}
assert all(v1.get(k)==v for k,v in configs.items()),'Historical configuration mismatch'
commands=['# Experimental drivers retained verbatim in tools/: matrix_stage.py, contact_stage.py, final_stage.py, observer_cleanup_stage.py, final_v3_stage.py; do not rerun historical edit drivers against the already-patched tree.','# Current reproducer: ~/yuri_gazebo_performance/fast_sim_8da998b/run_fast.sh [1|2|3]','# Regression command (see audit/regression-v3.log): source activate.sh; source worktree/ros2/install/setup.bash; python -m pytest src/a1_sorting_core/test src/a1_vision/test -q -k \"not real_table_initial and not unknown_state_uses and not wait_is_kept and not busy_arm_color and not wired_anchors_match and not anchor_table_gates and not world_image_camera_h_fov_final and not relay_section_identical\"']
for case in sorted((R/'runs').iterdir()):
 if not (case/'metadata.json').exists():continue
 m=json.loads((case/'metadata.json').read_text());version='v3' if m['label'].startswith('B3') or '-v3-' in m['label'] else ('v2' if m['label'].startswith('B2') or '-v2-' in m['label'] or m['label'].startswith('discovery') else 'v1')
 config={'provenance':'post-run reconstruction from immutable metadata and configuration files verified unchanged since v1','base_commit':m['commit'],'source_hash':m['source_hashes'],'source_version':version,'physics_ms':m['step_ms'],'fast_waits':m['fast_waits'],'rtf_target':m['rate'],'threads':m['threads'],'rmw':m.get('rmw'),'renderer':'X11/llvmpipe' if m.get('egl')=='0' else 'EGL','contact_mode':m['contact_mode'],'probe_mode':m.get('probe_mode'),'instances':m['instances'],'config_file_hashes':configs}
 config['configuration_hash']=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest();dump(case/'configuration-manifest.json',config)
 commands+=['# '+case.name]
 for i,ri in enumerate(m['runs'],1):commands+=['# instance '+str(i),'env '+ ' '.join(shlex.quote(k+'='+v) for k,v in ri['env'].items())+' '+shlex.join(ri['command'])]
 if not (case/'processes.csv').exists():continue
 a=json.loads((case/'analysis.json').read_text());end=m.get('start',0)+a['experiment_wall_s'];groups={}
 for row in csv.DictReader((case/'processes.csv').open()):
  if not m['start']+30<=float(row['wall'])<=end:continue
  key=(row['instance'],row['pid'],row['command']);g=groups.setdefault(key,{'cpu':[],'rss':[],'threads':[]})
  if row['cpu_one_core_pct']:g['cpu'].append(float(row['cpu_one_core_pct']))
  g['rss'].append(float(row['rss_mib']));g['threads'].append(int(row['threads']))
 stats=[]
 for (inst,pid,cmd),g in groups.items():stats.append({'instance':int(inst),'pid':int(pid),'command':cmd,'samples':len(g['rss']),'mean_cpu_one_core_percent':statistics.mean(g['cpu']) if g['cpu'] else None,'max_cpu_one_core_percent':max(g['cpu'],default=None),'mean_rss_mib':statistics.mean(g['rss']),'max_rss_mib':max(g['rss']),'max_threads':max(g['threads'])})
 dump(case/'process-summary.json',sorted(stats,key=lambda g:g['mean_cpu_one_core_percent'] or 0,reverse=True))
(R/'RUN_COMMANDS.txt').write_text('\n\n'.join(commands)+'\n')
# Preserve current local modifications and complete additive file diffs.
tracked=subprocess.run(['git','diff','--binary','--','ros2/src'],cwd=W,capture_output=True,text=True,check=True).stdout
extras=[]
for rel in git('ls-files','--others','--exclude-standard').splitlines():
 p=W/rel
 if p.is_file():extras.append(''.join(difflib.unified_diff([],p.read_text().splitlines(True),fromfile='/dev/null',tofile=rel)))
(R/'ALL_CHANGES.diff').write_text(tracked+''.join(extras))
py_diff=subprocess.run(['git','diff','--','ros2/src/**/*.py'],cwd=W,capture_output=True,text=True,check=True).stdout
(R/'PYTHON_CHANGES.diff').write_text(py_diff+''.join(x for x in extras if 'test_fast_sim_waits.py' in x))
base=(W/'ros2/src/a1_cell_sim/worlds/sorting_cell.sdf').read_text().splitlines(True)
(R/'SDF_CHANGES.diff').write_text(''.join(''.join(difflib.unified_diff(base,(W/f'ros2/src/a1_cell_sim/worlds/fast_sim_{step}ms.sdf').read_text().splitlines(True),fromfile='sorting_cell.sdf',tofile=f'fast_sim_{step}ms.sdf')) for step in (1,2)))
check=subprocess.run(['sha256sum','-c',str(R.parent/'original-tracked-files.sha256')],cwd=O,capture_output=True,text=True)
original_csv=Path('/home/yu/.ros/a1_sorting_results.csv');expected='9d1a577e9b0aa7086af45c07268eabbdfd84373c921351078fe77d4fa83d5799'
protected=['ros2/pixi.lock','ros2/src/a1_sorting_core/a1_sorting_core/q_feed.py','ros2/src/a1_cell_sim/worlds/sorting_cell.sdf']
unchanged={rel:hashlib.sha256(subprocess.run(['git','show','HEAD:'+rel],cwd=W,capture_output=True,check=True).stdout).hexdigest()==sha(W/rel) for rel in protected}
integrity={'time':datetime.datetime.now().isoformat(),'original_commit':git('rev-parse','HEAD',cwd=O),'original_git_status':git('status','--porcelain',cwd=O),'original_sha_files_ok':check.returncode==0,'original_sha_verified_count':check.stdout.count(': OK'),'original_csv_sha':sha(original_csv),'original_csv_unchanged':sha(original_csv)==expected,'original_csv_rows':sum(1 for _ in csv.DictReader(original_csv.open())),'new_worktree_commit':git('rev-parse','HEAD'),'new_worktree_branch':git('branch','--show-current'),'protected_upstream_files_identical':unchanged,'modified_tracked_files':git('diff','--name-only').splitlines()}
(R/'audit/original-integrity-final.log').write_text(check.stdout+check.stderr);dump(R/'audit/original-integrity-final.json',integrity)
assert integrity['original_sha_files_ok'] and integrity['original_csv_unchanged'] and not integrity['original_git_status'] and all(unchanged.values()),integrity
# Confirm all experiment application processes exited, without matching our own script source.
alive=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:args=(p/'cmdline').read_bytes().split(b'\0');exe=Path(args[0].decode()).name;cmd=b' '.join(args).decode(errors='replace')
 except (OSError,UnicodeError):continue
 if (args[0].decode(errors='replace').startswith('gz sim ') or exe in ('move_group','rviz2','parameter_bridge','pick_place_executor_node','task_dispatcher_node') or exe.startswith('python') and any(str(R/'tools'/t) in [x.decode(errors='replace') for x in args] for t in ('run_case.py','probe.py','record_contacts.py','latency_probe.py'))):alive.append({'pid':int(p.name),'command':cmd})
dump(R/'audit/process-cleanup-final.json',{'remaining_application_processes':alive});assert not alive,alive
subprocess.run(['python3',str(R/'tools/export_matrix.py')],check=True)
subprocess.run(['python3',str(R/'tools/write_final_report.py')],check=True)
OUT.mkdir(parents=True,exist_ok=True)
files=('REPORT.md','CHANGELOG.md','DATA_COMPATIBILITY.md','EXPERIMENT_MATRIX.csv','CONCLUSIONS.json','PYTHON_CHANGES.diff','SDF_CHANGES.diff','ALL_CHANGES.diff','RUN_COMMANDS.txt')
for f in files:shutil.copy2(R/f,OUT/f)
zip_path=OUT/'EVIDENCE.zip'
with zipfile.ZipFile(zip_path,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=4) as z:
 for top in ('audit','runs','tools'):
  for p in sorted((R/top).rglob('*')):
   if not p.is_file() or p.is_symlink() or '__pycache__' in p.parts:continue
   if p.name=='contacts.csv' and Path(str(p)+'.gz').exists():continue
   z.write(p,str(p.relative_to(R)))
 for p in sorted((W/'ros2').rglob('*')):
  rel=p.relative_to(W)
  if not p.is_file() or p.is_symlink() or any(s in rel.parts for s in ('build','install','log','__pycache__','.pixi','.pytest_cache','.git')):continue
  z.write(p,'experimental_source/'+str(rel))
 for f in files+('activate.sh','run_fast.sh'):z.write(R/f,f)
dump(OUT/'DELIVERY.json',{'source_root':str(R),'report_files':list(files),'evidence_zip_sha256':sha(zip_path),'evidence_zip_bytes':zip_path.stat().st_size,'raw_contacts':'All originals retained at source_root; compressed lossless copies included when present','zip_members':len(zipfile.ZipFile(zip_path).namelist())})
print('DELIVERED',OUT,flush=True)
