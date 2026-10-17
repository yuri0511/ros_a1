from pathlib import Path
import subprocess,tempfile,shutil,json
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');W=R/'worktree';OLD=R.parent/'fast_sim_8da998b';changes=[]
files=['ros2/src/a1_cell_sim/launch/cell_bringup.launch.py','ros2/src/a1_sorting_core/a1_sorting_core/motion_client.py','ros2/src/a1_sorting_core/a1_sorting_core/pick_place_executor_node.py','ros2/src/a1_sorting_core/a1_sorting_core/task_dispatcher_node.py','ros2/src/a1_sorting_core/launch/sorting_demo.launch.py']
for rel in files:
 base=subprocess.run(['git','show','8da998b:'+rel],cwd=W,capture_output=True,check=True).stdout
 with tempfile.TemporaryDirectory() as d:
  b=Path(d)/'base';b.write_bytes(base)
  result=subprocess.run(['git','merge-file','-p',str(W/rel),str(b),str(OLD/'worktree'/rel)],capture_output=True)
  text=result.stdout.decode()
  if result.returncode==1:
   # Only old log addition overlaps Yuri's new, retained relative loader.
   while '<<<<<<< ' in text:
    start=text.index('<<<<<<< ');mid=text.index('=======\n',start);end=text.index('>>>>>>> ',mid);endline=text.index('\n',end)
    ours=text[text.index('\n',start)+1:mid]
    (R/'audit'/(Path(rel).name+'.merge-conflict.txt')).write_text(text[start:endline+1])
    text=text[:start]+ours+text[endline+1:]
  elif result.returncode!=0:raise RuntimeError(result.stderr.decode())
  compile(text,rel,'exec');(W/rel).write_text(text);changes.append(rel)
for rel in ['ros2/src/a1_cell_sim/worlds/fast_sim_1ms.sdf','ros2/src/a1_sorting_core/test/test_fast_sim_waits.py']:
 shutil.copy2(OLD/'worktree'/rel,W/rel);changes.append(rel)
# Explicit external parameter overlay, after Yuri's YAML; upstream YAML stays intact.
p=W/'ros2/src/a1_sorting_core/launch/sorting_demo.launch.py';s=p.read_text()
s=s.replace("    feed_policy = LaunchConfiguration('feed_policy')", "    feed_policy = LaunchConfiguration('feed_policy')\n    policy_params = LaunchConfiguration('policy_params').perform(context)")
s=s.replace("                     **q_overrides}])", "                     **q_overrides}] + ([policy_params] if policy_params else []))")
needle="        DeclareLaunchArgument(\n            'feed_policy', default_value='q',"
s=s.replace(needle,"        DeclareLaunchArgument('policy_params', default_value='',\n                              description='Optional local experiment ROS parameter overlay'),\n"+needle)
compile(s,str(p),'exec');p.write_text(s)
(R/'audit/migration-files.json').write_text(json.dumps(changes,indent=2))
# Activation only relocates experimental root; the pinned shared runtime is retained.
(R/'activate.sh').write_text((OLD/'activate.sh').read_text().replace(str(OLD),str(R)))
for name in ['probe.py','latency_probe.py','record_contacts.py','analyze.py','extra_analysis.py','contact_summary.py','positions.py','run_case.py','instance_control.py']:
 s=(OLD/'tools'/name).read_text().replace(str(OLD),str(R)).replace('8da998b963eed3c70fda076329f04a75c8627ef3','72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd');(R/'tools'/name).write_text(s)
shutil.copytree(OLD/'tools/bin',R/'tools/bin',dirs_exist_ok=True)
p=R/'tools/run_case.py';s=p.read_text();s=s.replace("if step not in ('1','2')", "if step != '1'")
s=s.replace("inner=f'source {ROOT}/activate.sh;", "feed_policy=os.environ.get('PERF_POLICY','rule');policy_params=os.environ.get('PERF_POLICY_PARAMS','')\n if feed_policy not in ('rule','q'):raise ValueError('Explicit rule/q required')\n inner=f'source {ROOT}/activate.sh;",1)
s=s.replace("fast_sim_waits:={fast} result_csv:={run}/results.csv'", "fast_sim_waits:={fast} result_csv:={run}/results.csv feed_policy:={feed_policy}'\n if policy_params:inner+=' policy_params:='+policy_params")
s=s.replace("meta['fixture']=fixture", "meta['fixture']=fixture\nmeta['feed_policy']=os.environ.get('PERF_POLICY','rule')\nmeta['policy_params']=os.environ.get('PERF_POLICY_PARAMS','')")
compile(s,str(p),'exec');p.write_text(s)
print('Migrated by three-way source merge; preserved new loader/YAML/default q')
