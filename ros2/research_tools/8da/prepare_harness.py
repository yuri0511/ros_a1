from pathlib import Path
import shutil
old=Path('/home/yu/yuri_gazebo_performance');root=old/'fast_sim_8da998b';ws=root/'worktree/ros2'
for name in ('probe.py','latency_probe.py','analyze.py'):
 s=(old/'tools'/name).read_text().replace("Path('/home/yu/yuri_gazebo_performance')",f"Path('{root}')")
 if name=='probe.py':s=s.replace("if counts[kind]%10==1:","if kind in ('task','arrival') or counts[kind]%10==1:").replace("state=m.state,arm=m.arm)","state=m.state,arm=m.arm,detail=m.detail,task_id=m.task_id)")
 if name=='analyze.py':
  s=s.replace("'target':meta['rate']", "'target':meta['rate'],'step_ms':meta['step_ms'],'fast_waits':meta['fast_waits'],'contact_mode':meta['contact_mode']")
  s=s.replace("* .001", "* (float(meta['step_ms'])/1000)") if False else s.replace(")*.001-(sec", ")*(float(meta['step_ms'])/1000)-(sec")
 (root/'tools'/name).write_text(s)
s=(old/'tools/run_case.py').read_text().replace("ROOT=Path('/home/yu/yuri_gazebo_performance'); WS=ROOT/'workspace/ros2'",f"ROOT=Path('{root}'); WS=ROOT/'worktree/ros2'")
s=s.replace("label,rate,num,limit=sys.argv[1:5];num=int(num);limit=float(limit)","label,step,fast,num,limit=sys.argv[1:6];rate='4';num=int(num);limit=float(limit)\nif step not in ('1','2') or fast not in ('true','false'):raise SystemExit('Only audited 1/2 ms and explicit wait policy accepted')\ncontact_mode=os.environ.get('PERF_CONTACT','0')\nfor proc in Path('/proc').iterdir():\n if not proc.name.isdigit():continue\n try:c=(proc/'cmdline').read_bytes().split(b'\\0')[0].decode(errors='replace')\n except OSError:continue\n if c.startswith('gz sim '):raise SystemExit('Existing Gazebo server/GUI: '+str(proc)+' '+c)")
s=s.replace("'rate':rate,'instances'", "'rate':rate,'step_ms':step,'fast_waits':fast,'contact_mode':contact_mode,'commit':'8da998b963eed3c70fda076329f04a75c8627ef3','instances'")
s=s.replace("str(61+i)","str(81+i)").replace("performance_rtf_{rate}.sdf", "fast_sim_{step}ms.sdf fast_sim_waits:={fast}")
s=s.replace("threads=os.environ.get('PERF_LP_THREADS','default')", "threads=os.environ.get('PERF_LP_THREADS','2')")
s=s.replace("p=subprocess.Popen(cmd,cwd=WS", "contact=None\n if contact_mode=='1':\n  cf=(run/'contact-recorder.log').open('w')\n  contact=subprocess.Popen(['bash','-c',f'source {ROOT}/activate.sh; exec python {ROOT}/tools/record_contacts.py {run}/contacts.csv'],cwd=WS,env=env,stdout=cf,stderr=subprocess.STDOUT,start_new_session=True)\n  deadline=time.monotonic()+10\n  while not (run/'CONTACT_READY').exists() and time.monotonic()<deadline and contact.poll() is None:time.sleep(.1)\n  if not (run/'CONTACT_READY').exists():raise RuntimeError('Contact recorder not ready before Gazebo')\n p=subprocess.Popen(cmd,cwd=WS",1)
s=s.replace("'probe_pid':probe.pid,", "'probe_pid':probe.pid,'contact_pid':contact.pid if contact else None,")
s=s.replace("(case/'metadata.json').write_text", "meta['source_hashes']=__import__('hashlib').sha256(b''.join(p.read_bytes() for p in sorted((WS/'src/a1_sorting_core/a1_sorting_core').glob('*.py')))).hexdigest()\n(case/'metadata.json').write_text",1)
s=s.replace("  if len(done)+len(failed)==num:reason='completed_with_failures' if failed else 'completed';break", "  if len(done)+len(failed)==num:reason='completed_with_failures' if failed else 'completed';break")
s=s.replace("finally:\n # Stop only", "finally:\n # Collect settled final positions after all measurement timestamps, then stop.\n for p,probe,run,env in children:\n  if (run/'results.csv').exists():\n   try:subprocess.run(['bash','-c',f'source {ROOT}/activate.sh; exec python {ROOT}/tools/positions.py {run}'],env=env,stdout=subprocess.DEVNULL,stderr=(run/'positions-errors.log').open('w'),timeout=5)\n   except subprocess.TimeoutExpired:pass\n # Stop only",1)
s=s.replace(" for f in files:f.close()", " for info in meta['runs']:\n  run=Path(info['run']);(run/'CONTACT_STOP').touch()\n  if info.get('contact_pid'):\n   try:os.killpg(info['contact_pid'],signal.SIGINT)\n   except ProcessLookupError:pass\n for f in files:f.close()")
(root/'tools/run_case.py').write_text(s)
s=(ws/'measure/record_contacts.py').read_text().replace("    while True:\n        time.sleep(10)","    (out.parent/'CONTACT_READY').touch()\n    while not (out.parent/'CONTACT_STOP').exists():\n        time.sleep(1)")
(root/'tools/record_contacts.py').write_text(s)
shutil.copy2(old/'activate.sh',root/'activate.sh');(root/'tools/bin').mkdir(exist_ok=True)
shutil.copy2(old/'tools/bin/ruby',root/'tools/bin/ruby')
for p in (root/'tools').glob('*.py'):compile(p.read_text(),str(p),'exec')
