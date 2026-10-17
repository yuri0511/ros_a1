import os,time,json,subprocess,gzip,shutil,difflib
from pathlib import Path
root=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');ws=root/'worktree/ros2'
prior=Path(json.loads((root/'audit/observer-cleanup-transition.json').read_text())['prior_low_repeat'])
while not (prior/'summary.json').exists():time.sleep(3)
for name in ('analyze.py','extra_analysis.py'):subprocess.run(['python3',str(root/'tools'/name),str(prior)],check=True)
p=ws/'src/a1_sorting_core/a1_sorting_core/task_dispatcher_node.py';old=p.read_text()
needle="""        self._ready_image = self.create_subscription(
            Image, '/overhead_camera',
            lambda m: setattr(self, '_startup_rgb_wall', time.monotonic()),
            qos_profile_sensor_data)"""
replacement="""        self._startup_readiness_complete = False
        def observe_startup_rgb(message):
            self._startup_rgb_wall = time.monotonic()
            if self._startup_readiness_complete and self._ready_image is not None:
                subscription = self._ready_image
                self._ready_image = None
                # Destroy our temporary observer inside its own executor callback.
                # Gazebo, the original vision node and the performance probe remain.
                self.destroy_subscription(subscription)
                self.get_logger().info('STARTUP_OBSERVER_RELEASED: temporary RGB readiness subscriber only')
        self._ready_image = self.create_subscription(
            Image, '/overhead_camera', observe_startup_rgb,
            qos_profile_sensor_data)"""
assert needle in old;s=old.replace(needle,replacement,1)
needle2='                    self._request_spawn_and_start_belt()\n                    return'
assert needle2 in s;s=s.replace(needle2,'                    self._startup_readiness_complete = True\n'+needle2,1)
compile(s,str(p),'exec');p.write_text(s)
(root/'audit/observer-cleanup-only.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='v1/task_dispatcher_node.py',tofile='v2/task_dispatcher_node.py')))
(root/'audit/implementation-v2.diff').write_text(subprocess.run(['git','diff','--binary'],cwd=root/'worktree',capture_output=True,text=True).stdout)
with (root/'CHANGELOG.md').open('a') as f:f.write('\n## v2：启动观察订阅清理\n\n初始矩阵与接触组均为v1。后续v2在就绪后、下一个RGB回调内销毁仅用于启动检查的临时订阅，保留Gazebo RGB传感器、原视觉节点、性能探针和所有判据。没有改变等待算法或物理设置。v1数据全部保留，v2单独两轮低干扰复测、一轮完整接触验证，再开展并行测试；以source_hash区分，不跨版本混算吞吐量。\n')
cmd=f"source {root}/activate.sh; source {ws}/install/setup.bash; python -m pytest src/a1_sorting_core/test src/a1_vision/test -q -k 'not real_table_initial and not unknown_state_uses and not wait_is_kept and not busy_arm_color and not wired_anchors_match and not anchor_table_gates and not world_image_camera_h_fov_final and not relay_section_identical' --junitxml={root}/audit/regression-v2.xml"
with (root/'audit/regression-v2.log').open('w') as f:subprocess.run(['bash','-c',cmd],cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=True)
print('V2 OBSERVER CLEANUP AND REGRESSION PASSED',flush=True)
def run(label,n,contact=False):
 env=os.environ.copy()
 for k in ('PERF_CONTACT','PERF_FIXTURE','PERF_ISOLATION_ONLY','PERF_EGL','PERF_RMW','FASTDDS_BUILTIN_TRANSPORTS','LP_NUM_THREADS'):env.pop(k,None)
 env['PERF_LP_THREADS']='2'
 if contact:env['PERF_CONTACT']='1'
 print('BEGIN',label,flush=True)
 subprocess.run(['bash','-c',f'source {root}/activate.sh; exec python {root}/tools/run_case.py {label} 1 true {n} {900 if n>1 or contact else 650}'],env=env,check=True)
 case=Path((root/'latest-case.txt').read_text())
 for name in ('analyze.py','extra_analysis.py'):subprocess.run(['python3',str(root/'tools'/name),str(case)],check=True)
 a=json.loads((case/'analysis.json').read_text());a['extra']=json.loads((case/'extra-analysis.json').read_text())
 if contact:
  r=case/'instance-1';src=r/'contacts.csv';e=a['extra'][0]
  subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(src),str(r/'contacts-all-summary.json')],check=True)
  if e['batch_first_picking_sim'] is not None and e['batch_last_terminal_sim'] is not None:
   subprocess.run(['python3',str(root/'tools/contact_summary.py'),str(src),str(r/'contacts-task-summary.json'),str(e['batch_first_picking_sim']),str(e['batch_last_terminal_sim'])],check=True)
  with src.open('rb') as f,gzip.open(str(src)+'.gz','wb',compresslevel=4) as z:shutil.copyfileobj(f,z)
 print('END',label,flush=True);return a

def good(a):return a['reason']=='completed' and a['successful_batches']==a['n'] and not a['swapout_delta'] and not a['oom_delta'] and not any(i['interlock_violations'] or i['vision_mismatches'] or i['isolation']['ros_foreign'] or i['isolation']['gz_foreign'] for i in a['instances'])
one=run('B2-1ms-event-repeat1',1);assert good(one),'v2 single repeat1 failed'
two=run('B2-1ms-event-repeat2',1);assert good(two),'v2 single repeat2 failed'
contact=run('B2-contact-1ms-event',1,True);assert good(contact),'v2 contact task failed'
co=json.loads((Path(contact['case'])/'instance-1/contacts-task-summary.json').read_text());assert co['bad_rows']==0 and not co['categories'].get('interarm'),'v2 contact safety gate failed'
low_rate=7200/(one['experiment_wall_s']+two['experiment_wall_s'])
dual=run('P2-v2-1ms-event',2);parallel=[]
if good(dual):parallel.append(dual)
eligible=good(dual) and dual['episodes_per_hour']>low_rate and dual['cpu_mean_percent']<85 and dual['cpu_p95_percent']<90 and not any(e['controller_abort_count'] for e in dual['extra'])
(root/'three-instance-gate.json').write_text(json.dumps({'eligible':eligible,'dual_case':dual['case'],'dual_ep_h':dual['episodes_per_hour'],'same_version_single_pooled_ep_h':low_rate,'dual_cpu_mean':dual['cpu_mean_percent'],'dual_cpu_p95':dual['cpu_p95_percent']},indent=2))
if eligible:
 triple=run('P3-v2-1ms-event',3)
 if good(triple):parallel.append(triple)
else:print('THREE SKIPPED: measured resource/reliability/throughput gate not met',flush=True)
if parallel:
 best=max(parallel,key=lambda a:a['episodes_per_hour']);repeat=run(f'P{best["n"]}-v2-repeat-1ms-event',best['n'])
 chosen={'chosen':best['case'],'repeat':repeat['case'],'repeat_valid':good(repeat)}
 if not good(repeat) and best['n']==3 and good(dual):
  fallback=run('P2-v2-repeat-1ms-event',2);chosen['fallback_repeat']=fallback['case'];chosen['fallback_valid']=good(fallback)
 (root/'parallel-repeat-result.json').write_text(json.dumps(chosen,indent=2))
(root/'FINAL_EXPERIMENTS_DONE').touch();print('ALL V2 FINAL EXPERIMENTS FINISHED',flush=True)
