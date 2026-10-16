import difflib,subprocess
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');p=R/'worktree/ros2/src/a1_sorting_core/a1_sorting_core/task_dispatcher_node.py';old=p.read_text();s=old
s=s.replace('        from controller_manager_msgs.srv import ListControllers\n','        import subprocess\n        from controller_manager_msgs.srv import ListControllers\n',1)
a=s.index('            active = set(); future = None; last_sim = 0.0; checks = {}',s.index('    def _start_ready_feed'))
b=s.index('                checks = {',a)
s=s[:a]+'''            active = set(); future = None; last_sim = 0.0; checks = {}
            query_started = 0.0; response_at = 0.0; next_query = 0.0
            transport_at = 0.0; next_cli = 0.0; next_diagnostic = 0.0
            query_timeouts = 0; query_error = ''; transport_error = ''
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic(); sim = self.get_clock().now().nanoseconds / 1e9
                if future is not None and future.done():
                    try:
                        active = {c.name for c in future.result().controller if c.state == 'active'}
                        response_at = now; query_error = ''
                    except Exception as exc:
                        active = set(); query_error = str(exc)
                    future = None; next_query = now + .5
                if future is not None and now - query_started >= 2.0:
                    # One lost RPC must not occupy the whole startup deadline.
                    self._ready_control.remove_pending_request(future)
                    future.cancel(); future = None
                    query_timeouts += 1; next_query = now + .1
                if future is None and now >= next_query and self._ready_control.service_is_ready():
                    future = self._ready_control.call_async(ListControllers.Request())
                    query_started = now
                try:
                    _, subscribers = self._ready_gz.topic_info(self._belt_speed_topic)
                    if subscribers:
                        transport_at = time.monotonic(); transport_error = ''
                except Exception as exc:
                    transport_error = str(exc)
                if now - transport_at > 3.0 and now >= next_cli:
                    # Harmonic Python discovery can miss an existing subscriber.
                    # Independently require a real typed subscriber; never bypass.
                    remaining = deadline - time.monotonic()
                    if remaining > .1:
                        try:
                            result = subprocess.run(
                                ['gz', 'topic', '-i', '-t', self._belt_speed_topic],
                                capture_output=True, text=True,
                                timeout=min(2.0, remaining), check=False)
                            subscriber_section = result.stdout.partition('Subscribers [Address, Message Type]:')[2]
                            if result.returncode == 0 and any(
                                    'tcp://' in line and 'gz.msgs.Double' in line
                                    for line in subscriber_section.splitlines()):
                                transport_at = time.monotonic(); transport_error = ''
                            else:
                                transport_error = 'no typed subscriber in CLI discovery'
                        except (subprocess.TimeoutExpired, OSError) as exc:
                            transport_error = str(exc)
                    next_cli = time.monotonic() + 2.0
                now = time.monotonic()
                transport_ok = now - transport_at < 3.0
''' +s[b:]
s=s.replace("'controllers_active': required.issubset(active),","'controllers_active': required.issubset(active) and now-response_at < 3.0,",1)
needle='                last_sim = sim\n                time.sleep(.1)'
repl='''                if now >= next_diagnostic:
                    self.get_logger().info('STARTUP_CHECK '+json.dumps({
                        'checks': checks, 'active': sorted(active),
                        'controller_reply_age_s': now-response_at if response_at else None,
                        'controller_rpc_timeouts': query_timeouts,
                        'controller_error': query_error, 'transport_error': transport_error}))
                    next_diagnostic = now + 5.0
                last_sim = sim
                time.sleep(.1)'''
assert needle in s;s=s.replace(needle,repl,1)
compile(s,str(p),'exec');p.write_text(s)
(R/'audit/readiness-retry-only.diff').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='v2/task_dispatcher_node.py',tofile='v3/task_dispatcher_node.py')))
(R/'audit/implementation-v3.diff').write_text(subprocess.run(['git','diff','--binary'],cwd=R/'worktree',capture_output=True,text=True,check=True).stdout)
with (R/'CHANGELOG.md').open('a') as f:f.write('\n## v3：就绪查询健壮性修复\n\nv2精密接触轮在未送料时60秒超时，保留失败记录。控制器日志虽已激活，检查端仍缺RPC回复和GZ订阅发现证据。独立空载诊断复现Python topic_info交替漏报订阅者，CLI能发现。v3增加控制器RPC单次2秒超时/有界重发，0.5秒查询间隔，要求3秒内真实active回复；GZ查询缺订阅时以同分区gz topic -i有界2秒独立核验真实Double订阅者，证据有效期3秒。所有原10项条件及60秒总上限保留，不就绪仍禁止送料。增加5秒诊断。该修复不改变物理、Q、运动/抓取判据；v1/v2/v3分开统计。\n')
