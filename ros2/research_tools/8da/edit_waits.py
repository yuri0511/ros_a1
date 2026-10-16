from pathlib import Path
import ast
WS=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b/worktree/ros2')
core=WS/'src/a1_sorting_core/a1_sorting_core'
# Explicit switch, default false; preserve all Q policy code.
p=core/'task_dispatcher_node.py';s=p.read_text().replace('import threading','import json\nimport threading',1)
s=s.replace("        self.declare_parameter('auto_start', False)","        self.declare_parameter('fast_sim_waits', False)\n        self._fast_waits = bool(self.get_parameter('fast_sim_waits').value)\n        self._startup_logical_wall = 0.0\n        self._startup_pusher = None\n        self._startup_rgb_wall = 0.0\n        self._first_feed_t0 = time.monotonic()\n        self._first_feed_logged = False\n        self.declare_parameter('auto_start', False)",1)
s=s.replace("            if self._belt_first_cmd_delay > 0.0:","            if self._fast_waits:\n                self._start_ready_feed()\n            elif self._belt_first_cmd_delay > 0.0:",1)
s=s.replace('        self._cam_cache.update(\n',"        self._startup_logical_wall = time.monotonic()\n        self._startup_pusher = self._pusher_world_x(msg)\n        self._cam_cache.update(\n",1)
needle='    def _note_sim(self, stamp):'
method='''    def _start_ready_feed(self):
        """Wall-bounded readiness; no feed on timeout. Physical/scheduling gates remain."""
        from controller_manager_msgs.srv import ListControllers
        from ros_gz_interfaces.srv import SpawnEntity
        from sensor_msgs.msg import Image
        from gz.transport13 import Node as GzNode
        self._ready_control = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._ready_spawn = self.create_client(SpawnEntity, '/world/sorting_cell/create')
        self._ready_image = self.create_subscription(
            Image, '/overhead_camera',
            lambda m: setattr(self, '_startup_rgb_wall', time.monotonic()),
            __import__('rclpy.qos', fromlist=['qos_profile_sensor_data']).qos_profile_sensor_data)
        self._ready_gz = GzNode()
        def run():
            deadline = time.monotonic() + 60.0
            required = {'joint_state_broadcaster', 'joint_trajectory_controller',
                        'arm2_joint_trajectory_controller', 'gripper_controller',
                        'arm2_gripper_controller'}
            active = set(); future = None; last_sim = 0.0; checks = {}
            while rclpy.ok() and time.monotonic() < deadline:
                now = time.monotonic(); sim = self.get_clock().now().nanoseconds / 1e9
                if future is not None and future.done():
                    try:active = {c.name for c in future.result().controller if c.state == 'active'}
                    except Exception:active = set()
                    future = None
                if future is None and self._ready_control.service_is_ready():
                    future = self._ready_control.call_async(ListControllers.Request())
                try:
                    _, subscribers = self._ready_gz.topic_info(self._belt_speed_topic)
                    transport_ok = bool(subscribers)
                except Exception:transport_ok = False
                checks = {
                    'world_clock_advances': sim > last_sim > 0.0,
                    'logical_fresh': now-self._startup_logical_wall < 1.0,
                    'rgb_fresh': now-self._startup_rgb_wall < 1.0,
                    'pusher_at_home': self._startup_pusher is not None and self._startup_pusher <= .14,
                    'spawn_bridge_service': self._ready_spawn.service_is_ready(),
                    'spawn_subscriber': self._pub_spawn.get_subscription_count() > 0,
                    'pusher_bridge_subscriber': self._pub_belt.get_subscription_count() > 0,
                    'pusher_transport_subscriber': transport_ok,
                    'controllers_active': required.issubset(active),
                }
                if all(checks.values()):
                    self.get_logger().info('STARTUP_READY '+json.dumps({'checks':checks,'active':sorted(active),'wall_wait':now-self._first_feed_t0,'sim':sim}))
                    self._request_spawn_and_start_belt()
                    return
                last_sim = sim
                time.sleep(.1)
            self._halted = True
            self.get_logger().error('STARTUP_READINESS_TIMEOUT: no feed; '+json.dumps(checks))
        threading.Thread(target=run, daemon=True).start()

'''
assert needle in s;s=s.replace(needle,method+needle)
s=s.replace("        with self._lock:\n            if self._halted:\n                return\n            if not self._demand_unlimited", "        with self._lock:\n            if self._halted:\n                return\n            if not self._demand_unlimited",1) # gates unchanged
s=s.replace('        self._pub_spawn.publish(request)',"        if not self._first_feed_logged:\n            self._first_feed_logged = True\n            self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'first_feed','wall':time.monotonic()-self._first_feed_t0,'fast':self._fast_waits,'sim':self.get_clock().now().nanoseconds/1e9}))\n        self._pub_spawn.publish(request)",1)
s=s.replace("            elif kind == DISPATCH:\n                if self._next_timer is None:","            elif kind == DISPATCH:\n                self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'dispatch_delay_path','configured':delay,'fast':self._fast_waits,'per_arm':self._per_arm_slots}))\n                if self._fast_waits and self._per_arm_slots:\n                    self._try_dispatch()  # existing atomic slot/zone/belt/dedup gates\n                    continue\n                if self._next_timer is None:",1)
p.write_text(s)
# Future notification preserves original timeout checks, predicates and fallback.
p=core/'motion_client.py';s=p.read_text().replace('import time','import time\nimport threading',1)
s=s.replace('        self._node = node',"        self._node = node\n        self._fast_waits = bool(node.get_parameter('fast_sim_waits').value)\n        self._wait_future = None\n        self._wait_event = None\n        self._slot_event = threading.Event()",1)
s=s.replace('        self._status_sub = None',"            if self._active[0] == 0:self._slot_event.set()\n            else:self._slot_event.clear()\n        self._status_sub = None",1)
s=s.replace('            time.sleep(0.1)',"            if getattr(self, '_fast_waits', False):self._slot_event.wait(.1)\n            else:time.sleep(.1)",1)
method='''    def _future_pause(self, future, seconds):
        if not getattr(self, '_fast_waits', False):
            time.sleep(seconds)
            return
        if self._wait_future is not future:
            self._wait_future = future
            self._wait_event = threading.Event()
            event = self._wait_event
            future.add_done_callback(lambda unused: event.set())
        self._wait_event.wait(seconds)

'''
s=s.replace('    # ---------------------------------------------------------------- 楔死',method+'    # ---------------------------------------------------------------- 楔死',1)
lines=s.splitlines(keepends=True);tree=ast.parse(s)
for node in ast.walk(tree):
 if not isinstance(node,ast.While):continue
 fs=[x.func.value.id for x in ast.walk(node.test) if isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute) and x.func.attr=='done' and isinstance(x.func.value,ast.Name)]
 if len(fs)!=1:continue
 for k in range(node.lineno-1,node.end_lineno):
  if 'time.sleep(0.05)' in lines[k]:lines[k]=lines[k].replace('time.sleep(0.05)',f'self._future_pause({fs[0]}, 0.05)')
s=''.join(lines);p.write_text(s)
# Follow sampler: record same paired data, wait for sufficient already-collected evidence.
p=core/'pick_place_executor_node.py';s=p.read_text().replace('import math','import json\nimport math',1)
s=s.replace('        self._part: list[tuple[float, float]] = []',"        self._part: list[tuple[float, float]] = []\n        self._sample_times = []\n        self._source_stamps = []\n        self._source_fn = lambda: None\n        self._condition = threading.Condition()",1)
s=s.replace('                self._gripper.append((float(g[0]), float(g[1])))\n                self._part.append((float(p[0]), float(p[1])))',"                with self._condition:\n                    self._gripper.append((float(g[0]), float(g[1])))\n                    self._part.append((float(p[0]), float(p[1])))\n                    self._sample_times.append(time.monotonic())\n                    self._source_stamps.append(self._source_fn())\n                    self._condition.notify_all()",1)
method='''    def wait_evidence(self, span, verifier):
        """Never shorten an unsuccessful/undersampled probe; retain original wall cap."""
        deadline = time.monotonic() + span
        with self._condition:
            while True:
                count = len(self._sample_times)
                elapsed = self._sample_times[-1]-self._sample_times[0] if count>1 else 0.
                distinct = len({x for x in self._source_stamps if x is not None})
                enough = count >= math.ceil(span/self._period)+1 and elapsed >= span and distinct >= max(2, math.ceil(span*10))
                if enough and verifier(self._gripper,self._part)[0]:return True
                remaining = deadline-time.monotonic()
                if remaining <= 0:return False
                self._condition.wait(remaining)

'''
s=s.replace('    def stop(self) -> tuple[list, list]:',method+'    def stop(self) -> tuple[list, list]:',1)
s=s.replace("        self.declare_parameter('verify_sample_span', 0.5)","        self.declare_parameter('fast_sim_waits', False)\n        self._fast_waits = bool(self.get_parameter('fast_sim_waits').value)\n        self._part_source_stamps = {}\n        self.declare_parameter('verify_sample_span', 0.5)",1)
s=s.replace('        self._cam_cache.update(\n',"        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9\n        for model in msg.model:\n            if model.name.startswith('part_'):self._part_source_stamps[model.name] = stamp\n        self._cam_cache.update(\n",1)
s=s.replace('                        time.sleep(self._verify_sample_span)',"                        t_wait = time.monotonic(); sim_wait = self.get_clock().now().nanoseconds/1e9\n                        evidence = False\n                        if self._fast_waits:\n                            evidence = sampler.wait_evidence(self._verify_sample_span, self._verifier.verify)\n                        else:\n                            time.sleep(self._verify_sample_span)\n                        self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'grasp_extra','wall':time.monotonic()-t_wait,'sim_start':sim_wait,'sim_end':self.get_clock().now().nanoseconds/1e9,'fast':self._fast_waits,'evidence':evidence,'pairs':len(sampler._gripper),'sample_wall_span':sampler._sample_times[-1]-sampler._sample_times[0] if len(sampler._sample_times)>1 else 0.,'distinct_source_frames':len(set(sampler._source_stamps))}))",1)
s=s.replace('                    time.sleep(0.4)',"                    t_wait = time.monotonic()\n                    time.sleep(0.4)  # retained: gap alone does not prove settled release\n                    self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'release_settle','wall':time.monotonic()-t_wait,'fast':self._fast_waits}))",1)
s=s.replace('                    time.sleep(1.5)',"                    t_wait = time.monotonic()\n                    time.sleep(1.5)\n                    self.get_logger().info('WAIT_METRIC '+json.dumps({'label':'drop_return_branch','wall':time.monotonic()-t_wait,'fast':self._fast_waits}))",1)
s=s.replace('        sampler.start()\n        return sampler',"        sampler._source_fn = lambda: self._part_source_stamps.get(part_id)\n        sampler.start()\n        return sampler",1)
p.write_text(s)
# Outer/inner launch switches. Original 15-second service startup only shortened in fast variant.
p=WS/'src/a1_sorting_core/launch/sorting_demo.launch.py';s=p.read_text();s=s.replace("                          'gazebo_gui': LaunchConfiguration('gazebo_gui'),","                          'gazebo_gui': LaunchConfiguration('gazebo_gui'),\n                          'fast_sim_waits': LaunchConfiguration('fast_sim_waits'),",1)
s=s.replace("{'use_sim_time': True,", "{'use_sim_time': True, 'fast_sim_waits': ParameterValue(LaunchConfiguration('fast_sim_waits'), value_type=bool),")
s=s.replace("        DeclareLaunchArgument('gazebo_gui'", "        DeclareLaunchArgument('fast_sim_waits', default_value='false'),\n        DeclareLaunchArgument('gazebo_gui'",1);p.write_text(s)
p=WS/'src/a1_cell_sim/launch/cell_bringup.launch.py';s=p.read_text();s=s.replace('    delayed_parts = TimerAction(period=15.0, actions=[spawn_parts])',"    fast = LaunchConfiguration('fast_sim_waits').perform(context).lower() in ('true','1','yes')\n    delayed_parts = spawn_parts if fast else TimerAction(period=15.0, actions=[spawn_parts])")
s=s.replace("        DeclareLaunchArgument('ur_type'", "        DeclareLaunchArgument('fast_sim_waits', default_value='false'),\n        DeclareLaunchArgument('ur_type'",1);p.write_text(s)
for p in list(core.glob('*.py'))+list((WS/'src/a1_sorting_core/launch').glob('*.py'))+list((WS/'src/a1_cell_sim/launch').glob('*.py')):compile(p.read_text(),str(p),'exec')
print('State waits and explicit default-off switch written')
