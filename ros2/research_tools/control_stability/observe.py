"""Read-only bounded full-rate ROS evidence, with external safety stop request."""
import sys,time,json,gzip,threading
from pathlib import Path
from a1_sorting_core.physical_ready import publisher_identity
import rclpy
from rclpy.qos import qos_profile_sensor_data,qos_profile_action_status_default
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from a1_interfaces.msg import TaskStatus
from action_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from rosidl_runtime_py.convert import message_to_ordereddict
D=Path(sys.argv[1]);f=gzip.open(D/'high-rate.jsonl.gz','wt',compresslevel=1);clock=0;ready=0;count=0;start=time.monotonic();lastflush=0
rclpy.init();n=rclpy.create_node('physical_ready_evidence')
def fault(kind,**kw):
 try:
  with (D/'guard-fault.json').open('x') as h:json.dump({'kind':kind,'wall':time.time(),**kw},h)
 except FileExistsError:pass
def emit(kind,**kw):
 global count
 f.write(json.dumps({'wall':time.time(),'monotonic':time.monotonic(),'observed_clock':clock,'kind':kind,**kw})+'\n');count+=1
 if count>2000000:fault('diagnostic_record_bound')
def joints(m,info):emit('joint_state',stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,names=list(m.name),positions=list(m.position),velocities=list(m.velocity),efforts=list(m.effort),publisher_gid=publisher_identity(info,n.get_publishers_info_by_topic('/joint_states')))
def clk(m):
 global clock
 clock=m.clock.sec+m.clock.nanosec*1e-9

def status(m):
 global ready
 emit('task_status',stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,part=m.part_id,arm=m.arm,state=m.state,detail=m.detail)
 if m.state=='ARM_READY':
  try:
   d=json.loads(m.detail);e=d['evidence'];c=d['controller_result']
   if not (e['ready'] and e['samples']>=50 and e['stable_span']>=.25 and e['max_reported_speed']<=.3 and e['max_position_error']<=.01 and e['max_difference_speed']<=.3 and c['status']==4 and c['error_code']==0 and d['no_active_goal']):fault('invalid_arm_ready_evidence',part=m.part_id,detail=d)
   else:ready+=1;(D/'verified-ready-count.txt').write_text(str(ready))
  except (KeyError,ValueError,TypeError):fault('missing_arm_ready_evidence',part=m.part_id,detail=m.detail)
 if m.state in ('ARM_FAULT','FAILED','PART_REJECTED'):fault('executor_'+m.state,part=m.part_id,arm=m.arm,detail=m.detail)
subs=[n.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data),n.create_subscription(Clock,'/clock',clk,qos_profile_sensor_data),n.create_subscription(TaskStatus,'/task/status',status,20),n.create_subscription(TaskStatus,'/task/placement_result',status,20)]
for base in ('/joint_trajectory_controller','/arm2_joint_trajectory_controller'):
 subs.append(n.create_subscription(JointTrajectoryControllerState,base+'/controller_state',lambda m,b=base:emit('controller_state',controller=b,message=message_to_ordereddict(m)),qos_profile_sensor_data))
 subs.append(n.create_subscription(GoalStatusArray,base+'/follow_joint_trajectory/_action/status',lambda m,b=base:emit('action_status',controller=b,message=message_to_ordereddict(m)),qos_profile_action_status_default))
try:
 while not (D/'PROBE_STOP').exists():
  rclpy.spin_once(n,timeout_sec=.05)
  if time.monotonic()-lastflush>1:
   lastflush=time.monotonic();f.flush()
   (D/'observer-health.json').write_text(json.dumps({'records':count,'wall':time.time(),'clock':clock,'joint_publishers':[{'node':p.node_name,'gid':list(p.endpoint_gid)} for p in n.get_publishers_info_by_topic('/joint_states')]}))
   if time.monotonic()-start>600:fault('diagnostic_time_bound')
except KeyboardInterrupt:pass
finally:
 f.close();n.destroy_node();rclpy.shutdown()
