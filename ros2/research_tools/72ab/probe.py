import sys,os,time,json,threading,hashlib
from pathlib import Path
from google.protobuf.json_format import MessageToDict
import gz.transport13 as gt
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.image_pb2 import Image as GzImage
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import LogicalCameraImage
from std_msgs.msg import String
from a1_interfaces.msg import PartInfo,TaskStatus
run=Path(sys.argv[1]); token=sys.argv[2]
full_clock=os.environ.get('PERF_ISOLATION_ONLY')=='1' or os.environ.get('PERF_PROBE_MODE')=='full'
latestgz=0.
raw=(run/'stats.jsonl').open('w',buffering=1); events=(run/'ros-probe.jsonl').open('w',buffering=1)
lock=threading.Lock(); gzseen=set(); rosseen=set(); counts={}; latestclock=0.; prevclock=None
rclpy.init(); n=rclpy.create_node('performance_probe_'+token.replace('-','_'))
g=gt.Node(); gp=g.advertise('/performance/isolation_marker',StringMsg)
def stats(m):
 global latestgz
 latestgz=m.sim_time.sec+m.sim_time.nsec/1e9
 with lock: raw.write(json.dumps({'wall':time.time(),'stats':MessageToDict(m,preserving_proto_field_name=True)})+'\n')
def gm(m):gzseen.add(m.data)
g.subscribe(WorldStatistics,'/world/sorting_cell/stats',stats)
g.subscribe(StringMsg,'/performance/isolation_marker',gm)
def gz_image(m):counts['gz_rgb']=counts.get('gz_rgb',0)+1
g.subscribe(GzImage,'/overhead_camera',gz_image)
pub=n.create_publisher(String,'/performance/isolation_marker',10)
def clock(m):
 global latestclock,prevclock
 latestclock=m.clock.sec+m.clock.nanosec/1e9
 if prevclock is not None and latestclock<prevclock:events.write(json.dumps({'wall':time.time(),'type':'clock_backwards','previous':prevclock,'clock':latestclock})+'\n')
 prevclock=latestclock

def sensor(kind,m):
 counts[kind]=counts.get(kind,0)+1
 if kind in ('task','arrival') or counts[kind]%10==1:
  row={'wall':time.time(),'type':kind,'clock':latestclock}
  if hasattr(m,'header'):
   row['stamp']=m.header.stamp.sec+m.header.stamp.nanosec/1e9
   if full_clock:row['sim_age']=latestclock-row['stamp']
  if kind=='rgb':row.update(width=m.width,height=m.height,encoding=m.encoding,hash=hashlib.sha256(bytes(m.data)).hexdigest())
  if kind=='arrival':row.update(part=m.part_id,color=m.category)
  if kind=='task':row.update(part=m.part_id,state=m.state,arm=m.arm,detail=m.detail,task_id=m.task_id)
  events.write(json.dumps(row)+'\n')
subs=[n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)] if full_clock else []
for cls,topic,kind in [(Image,'/overhead_camera','rgb'),(JointState,'/joint_states','joints'),(LogicalCameraImage,'/overhead_logical','logical'),(PartInfo,'/parts/arrival','arrival'),(TaskStatus,'/task/status','task')]:
 if kind=='joints' and not full_clock:continue
 subs.append(n.create_subscription(cls,topic,lambda m,k=kind:sensor(k,m),qos_profile_sensor_data if kind in ('rgb','joints') else 10))
subs.append(n.create_subscription(String,'/performance/isolation_marker',lambda m:rosseen.add(m.data),10))
last=0
while not (run/'PROBE_STOP').exists():
 rclpy.spin_once(n,timeout_sec=.1)
 if time.monotonic()-last>1:
  last=time.monotonic(); pub.publish(String(data=token));gp.publish(StringMsg(data=token))
  status={'wall':time.time(),'clock':latestclock,'gz_clock':latestgz,'clock_source':'ROS topic' if full_clock else 'not continuously sampled','counts':counts,'ros_markers':sorted(rosseen),'gz_markers':sorted(gzseen),'clock_publishers':[{'node':x.node_name,'gid':list(x.endpoint_gid)} for x in n.get_publishers_info_by_topic('/clock')],'controllers':sum(1 for x,ns in n.get_node_names_and_namespaces() if x=='controller_manager')}
  (run/'probe-status.json').write_text(json.dumps(status))
  events.write(json.dumps({'type':'status',**status})+'\n')
n.destroy_node();rclpy.shutdown();raw.close();events.close()
