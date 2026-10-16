import rclpy,time,json,sys
from pathlib import Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState,Image
from rclpy.qos import qos_profile_sensor_data
run=Path(sys.argv[1]);rclpy.init();n=rclpy.create_node('short_latency_probe');clock=0.;counts={};f=(run/'latency-window.jsonl').open('w',buffering=1)
def c(m):
 global clock
 clock=m.clock.sec+m.clock.nanosec/1e9
def sample(k,m):
 counts[k]=counts.get(k,0)+1
 if counts[k]%10==1:f.write(json.dumps({'wall':time.time(),'type':k,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'clock':clock,'sim_age':clock-m.header.stamp.sec-m.header.stamp.nanosec/1e9})+'\n')
subs=[n.create_subscription(Clock,'/clock',c,qos_profile_sensor_data),n.create_subscription(JointState,'/joint_states',lambda m:sample('joints',m),qos_profile_sensor_data),n.create_subscription(Image,'/overhead_camera',lambda m:sample('rgb',m),qos_profile_sensor_data)]
start=time.time()
while time.time()-start<10:rclpy.spin_once(n,timeout_sec=.1)
f.close();(run/'latency-window-summary.json').write_text(json.dumps({'start':start,'end':time.time(),'counts':counts}));n.destroy_node();rclpy.shutdown()
