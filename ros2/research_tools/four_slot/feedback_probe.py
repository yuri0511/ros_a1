"""Read-only feedback test, never creates a command publisher or action client."""
import argparse,json,time,threading,subprocess,sys,os
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import JointState
from joint_receiver import JointReceiver
from a1_sorting_core.four_slot_feedback import ReleaseEvidence

NAMES=['gripper_finger_left_joint','gripper_finger_right_joint']
def main():
 p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--publisher',action='store_true');p.add_argument('--seconds',type=float,default=20);p.add_argument('--synthetic',action='store_true');p.add_argument('--load',action='store_true');args=p.parse_args();args.output.mkdir(exist_ok=True,parents=True)
 rclpy.init();n=rclpy.create_node('feedback_probe_publisher' if args.publisher else 'feedback_probe_reference');qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
 if args.publisher:
  pub=n.create_publisher(JointState,'/joint_states',qos);time.sleep(1);begin=time.monotonic();i=0
  with (args.output/'published.jsonl').open('w') as f:
   while time.monotonic()-begin<args.seconds:
    elapsed=time.monotonic()-begin
    # An intentional real source pause must remain a safety fault.
    if 12<=elapsed<12.09:time.sleep(.001);continue
    m=JointState();t=time.monotonic()-begin;m.header.stamp.sec=int(t);m.header.stamp.nanosec=int(t%1*1e9);m.name=NAMES;m.position=[.012,.012];m.velocity=[0.,0.];pub.publish(m);f.write(json.dumps({'stamp':t,'wall':time.time()})+'\n');i+=1;time.sleep(.001 if elapsed>6 else .002)
  n.destroy_node();rclpy.shutdown();return
 rows=[];faults=[];monitor=ReleaseEvidence(NAMES);monitor.begin();armed=[False];last=[None];gaps=[]
 def handle(m,info,receipt,source):
  t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if last[0] is not None and t-last[0]>.05:
   gaps.append({'before':last[0],'after':t,'gap':t-last[0]})
   if not args.synthetic:fault('joint_source_gap')
  last[0]=t
  if args.synthetic:monitor.observe(m.name,m.position,t,receipt,source)
  rows.append({'stamp':t,'receipt':receipt})
 def fault(reason):
  faults.append({'wall':time.time(),'reason':reason})
  if not args.synthetic:
   try:
    with (args.output/'guard-fault.json').open('x') as f:json.dump(faults[-1],f)
   except FileExistsError:pass
 receiver=JointReceiver(handle,fault,args.output/'ingress.jsonl');reference=[]
 sub=n.create_subscription(JointState,'/joint_states',lambda m:reference.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'receipt':time.monotonic()}),qos)
 executor=SingleThreadedExecutor();executor.add_node(n);thread=threading.Thread(target=executor.spin);thread.start();child=None
 if args.synthetic:child=subprocess.Popen([sys.executable,__file__,str(args.output),'--publisher','--seconds',str(args.seconds)])
 load_stop=threading.Event()
 def load():
  while not load_stop.is_set():sum(range(1000000));time.sleep(.005)
 load_thread=threading.Thread(target=load) if args.load else None
 if load_thread:load_thread.start()
 start=time.monotonic()
 try:
  while time.monotonic()-start<args.seconds+(3 if args.synthetic else 0):
   if not args.synthetic and (args.output/'STOP').exists():break
   if (args.output/'SHUTDOWN_INTENT').exists():break
   if not args.synthetic and rows and time.monotonic()-rows[-1]['receipt']>.5:fault('joint_receive_stale')
   time.sleep(.01)
 finally:
  if child is not None:child.wait(timeout=5)
  load_stop.set()
  if load_thread:load_thread.join(2)
  summary=receiver.close();executor.shutdown(timeout_sec=2);thread.join(2);n.destroy_node();rclpy.shutdown()
  (args.output/'reference.json').write_text(json.dumps(reference));(args.output/'consumer.json').write_text(json.dumps(rows))
  rs={r['stamp'] for r in reference};cs={r['stamp'] for r in rows};shared=(max(min(rs),min(cs)),min(max(rs),max(cs))) if rs and cs else (0,0)
  missing=sorted(t for t in rs-cs if shared[0]<=t<=shared[1]);extra=sorted(t for t in cs-rs if shared[0]<=t<=shared[1])
  result={'synthetic':args.synthetic,'receiver':summary,'reference_count':len(reference),'consumer_count':len(rows),'missing_from_consumer':missing,'missing_from_reference':extra,'source_gaps':gaps,'release_monitor_fault':monitor.fault,'faults':faults,'max_gap_limit':.05,'no_commands_or_actions':True}
  (args.output/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='receiver'},indent=2))
if __name__=='__main__':main()
