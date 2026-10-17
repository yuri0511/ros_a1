"""ROS service transport check with NO Gazebo, no actuators and no ready signals."""
import json,time,os
if os.environ.get('ROS_DOMAIN_ID')!='96' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':raise SystemExit('REFUSED: mock requires isolated domain96 LOCALHOST')
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from a1_interfaces.srv import SubmitBatchPlan,SortTask
from a1_sorting_core.task_dispatcher_node import TaskDispatcherNode
from a1_sorting_core.voice_batch_plan import validate_candidate

rclpy.init(args=['--ros-args','-p','voice_batch_mode:=true','-p','fast_sim_waits:=true','-p','demand_spawn:=true','-p','belt_control:=true','-p','arm2_colors:=red,green','-p','q_table_path:=""'])
server=TaskDispatcherNode();client=rclpy.create_node('voice_transport_smoke');exe=SingleThreadedExecutor();exe.add_node(server);exe.add_node(client);requests=[];statuses=[]
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
sub=client.create_subscription(String,'/parts/request_spawn',lambda m:requests.append(m.data),q)
status=client.create_subscription(String,'/voice_batch/status',lambda m:statuses.append(json.loads(m.data)),q)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:exe.spin_once(timeout_sec=.05)
def call(typ,name,request):
 c=client.create_client(typ,name);assert c.wait_for_service(timeout_sec=3);future=c.call_async(request);end=time.monotonic()+5
 while not future.done() and time.monotonic()<end:exe.spin_once(timeout_sec=.05)
 assert future.done();return future.result()
def request(count):
 p=validate_candidate(json.dumps(dict(schema_version=1,action='sort_batch',counts=dict(blue=0,yellow=1,red=count,green=0),priority_first='yellow',proposed_order=['yellow']+['red']*count)))
 r=SubmitBatchPlan.Request();r.session_id=server._voice.session_id;r.request_id='a'*32;r.user_confirmed=True;r.plan_sha256=p.sha256;r.blue,r.yellow,r.red,r.green=p.counts;r.priority_first=p.priority_first;r.proposed_order=list(p.order);return r
# Scope simulation only, no calibration authority or robot nodes exist here.
server._trial_permit='TEST_CONTRACT_ONLY_NO_GAZEBO'
out={'mock_only':True,'no_robot_or_Gazebo':True}
try:
 spin(.5);out['before_confirmation_spawn_requests']=list(requests);assert not requests
 unconfirmed=request(0);unconfirmed.user_confirmed=False;r=call(SubmitBatchPlan,'/voice_batch/submit',unconfirmed);assert not r.accepted;out['unconfirmed_rejected']=r.message
 r=call(SubmitBatchPlan,'/voice_batch/submit',request(1));assert not r.accepted and r.message=='single_yellow_trial_only';out['primary_plan_safety_rejected']=r.message
 r=call(SubmitBatchPlan,'/voice_batch/submit',request(0));assert r.accepted;out['safe_subset_accepted_but_pending_hardware']=r.message
 r=call(SubmitBatchPlan,'/voice_batch/submit',request(0));assert not r.accepted;out['duplicate_rejected']=r.message
 legacy=SortTask.Request();legacy.command='start_batch';r=call(SortTask,'/sort_task',legacy);assert not r.accepted;out['legacy_service_blocked']=r.message
 spin(.5);assert not requests;out['without_gazebo_readiness_spawn_requests']=list(requests)
 r=call(Trigger,'/voice_batch/stop',Trigger.Request());assert r.success;assert server._voice.state=='HALTED';out['stop_latched']=True;out['status_states']=list(dict.fromkeys(s['state'] for s in statuses));out['no_gazebo_started']=True
finally:
 server.destroy_node();client.destroy_node();exe.shutdown();rclpy.try_shutdown()
print('SINGLE_YELLOW_ROS_MOCK_RESULT '+json.dumps(out,ensure_ascii=False))
