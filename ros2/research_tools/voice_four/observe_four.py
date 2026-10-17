"""Evidence and one authorized plan submission; no trajectory/part/Gazebo commands.

The only command client submits the already human-confirmed typed batch service.
All safety stops are requests to the owning supervisor, never global process kills.
"""
import csv,gzip,json,sys,threading,time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data,qos_profile_action_status_default
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String
from sensor_msgs.msg import JointState,Image
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import LogicalCameraImage
from control_msgs.msg import JointTrajectoryControllerState
from action_msgs.msg import GoalStatusArray
from a1_interfaces.msg import PartInfo,TaskStatus
from a1_interfaces.srv import SubmitBatchPlan
from a1_sorting_core.voice_batch_state import physical_ready_claim
from google.protobuf.json_format import MessageToDict
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from task_io import load_task,atomic_json

def main():
    run=Path(sys.argv[1]);record,plan=load_task(run/'approved-task.json');readonly='--no-submit' in sys.argv[2:]
    if not readonly and record.get('human_confirmed') is not True:raise RuntimeError('No human authorization')
    rclpy.init();node=rclpy.create_node('voice_four_evidence');gz=GzNode();lock=threading.RLock()
    raw=gzip.open(run/'raw-ros.jsonl.gz','wt',compresslevel=1);events=(run/'events.jsonl').open('w',buffering=1);stats=(run/'stats.jsonl').open('w',buffering=1)
    contactfile=(run/'contacts.csv').open('w',newline='');cw=csv.writer(contactfile);cw.writerow(['wall','sim_sec','topic','collision1','collision2','x','y','z','depth'])
    state={'ready_parts':[],'placed_parts':[],'spawn_requests':[],'arrivals':[],'latest_clock':0.,'logical_frames':0,'rgb_frames':0,'joint_messages':0,'controller_messages':0,'contact_points':0,'stats_messages':0}
    snapshot={};start=time.monotonic();lastflush=0.;submit_future=None;submitted=False;ready_at=None;submit_at=None;stopping=threading.Event()
    readyset=set();placedset=set();subscriptions=[];seen_actions=set()
    def emit(kind,**data):
        with lock:events.write(json.dumps(dict(kind=kind,wall=time.time(),clock=state['latest_clock'],**data),ensure_ascii=False)+'\n')
    def fault(kind,**data):
        if (run/'SUPERVISOR_STOP').exists():return
        try:
            with (run/'guard-fault.json').open('x') as f:json.dump(dict(kind=kind,wall=time.time(),**data),f)
        except FileExistsError:pass
    def raw_emit(kind,m):
        with lock:raw.write(json.dumps(dict(kind=kind,wall=time.time(),clock=state['latest_clock'],message=message_to_ordereddict(m)))+'\n')
    def joints(m):state['joint_messages']+=1;raw_emit('joint_state',m)
    def controller(m):state['controller_messages']+=1;raw_emit('controller_state',m)
    def clk(m):state['latest_clock']=m.clock.sec+m.clock.nanosec*1e-9
    def task(m):
        emit('task_status',part=m.part_id,arm=m.arm,state=m.state,detail=m.detail,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
        if m.state=='ARM_READY':
            if not physical_ready_claim(m.detail):fault('invalid_ARM_READY',part=m.part_id)
            else:readyset.add(m.part_id);state['ready_parts']=sorted(readyset)
        if m.state=='PART_PLACED':placedset.add(m.part_id);state['placed_parts']=sorted(placedset)
        if m.state in ('ARM_FAULT','FAILED','PART_REJECTED'):fault('executor_'+m.state,part=m.part_id,detail=m.detail)
    def spawn(m):
        state['spawn_requests'].append(m.data);emit('actual_spawn_request',color=m.data,submitted=submitted)
        if not submitted or state['spawn_requests']!=list(plan.order[:len(state['spawn_requests'])]) or len(state['spawn_requests'])>plan.total:fault('unauthorized_spawn')
        if len(state['spawn_requests'])>1 and not any(x.get('color')=='yellow' and x.get('part') in readyset and x.get('part') in placedset for x in state['arrivals']):fault('yellow_not_finished_before_red')
    def arrival(m):
        state['arrivals'].append({'part':m.part_id,'color':m.category});emit('vision_arrival',part=m.part_id,color=m.category,pose=message_to_ordereddict(m.pose))
        i=len(state['arrivals'])-1
        if not submitted or i>=plan.total or m.category!=plan.order[i] or len({r['part'] for r in state['arrivals']})!=i+1:fault('unauthorized_duplicate_or_wrong_visual_arrival')
    def batch(m):snapshot.clear();snapshot.update(json.loads(m.data));emit('batch_snapshot',snapshot=snapshot)
    def logical(m):
        state['logical_frames']+=1
        emit('logical',stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,models=[{'name':x.name,'pose':message_to_ordereddict(x.pose)} for x in m.model])
    def rgb(m):state['rgb_frames']+=1
    def action(m):
        raw_emit('action_status',m)
        for s in m.status_list:
            key=(bytes(s.goal_info.goal_id.uuid).hex(),s.status)
            if key not in seen_actions:
                seen_actions.add(key);emit('action_status',goal_id=key[0],status=s.status)
            if s.status in (5,6):fault('controller_action_cancelled_or_aborted',goal_id=key[0],status=s.status)
    for cls,topic,cb,qos in [(Clock,'/clock',clk,qos_profile_sensor_data),(JointState,'/joint_states',joints,qos_profile_sensor_data),(Image,'/overhead_camera',rgb,qos_profile_sensor_data),(LogicalCameraImage,'/overhead_logical',logical,10),(String,'/parts/request_spawn',spawn,10),(PartInfo,'/parts/arrival',arrival,10),(TaskStatus,'/task/status',task,20),(TaskStatus,'/task/placement_result',task,20)]:subscriptions.append(node.create_subscription(cls,topic,cb,qos))
    subscriptions.append(node.create_subscription(String,'/voice_batch/status',batch,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
    for base in ('/joint_trajectory_controller','/arm2_joint_trajectory_controller'):
        subscriptions.append(node.create_subscription(JointTrajectoryControllerState,base+'/controller_state',controller,qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(GoalStatusArray,base+'/follow_joint_trajectory/_action/status',action,qos_profile_action_status_default))
    def on_stats(m):
        with lock:
            if stopping.is_set():return
            state['stats_messages']+=1;stats.write(json.dumps(dict(wall=time.time(),stats=MessageToDict(m,preserving_proto_field_name=True)))+'\n')
    def on_contact(topic):
        def callback(m):
            t=m.header.stamp.sec+m.header.stamp.nsec*1e-9
            with lock:
                if stopping.is_set():return
                for c in m.contact:
                    a=c.collision1.name;b=c.collision2.name
                    def arm(x):
                        if 'arm2_' in x:return 2
                        return 1 if x.startswith('ur::') else 0
                    aa,bb=arm(a),arm(b)
                    for i,point in enumerate(c.position):
                        depth=c.depth[i] if i<len(c.depth) else None
                        cw.writerow([time.time(),t,topic,a,b,point.x,point.y,point.z,depth]);state['contact_points']+=1
                    if any(d>0 for d in c.depth):
                        if aa and bb and aa!=bb:fault('native_interarm_contact',sim=t,a=a,b=b)
                        if ((aa and b.startswith('bin_')) or (bb and a.startswith('bin_'))):fault('robot_bin_contact_requires_review',sim=t,a=a,b=b,max_depth=max(c.depth))
        return callback
    gz.subscribe(WorldStatistics,'/world/sorting_cell/stats',on_stats)
    links=['pedestal','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link','gripper_finger_left_link','gripper_finger_right_link','pedestal2','arm2_shoulder_link','arm2_upper_arm_link','arm2_forearm_link','arm2_wrist_1_link','arm2_wrist_2_link','arm2_wrist_3_link','arm2_gripper_finger_left_link','arm2_gripper_finger_right_link']
    callbacks=[]
    for link in links:
        topic='/a1_contact/'+link;cb=on_contact(topic);callbacks.append(cb)
        if not gz.subscribe(Contacts,topic,cb):raise RuntimeError('contact subscription failed:'+topic)
    client=None if readonly else node.create_client(SubmitBatchPlan,'/voice_batch/submit')
    (run/'OBSERVER_READY').touch()
    try:
        while not (run/'SUPERVISOR_STOP').exists():
            rclpy.spin_once(node,timeout_sec=.025);now=time.monotonic()
            if (run/'guard-fault.json').exists():continue
            if not readonly and not submitted and snapshot.get('hardware_ready') and snapshot.get('state')=='WAITING_FOR_PLAN':
                if ready_at is None:ready_at=now
                # Independently record at least 3 seconds ready, no accepted plan,
                # no requests, no visible arrivals. Authorization never fakes hardware.
                if now-ready_at>=3 and client.service_is_ready():
                    if state['spawn_requests'] or state['arrivals']:fault('not_zero_before_submit');continue
                    req=SubmitBatchPlan.Request();req.session_id=snapshot['session_id'];req.request_id=record['request_id'];req.user_confirmed=True;req.plan_sha256=plan.sha256;req.blue,req.yellow,req.red,req.green=plan.counts;req.priority_first=plan.priority_first;req.proposed_order=list(plan.order)
                    atomic_json(run/'submission-receipt.json',dict(request_id=req.request_id,session_id=req.session_id,plan=plan.as_dict(),plan_sha256=plan.sha256,human_confirmed=True,ready_without_plan_seconds=now-ready_at,pre_submit_spawn_requests=[],outcome='pending_unknown_do_not_resend'))
                    submitted=True;submit_at=now;submit_future=client.call_async(req);emit('submit_once',request_id=req.request_id)
            if submit_future is not None and submit_future.done():
                response=submit_future.result();receipt=json.loads((run/'submission-receipt.json').read_text());receipt.update(accepted=response.accepted,message=response.message,outcome='accepted' if response.accepted else 'rejected');atomic_json(run/'submission-receipt.json',receipt)
                if not response.accepted:fault('plan_rejected',message=response.message)
                submit_future=None
            if submit_future is not None and now-submit_at>10:fault('submission_outcome_unknown_no_resend')
            if now-lastflush>.25:
                lastflush=now
                state['clock_publishers']=[{'node':p.node_name,'gid':list(p.endpoint_gid)} for p in node.get_publishers_info_by_topic('/clock')]
                if len(state['clock_publishers'])>1:fault('multiple_clock_publishers')
                with lock:raw.flush();contactfile.flush();atomic_json(run/'observed-status.json',state);atomic_json(run/'observer-health.json',dict(wall=time.time(),clock=state['latest_clock']))
            if now-start>590:fault('observer_wall_bound')
    except KeyboardInterrupt:pass
    finally:
        with lock:stopping.set();raw.close();events.close();stats.close();contactfile.close()
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
