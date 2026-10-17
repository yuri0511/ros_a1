"""MoveIt-only replay from preserved real pickup observations. Never executes."""
import copy,json,gzip,re,time,subprocess,os,threading,sys,hashlib
from pathlib import Path
import numpy as np,rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints,JointConstraint
from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene,GetStateValidity,GetMotionPlan
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict as md
from a1_sorting_core.held_payload_scene import HeldPayloadScene
from a1_sorting_core.four_slot_scene import collision_objects
from a1_sorting_core.attach_helper import mat_to_pose,pose_to_mat
from a1_sorting_core.retained_scene_sync import camera_observation
from geometry import ROOT,ROBOT,fk,origin
from support_contact import sample_trajectory
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1]
sys.path.insert(0,str(HERE.parent/'voice_four'))
from run_four import stop_group

def recorded_case():
 base=Path('/home/yu/yuri_gazebo_performance/voice_batch_four/runs/four-20261017T081850-4d18c0');joints=[];controls=[];observations=[]
 camera=mat_to_pose(origin(__import__('xml.etree.ElementTree',fromlist=['parse']).parse(WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf').find(".//model[@name='overhead_camera_model']/pose"),True))
 for l in (base/'events.jsonl').open():
  a=json.loads(l)
  if a.get('kind')=='logical' and 1792196363.3<a['wall']<1792196363.583:
   for model in a['models']:
    if model['name']=='part_1':
     p=Pose();set_message_fields(p,model['pose']);observations.append(camera_observation(camera,p,a['stamp']))
 with gzip.open(base/'raw-ros.jsonl.gz','rt') as f:
  for l in f:
   a=json.loads(l)
   if a['kind']=='joint_state' and 1792196363.3<a['wall']<1792196363.59:joints.append(a['message'])
   if a['kind']=='controller_state' and 1792196363.58<a['wall']<1792196367.08 and not a['message']['joint_names'][0].startswith('arm2_'):controls.append(a)
 obs=observations[-1];jm=min(joints,key=lambda x:abs(x['header']['stamp']['sec']+x['header']['stamp']['nanosec']/1e9-obs['stamp']));vals=dict(zip(jm['name'],jm['position']));gripper=mat_to_pose(fk('gripper_base',vals));targets=[]
 for l in (base/'launch.log').open():
  if 'CONTROLLER_RESULT ' not in l:continue
  a=json.loads(l.split('CONTROLLER_RESULT ',1)[1]);m=re.search(r'\[(179\d+\.\d+)\]',l)
  if a['label'] in ('lift','verify_probe_out','verify_probe_back','carry_rise','carry_clear'):
   wall=float(m.group(1));c=min([x for x in controls if x['wall']>=wall],key=lambda x:x['wall']);targets.append((a['label'],c['message']['joint_names'],c['message']['reference']['positions']))
 return base,jm,obs,gripper,targets

def main():
 out=ROOT/'final_concentrated'/('payload-plan-'+time.strftime('%Y%m%dT%H%M%S'));out.mkdir();print(out,flush=True);os.environ.update(ROS_DOMAIN_ID='97',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(out/'ros'))
 log=(out/'planner.log').open('w');server=subprocess.Popen(['ros2','launch',str(HERE/'plan_only.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();n=rclpy.create_node('real_payload_replay');ex=rclpy.executors.SingleThreadedExecutor();ex.add_node(n);thread=threading.Thread(target=ex.spin);thread.start();result={'no_gazebo':True,'no_motion':True,'segments':[]};model=None
 def call(c,q):
  if not c.wait_for_service(timeout_sec=20):raise RuntimeError('service_missing')
  f=c.call_async(q);end=time.monotonic()+20
  while not f.done():
   if time.monotonic()>end:raise RuntimeError('RPC_timeout')
   time.sleep(.0005)
  return f.result()
 try:
  apply=n.create_client(ApplyPlanningScene,'/apply_planning_scene');read=n.create_client(GetPlanningScene,'/get_planning_scene');valid=n.create_client(GetStateValidity,'/check_state_validity');planner=n.create_client(GetMotionPlan,'/plan_kinematic_path')
  from moveit_msgs.msg import PlanningSceneComponents
  def get():
   q=GetPlanningScene.Request();q.components.components=PlanningSceneComponents.ROBOT_STATE|PlanningSceneComponents.WORLD_OBJECT_GEOMETRY|PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.ALLOWED_COLLISION_MATRIX;return call(read,q).scene
  base,jm,obs,gripper,targets=recorded_case();req=ApplyPlanningScene.Request();req.scene.is_diff=True;req.scene.world.collision_objects=collision_objects(WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf');req.scene.robot_state.is_diff=True;set_message_fields(req.scene.robot_state.joint_state,jm);assert call(apply,req).success
  model=HeldPayloadScene(lambda q:call(apply,q).success,get,lambda **kw:result.setdefault('events',[]).append(kw));model.attach('part_1',obs,gripper,obs['stamp'],obs['stamp'],'gripper_base',['gripper_finger_left_link','gripper_finger_right_link'],clamp_verified=True)
  result['reconstructed_case']=dict(source=str(base),joint_state=jm,observation=obs,gripper=md(gripper),targets=targets,held=md(model.held),TF_source='offline FK at observed source stamp; not fabricated live TF')
  check=GetStateValidity.Request();check.robot_state=copy.deepcopy(get().robot_state);ans=call(valid,check);result['start_validity']=md(ans)
  pair={model.held.object.id,'belt_belt_collision'}
  if not ans.valid:
   if any({c.contact_body_1,c.contact_body_2}!=pair for c in ans.contacts):raise RuntimeError('start_has_non_support_collision')
   model.permit_support('belt_belt_collision',contact_verified=True,clamp_verified=True,direction='depart');result['scoped_plan_only_support_hypothesis']=True
  initial=pose_to_mat(gripper)@pose_to_mat(model.held.object.pose)@pose_to_mat(model.held.object.primitive_poses[0]);initialbottom=float(initial[2,3]-np.abs(initial[2,:3])@np.array([.025]*3))
  state=get().robot_state
  for label,names,goal in targets:
   q=GetMotionPlan.Request();p=q.motion_plan_request;p.group_name='ur_manipulator';p.pipeline_id='pilz_industrial_motion_planner';p.planner_id='LIN';p.allowed_planning_time=3.;p.num_planning_attempts=1;p.max_velocity_scaling_factor=.2;p.max_acceleration_scaling_factor=.2;p.start_state=copy.deepcopy(state);p.start_state.joint_state.velocity=[0.]*len(state.joint_state.name);p.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=v,tolerance_above=.01,tolerance_below=.01,weight=1.) for n,v in zip(names,goal)])]
   response=call(planner,q);row={'stage':label,'code':response.motion_plan_response.error_code.val,'request':md(q),'response':md(response),'samples':[]};result['segments'].append(row)
   if row['code']!=1:raise RuntimeError('held_plan_failed:'+label)
   tr=response.motion_plan_response.trajectory.joint_trajectory;start=dict(zip(state.joint_state.name,state.joint_state.position))
   for t,vals in sample_trajectory(tr,start):
    M=fk('gripper_base',vals)@pose_to_mat(model.held.object.pose)@pose_to_mat(model.held.object.primitive_poses[0]);bottom=float(M[2,3]-abs(M[2,:3])@np.array([.025]*3))
    if model.base is not None and bottom<initialbottom-64*np.finfo(float).eps:
     result['support_violation']={'stage':label,'t':t,'initial_bottom':initialbottom,'planned_bottom':bottom,'growth_m':initialbottom-bottom,'joints':vals};raise RuntimeError('planned_departure_penetration_grows')
    if model.base is not None and bottom>.780:model.restore_support(separation_verified=True)
    v=GetStateValidity.Request();v.robot_state=copy.deepcopy(state);v.robot_state.joint_state.position=[vals[n] for n in state.joint_state.name];a=call(valid,v);row['samples'].append(dict(t=t,bottom=bottom,valid=a.valid,contacts=[md(x) for x in a.contacts]))
    if not a.valid:raise RuntimeError('held_path_collision:'+label)
   end=dict(start);end.update(zip(tr.joint_names,tr.points[-1].positions));state.joint_state.position=[end[n] for n in state.joint_state.name];print(label,len(row['samples']),flush=True)
  result['pass']=True
 except Exception as e:result['pass']=False;result['blocker']=str(e);print(str(e),flush=True)
 finally:
  if model and model.base is not None:
   try:model.restore_support(world_paused=True) # planner-only process, no motion authority
   except Exception as e:result['cleanup_error']=str(e)
  ex.shutdown(timeout_sec=2);thread.join(2);n.destroy_node();rclpy.try_shutdown();stop_group(server);log.close();(out/'RESULT.json').write_text(json.dumps(result,indent=2));(ROOT/'final_concentrated/latest-payload-review.txt').write_text(str(out))
if __name__=='__main__':main()
