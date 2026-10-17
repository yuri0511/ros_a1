"""Selected real Pilz requests plus <=1ms FCL replay. No Gazebo or execution."""
import json,sys,time,copy,subprocess,os,threading,hashlib
from pathlib import Path
import rclpy
import numpy as np
from moveit_msgs.srv import GetMotionPlan,ApplyPlanningScene,GetStateValidity
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict as msgdict
from a1_sorting_core.four_slot_scene import collision_objects
from a1_sorting_core.four_slot_trial import gripper_bottoms
from geometry import ROOT,ROBOT
from support_contact import sample_trajectory
from diagnose_held import remove_char_operations
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'voice_four'))
from run_four import stop_group
HERE=Path(__file__).resolve().parent;WS=HERE.parents[1]
def main():
 out=ROOT/'final_concentrated'/('all-paths-'+time.strftime('%Y%m%dT%H%M%S'));out.mkdir();print(out,flush=True)
 os.environ.update(ROS_DOMAIN_ID='97',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(out/'ros'))
 log=(out/'planner.log').open('w');server=subprocess.Popen(['ros2','launch',str(HERE/'plan_only.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();n=rclpy.create_node('acceptance_path_review');ex=rclpy.executors.SingleThreadedExecutor();ex.add_node(n);thread=threading.Thread(target=ex.spin);thread.start()
 def wait(f):
  end=time.monotonic()+10
  while not f.done():
   if time.monotonic()>end:raise RuntimeError('RPC_bound')
   time.sleep(.0003)
  return f.result()
 def client(cls,name):
  c=n.create_client(cls,name)
  if not c.wait_for_service(timeout_sec=20):raise RuntimeError('service_missing')
  return c
 result={'no_gazebo':True,'no_motion':True,'segments':[],'nominal_model_only':True};requestlog=[]
 try:
  plan=client(GetMotionPlan,'/plan_kinematic_path');apply=client(ApplyPlanningScene,'/apply_planning_scene');valid=client(GetStateValidity,'/check_state_validity')
  cfg=json.loads((WS/'src/a1_sorting_core/config/four_slot_candidate.json').read_text());urdf=(ROOT/'audit/robot-static.urdf').read_text()
  from moveit_msgs.msg import CollisionObject
  from shape_msgs.msg import SolidPrimitive
  from geometry_msgs.msg import Pose
  def box(name,xyz):
   o=CollisionObject();o.id=name;o.header.frame_id='world';o.operation=o.ADD;p=SolidPrimitive();p.type=p.BOX;p.dimensions=[.05]*3;pose=Pose();pose.position.x,pose.position.y,pose.position.z=xyz;pose.orientation.w=1.;o.primitives=[p];o.primitive_poses=[pose];return o
  old=json.loads((ROOT/'audit/stage2/plans.json').read_text());selected=[r for r in old if r['slot'] in ('bin_yellow/slot_1','bin_red/slot_4','bin_red/slot_1','bin_green/slot_1','bin_green/slot_2','bin_green/slot_3')]
  from a1_sorting_core.attach_helper import pose_to_mat,mat_to_pose
  from geometry import fk
  actual=json.loads((Path((ROOT/'final_concentrated/latest-payload-review.txt').read_text().strip())/'RESULT.json').read_text())
  if actual.get('pass') is not True:raise RuntimeError('initial_real_payload_path_failed')
  actual2=json.loads((ROOT/'calibration_runs/C-20261017T102733-3d3e3a/held-cycle.json').read_text())[0]['request']['motion_plan_request']['start_state'];remove_char_operations(actual2)
  old_att=actual2['attached_collision_objects'][0]
  from moveit_msgs.msg import AttachedCollisionObject
  arm2=AttachedCollisionObject();set_message_fields(arm2,old_att);q2=dict(zip(actual2['joint_state']['name'],actual2['joint_state']['position']))
  M=np.linalg.inv(fk('arm2_gripper_base',q2))@fk(arm2.link_name,q2)@pose_to_mat(arm2.object.pose)@pose_to_mat(arm2.object.primitive_poses[0]);arm2.link_name='arm2_gripper_base';arm2.object.header.frame_id=arm2.link_name;arm2.object.pose=Pose();arm2.object.pose.orientation.w=1.;arm2.object.primitive_poses=[mat_to_pose(M)];arm2.touch_links=['arm2_gripper_finger_left_link','arm2_gripper_finger_right_link']
  arm1=AttachedCollisionObject();data=actual['reconstructed_case']['held'];remove_char_operations(data);set_message_fields(arm1,data)
  stored=[]
  for row in selected:
   data=copy.deepcopy(row['request']);remove_char_operations(data);rq=GetMotionPlan.Request();set_message_fields(rq,data)
   stage=row['stage'];bn=row['slot'].split('/')[0]
   if stage not in ('rise','return'):rq.motion_plan_request.start_state.attached_collision_objects=[copy.deepcopy(arm1 if bn=='bin_yellow' else arm2)]
   # Match candidate runtime LIN down/up; PTP carry/standby unchanged.
   if stage in ('settle_top','settle_bottom','rise'):rq.motion_plan_request.planner_id='LIN'
   scene=ApplyPlanningScene.Request();scene.scene.is_diff=True;scene.scene.robot_state.is_diff=True
   scene.scene.world.collision_objects=collision_objects(WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf')
   for oldobj in stored:
    removed=copy.deepcopy(oldobj);removed.operation=CollisionObject.REMOVE;scene.scene.world.collision_objects.append(removed)
   stored=[box('stored_'+s['id'],s['xyz']) for s in cfg['bins'][bn]['slots'] if s['id']!=row['slot'] or stage in ('rise','return')]
   scene.scene.world.collision_objects += stored
   if not wait(apply.call_async(scene)).success:raise RuntimeError('scene_not_acknowledged')
   response=wait(plan.call_async(rq));requestlog.append({'slot':row['slot'],'stage':stage,'request':msgdict(rq),'response':msgdict(response)})
   rec={'slot':row['slot'],'stage':stage,'planner':rq.motion_plan_request.planner_id,'code':response.motion_plan_response.error_code.val,'samples':0,'invalid':[]};result['segments'].append(rec)
   if rec['code']!=1:
    endcheck=GetStateValidity.Request();endcheck.robot_state=copy.deepcopy(rq.motion_plan_request.start_state);goalmap={x.joint_name:x.position for x in rq.motion_plan_request.goal_constraints[0].joint_constraints};vmap=dict(zip(endcheck.robot_state.joint_state.name,endcheck.robot_state.joint_state.position));vmap.update(goalmap);endcheck.robot_state.joint_state.position=[vmap[n] for n in endcheck.robot_state.joint_state.name];rec['goal_validity']=msgdict(wait(valid.call_async(endcheck)))
    print(rec,flush=True);continue
   start=dict(zip(rq.motion_plan_request.start_state.joint_state.name,rq.motion_plan_request.start_state.joint_state.position));tr=response.motion_plan_response.trajectory.joint_trajectory
   for sample_t,sample_joints in sample_trajectory(tr,start):
    check=GetStateValidity.Request();check.robot_state=copy.deepcopy(rq.motion_plan_request.start_state)
    check.robot_state.joint_state.position=[sample_joints[x] for x in check.robot_state.joint_state.name]
    ans=wait(valid.call_async(check));rec['samples']+=1
    if not ans.valid:rec['invalid'].append({'t':sample_t,'result':msgdict(ans)});break
   if stage=='rise':
    end=dict(start);end.update(zip(tr.joint_names,tr.points[-1].positions));rec['exit_bottoms']=gripper_bottoms(urdf,end,'' if bn=='bin_yellow' else 'arm2_');rec['above_bin']=min(rec['exit_bottoms'].values())>.815
   print({k:v for k,v in rec.items() if k!='invalid'},flush=True)
  files=[WS/'src/a1_sorting_core/config/four_slot_candidate.json',WS/'src/a1_cell_sim/worlds/four_slot_1ms.sdf',ROOT/'audit/robot-static.urdf',WS/'src/a1_sorting_core/a1_sorting_core/pick_place_executor_node.py',WS/'src/a1_sorting_core/a1_sorting_core/motion_client.py']
  result['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};result['pickup_to_carry_payload_scene_verified']=actual.get('pass') is True;result['real_attachment_sources']=[str(ROOT/'final_concentrated/latest-payload-review.txt'),str(ROOT/'calibration_runs/C-20261017T102733-3d3e3a/held-cycle.json')];result['pass']=bool(result['segments']) and all(r['code']==1 and r['samples']>0 and not r['invalid'] and r.get('above_bin',True) for r in result['segments'])
  (out/'requests.json').write_text(json.dumps(requestlog))
 except Exception as e:result['error']=repr(e)
 finally:
  ex.shutdown(timeout_sec=2);thread.join(2);n.destroy_node();rclpy.try_shutdown();stop_group(server);log.close()
  (out/'RESULT.json').write_text(json.dumps(result,indent=2));(ROOT/'final_concentrated/latest-all-path-review.txt').write_text(str(out))
if __name__=='__main__':main()
