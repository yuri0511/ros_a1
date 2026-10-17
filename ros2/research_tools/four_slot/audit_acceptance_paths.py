"""Selected real Pilz requests plus <=1ms FCL replay. No Gazebo or execution."""
import json,sys,time,copy,subprocess,os,threading,hashlib
from pathlib import Path
import rclpy
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
 out=ROOT/'acceptance_preparation'/('paths-'+time.strftime('%Y%m%dT%H%M%S'));out.mkdir();print(out,flush=True)
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
  old=json.loads((ROOT/'audit/stage2/plans.json').read_text());selected=[r for r in old if r['slot'] in ('bin_yellow/slot_1','bin_red/slot_4')]
  stored=[]
  for row in selected:
   data=copy.deepcopy(row['request']);remove_char_operations(data);rq=GetMotionPlan.Request();set_message_fields(rq,data)
   stage=row['stage'];bn=row['slot'].split('/')[0]
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
  result['sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};result['pass']=bool(result['segments']) and all(r['code']==1 and r['samples']>0 and not r['invalid'] and r.get('above_bin',True) for r in result['segments'])
  (out/'requests.json').write_text(json.dumps(requestlog))
 except Exception as e:result['error']=repr(e)
 finally:
  ex.shutdown(timeout_sec=2);thread.join(2);n.destroy_node();rclpy.try_shutdown();stop_group(server);log.close()
  (out/'RESULT.json').write_text(json.dumps(result,indent=2));(ROOT/'acceptance_preparation/latest-path-review.txt').write_text(str(out))
if __name__=='__main__':main()
