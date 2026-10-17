"""Replay one saved physical state into isolated MoveIt; no Gazebo/action clients."""
import json,os,subprocess,sys,time
from pathlib import Path
import rclpy
from moveit_msgs.srv import ApplyPlanningScene,GetStateValidity
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(HERE.parent/'voice_four'))
from run_four import stop_group

def remove_char_operations(x):
 if isinstance(x,dict):
  for k in list(x):
   if k=='operation':x.pop(k)
   else:remove_char_operations(x[k])
 elif isinstance(x,list):
  for v in x:remove_char_operations(v)

def main():
 run=Path(sys.argv[1]);os.environ['ROS_DOMAIN_ID']='96';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST';os.environ['ROS_LOG_DIR']=str(run/'replay-ros')
 log=(run/'held-replay-planner.log').open('w');server=subprocess.Popen(['ros2','launch',str(HERE/'plan_only.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();n=rclpy.create_node('held_state_replay');apply=n.create_client(ApplyPlanningScene,'/apply_planning_scene');valid=n.create_client(GetStateValidity,'/check_state_validity')
 def call(c,q):
  if not c.wait_for_service(timeout_sec=15):raise RuntimeError('service missing')
  f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
  if not f.done():raise RuntimeError('RPC timeout')
  return f.result()
 try:
  scene=json.loads((ROOT/'audit/stage2/planning-scene.json').read_text());remove_char_operations(scene)
  q=ApplyPlanningScene.Request();set_message_fields(q,scene)
  for o in q.scene.world.collision_objects:o.operation=o.ADD
  assert call(apply,q).success
  state=json.loads((run/'held-cycle.json').read_text())[0]['request']['motion_plan_request']['start_state'];remove_char_operations(state)
  q=GetStateValidity.Request();set_message_fields(q.robot_state,state)
  for o in q.robot_state.attached_collision_objects:o.object.operation=o.object.ADD
  answer=call(valid,q);(run/'held-state-validity.json').write_text(json.dumps(message_to_ordereddict(answer),indent=2));print(message_to_ordereddict(answer))
 finally:
  n.destroy_node();rclpy.try_shutdown();stop_group(server);log.close()
if __name__=='__main__':main()
