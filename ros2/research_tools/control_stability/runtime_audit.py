"""Read-only ROS parameters, controller and hardware interface audit."""
import rclpy,time,json,sys
from pathlib import Path
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.parameter import parameter_value_to_python
rclpy.init();n=rclpy.create_node('control_parameter_audit');dest=Path(sys.argv[1]);deadline=time.monotonic()+50

def call(service,cls,request):
 c=n.create_client(cls,service)
 while not c.wait_for_service(timeout_sec=.2):
  if time.monotonic()>deadline:raise TimeoutError(service)
 f=c.call_async(request)
 while not f.done():
  rclpy.spin_once(n,timeout_sec=.05)
  if time.monotonic()>deadline:raise TimeoutError(service)
 return f.result()
try:
 while True:
  controllers=call('/controller_manager/list_controllers',ListControllers,ListControllers.Request())
  active={x.name for x in controllers.controller if x.state=='active'}
  if {'joint_trajectory_controller','arm2_joint_trajectory_controller','joint_state_broadcaster'}<=active:break
  if time.monotonic()>deadline:raise TimeoutError('active controllers')
  time.sleep(.2)
 out={'controllers':message_to_ordereddict(controllers),'hardware_interfaces':message_to_ordereddict(call('/controller_manager/list_hardware_interfaces',ListHardwareInterfaces,ListHardwareInterfaces.Request())),'parameters':{}}
 for node,params in [('/gz_ros_control',['position_proportional_gain','hold_joints']),('/controller_manager',['update_rate','use_sim_time']),('/joint_trajectory_controller',['command_interfaces','state_interfaces','open_loop','open_loop_control','interpolate_from_desired_state','constraints.stopped_velocity_tolerance','constraints.goal_time']),('/arm2_joint_trajectory_controller',['command_interfaces','state_interfaces','open_loop','open_loop_control','constraints.goal_time'])]:
  req=GetParameters.Request();req.names=params;v=call(node+'/get_parameters',GetParameters,req)
  out['parameters'][node]={k:parameter_value_to_python(x) for k,x in zip(params,v.values)}
 out['wall']=time.time();dest.write_text(json.dumps(out,indent=2));print(json.dumps(out['parameters'],indent=2))
finally:n.destroy_node();rclpy.shutdown()
