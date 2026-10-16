"""Original cell/MoveIt/collision scene; no dispatcher, no part requests."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 root=FindPackageShare('a1_cell_sim')
 def include(name,args):return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([root,'launch',name])),launch_arguments=args.items())
 return LaunchDescription([
  DeclareLaunchArgument('controllers_file'),DeclareLaunchArgument('trace_world',default_value='control_stability_prepost_1ms.sdf'),
  include('cell_bringup.launch.py',{'dual':'true','gpu_offload':'false','gazebo_gui':'false','spawn_mode':'service','world_file':LaunchConfiguration('trace_world'),'controllers_file':LaunchConfiguration('controllers_file'),'fast_sim_waits':'true'}),
  include('cell_moveit.launch.py',{'dual':'true','launch_rviz':'false','use_sim_time':'true','ur_type':'ur5e'}),
  Node(package='a1_sorting_core',executable='scene_publisher_node',name='scene_publisher',parameters=[{'use_sim_time':True}],output='screen')])
