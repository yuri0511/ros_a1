"""Isolated planning server: no Gazebo, hardware, executor or trajectory execution."""
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    cell = Path(get_package_share_directory('a1_cell_sim'))
    config = (MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        .robot_description_semantic(str(cell/'srdf/a1_cell_dual.srdf.xacro'), {'name':'ur'})
        .robot_description_kinematics(str(cell/'config/kinematics_dual.yaml'))
        .planning_pipelines(default_planning_pipeline='pilz_industrial_motion_planner')
        .joint_limits(str(cell/'config/joint_limits_dual.yaml'))
        .trajectory_execution(str(cell/'config/moveit_controllers_dual.yaml'))
        .to_moveit_configs())
    robot = (Path(__file__).resolve().parents[4]/'audit/robot-static.urdf').read_text()
    return LaunchDescription([Node(package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=[config.to_dict(), {'robot_description':robot,
        'use_sim_time':False, 'allow_trajectory_execution':False,
        'publish_robot_description':True, 'publish_robot_description_semantic':True}])])
