"""A1 工作站 MoveIt bringup(move_group + RViz)。

以官方 ur_moveit_config/launch/ur_moveit.launch.py 为蓝本,替换两处:
1. SRDF 用本包的 srdf/a1_cell.srdf.xacro —— 官方 SRDF 不认识自制
   夹爪/立柱,规划首条轨迹即被自碰撞检查否决(详见该文件头注释);
2. 控制器配置用本包的 config/moveit_controllers.yaml —— 只登记
   实际激活的 joint_trajectory_controller。

dual:=true 时 SRDF/运动学/控制器配置切换为双臂版(a1_cell_dual.srdf.xacro
+ kinematics_dual.yaml + moveit_controllers_dual.yaml):单一 move_group
同时管理 ur_manipulator(arm1)与 arm2_ur_manipulator(arm2)两个规划组。

robot_description 不作为参数传入,move_group 侧从 /robot_description
话题获取(与官方一致),故先启动 wait_for_robot_description 再拉起
move_group。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, \
    RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    dual = LaunchConfiguration('dual').perform(context) == 'true'

    cell_share = get_package_share_directory('a1_cell_sim')
    srdf_abs = os.path.join(cell_share, 'srdf',
                            'a1_cell_dual.srdf.xacro' if dual
                            else 'a1_cell.srdf.xacro')
    controllers_abs = os.path.join(cell_share, 'config',
                                   'moveit_controllers_dual.yaml' if dual
                                   else 'moveit_controllers.yaml')
    kinematics_abs = os.path.join(cell_share, 'config',
                                  'kinematics_dual.yaml' if dual
                                  else 'kinematics.yaml')
    joint_limits_abs = os.path.join(cell_share, 'config',
                                    'joint_limits_dual.yaml')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        .robot_description_semantic(srdf_abs, {'name': 'ur'})
        .robot_description_kinematics(kinematics_abs)
        .planning_pipelines(default_planning_pipeline='pilz_industrial_motion_planner')
        .trajectory_execution(controllers_abs)
    )
    if dual:
        # 官方包的 joint_limits.yaml 只有无前缀关节,arm2 组会缺加速度限值
        # (Pilz 拒绝规划),双臂模式显式加载两臂限值
        moveit_config = moveit_config.joint_limits(joint_limits_abs)
    moveit_config = moveit_config.to_moveit_configs()

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
    }

    wait_robot_description = Node(
        package='ur_robot_driver', executable='wait_for_robot_description',
        output='screen')

    move_group_node = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), warehouse_ros_config,
                    {'use_sim_time': use_sim_time,
                     'publish_robot_description_semantic': True}])

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur_moveit_config'), 'config', 'moveit.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_moveit',
        condition=IfCondition(launch_rviz), output='log',
        arguments=['-d', rviz_config_file],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    warehouse_ros_config,
                    {'use_sim_time': use_sim_time}])

    return [wait_robot_description,
            RegisterEventHandler(OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('dual', default_value='false',
                              description='双臂接力模式(MoveIt 双规划组)'),
        OpaqueFunction(function=launch_setup),
    ])
