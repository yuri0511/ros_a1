"""A1 工作站仿真层 bringup(不含 MoveIt 与任务节点)。

机械臂与控制复用官方 ur_simulation_gz/launch/ur_sim_control.launch.py,
通过参数注入本包的工作站 xacro(UR5e+立柱+夹爪)、控制器配置与
sorting_cell 世界;再补齐官方未覆盖的部分:相机/位姿/附着服务桥接、
夹爪控制器加载、来料工件生成。

dual:=true 时切换为双臂整机描述(ur5e_cell_dual.urdf.xacro,arm2 带
arm2_ 前缀,与 arm1 同款低肩立柱镜像,装在工作台南缘外 (0.60,-0.47)),
并追加第二组控制器 spawner。

gpu_offload:=true(默认)时注入 NVIDIA PRIME offload 环境变量,让 gz 的
GUI 与传感器渲染落在独显上。背景:本机 prime 为 on-demand 模式,Mesa EGL
枚举到独显 DRM 节点但没有对应 DRI 驱动(日志报 `pci id ... 10de:28e0,
driver (null)` / `failed to create dri2 screen`),会退化到 llvmpipe 软件
光栅,空场景 GUI 也烧 250%+ CPU。offload 三变量使 OGRE2/Qt 走
libGLX_nvidia/nvidia EGL,实测 nvidia-smi 可见 gzserver/gz-gui 的显存占用。
若以后整机 `prime-select nvidia`,这些变量是无害的空操作。

由 a1_sorting_core/launch/sorting_demo.launch.py 包含调用,也可单独启动。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (IfElseSubstitution, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

CELL_SHARE = FindPackageShare('a1_cell_sim')


def _bridge_args() -> str:
    """官方 bringup 未覆盖的桥接:相机、逻辑相机、抓取附着服务、传送带速度。

    注意:
    - 话题桥用 `@ros_type[gz_type` 方向语法;服务桥必须用
      `@ros_srv_type@gz_req_type@gz_rep_type` 三段式(不接受 `[` 方向符);
    - gz-sim 8 中相机图像发布在 `<topic>` 原样话题上(本世界为
      /overhead_camera),camera_info 在 /camera_info,与常见约定不同;
    - 逻辑相机提供工件位姿(连续发布,不依赖渲染);
    - 传送带速度是 ROS->GZ 单向(`]` 方向符):任务层发布 Float64,
      gz-sim TrackController 订阅 gz.msgs.Double(契约 docs/pipeline_contract.md §4)。
    """
    return ','.join([
        '/overhead_camera@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/overhead_logical@ros_gz_interfaces/msg/LogicalCameraImage'
        '[gz.msgs.LogicalCameraImage',
        '/world/sorting_cell/dynamic_pose/info@tf2_msgs/msg/TFMessage'
        '[gz.msgs.Pose_V',
        '/world/sorting_cell/create@ros_gz_interfaces/srv/SpawnEntity'
        '@gz.msgs.EntityFactory@gz.msgs.Boolean',
        '/world/sorting_cell/remove@ros_gz_interfaces/srv/DeleteEntity'
        '@gz.msgs.Entity@gz.msgs.Boolean',
        '/world/sorting_cell/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        '@gz.msgs.Pose@gz.msgs.Boolean',
        '/model/pusher/joint/pusher_slide/cmd_vel@std_msgs/msg/Float64'
        ']gz.msgs.Double',
    ])


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type')
    part_colors = LaunchConfiguration('part_colors')
    random_colors = LaunchConfiguration('random_colors')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    dual = LaunchConfiguration('dual')
    spawn_mode = LaunchConfiguration('spawn_mode')
    # 世界文件选择(默认 sorting_cell = 布局 v2 四色四箱主线世界;
    # 旧判别实验变体 sorting_cell_binC.sdf 已随 v2 删除,历史可考
    # 64cd236/3f02c2b —— 四箱直接进主线,不再需要变体隔离)
    world_file = LaunchConfiguration('world_file')
    controllers_file = LaunchConfiguration('controllers_file')

    # 0) NVIDIA PRIME offload 环境(默认开,见模块 docstring)。
    #    必须排在 ur_control include 之前:launch 的环境变量按设置顺序
    #    被之后 spawn 的进程继承,gzserver/gz-gui 都在 include 里拉起。
    #    取值方式与 sorting_demo 的 pipeline_on 一致:此处直接求值,
    #    不依赖 SetEnvironmentVariable 对 condition 的支持(版本行为不一)。
    from launch.utilities import perform_substitutions
    offload_on = perform_substitutions(
        context, [LaunchConfiguration('gpu_offload')]
    ).lower() in ('1', 'true', 'yes')
    offload_env = [
        SetEnvironmentVariable(name=var, value=val)
        for var, val in (
            ('__NV_PRIME_RENDER_OFFLOAD', '1'),
            ('__NV_PRIME_RENDER_OFFLOAD_PROVIDER', 'NVIDIA-G0'),
            ('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        )
    ] if offload_on else []

    # 1) 官方 Gazebo+UR 控制启动,注入本包的自定义文件
    #    (dual 模式换双臂整机 xacro;官方 launch 固定以 name:=ur 渲染,
    #     双臂两台 UR 同属一个 "ur" 模型,arm2 靠 tf_prefix 区分)
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ur_simulation_gz'), 'launch',
             'ur_sim_control.launch.py'])),
        launch_arguments={
            'ur_type': ur_type,
            'description_file': IfElseSubstitution(
                dual,
                PathJoinSubstitution(
                    [CELL_SHARE, 'urdf', 'ur5e_cell_dual.urdf.xacro']),
                PathJoinSubstitution(
                    [CELL_SHARE, 'urdf', 'ur5e_cell.urdf.xacro'])),
            'controllers_file': PathJoinSubstitution(
                [CELL_SHARE, 'config', controllers_file]),
            'world_file': PathJoinSubstitution(
                [CELL_SHARE, 'worlds', world_file]),
            'launch_rviz': 'false',
            'gazebo_gui': gazebo_gui,
            'initial_joint_controller': 'joint_trajectory_controller',
        }.items())

    # 2) 夹爪控制器 + (双臂时)第二组臂/夹爪控制器(等 controller_manager 起来)
    gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen')
    arm2_jtc_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm2_joint_trajectory_controller', '-c',
                   '/controller_manager'],
        condition=IfCondition(dual), output='screen')
    arm2_gripper_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm2_gripper_controller', '-c', '/controller_manager'],
        condition=IfCondition(dual), output='screen')
    # 官方 jsb spawner 进程启动 8 秒后再加载夹爪/第二组控制器
    delayed_grippers = TimerAction(
        period=8.0, actions=[gripper_spawner, arm2_jtc_spawner,
                             arm2_gripper_spawner])

    # 3) 桥接(相机/位姿/附着服务)。注意 parameter_bridge 需要分立的参数,
    #    不能把多个话题合并进一个逗号字符串(会打印用法并退出)。
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=_bridge_args().split(','),
        parameters=[{'use_sim_time': True}],
        output='screen')

    # 4) 来料工件(等世界与桥接就绪;仿真以 -r 启动,15s 后世界已就绪)。
    #    spawn_mode=batch:一次性三件后退出(legacy);service:常驻节点按
    #    /parts/request_spawn 请求逐件生成(pipeline,契约 §5)
    spawn_parts = Node(
        package='a1_cell_sim', executable='spawn_parts.py',
        parameters=[{'part_colors': part_colors,
                     'random_colors': random_colors,
                     'world_name': 'sorting_cell',
                     'mode': spawn_mode}],
        output='screen')
    delayed_parts = TimerAction(period=15.0, actions=[spawn_parts])

    return [*offload_env, ur_control, delayed_grippers, bridge,
            delayed_parts]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('part_colors', default_value='red,green,red'),
        DeclareLaunchArgument(
            'random_colors', default_value='',
            description='service 模式 random 色池覆盖(空=用 spawn_parts 内置'
                        '四色池;判别实验可收窄,如 red,green)'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('dual', default_value='false',
                              description='双臂接力模式(第二台 UR5e)'),
        DeclareLaunchArgument('spawn_mode', default_value='batch',
                              description='来料模式 batch(legacy)|service(pipeline)'),
        DeclareLaunchArgument(
            'controllers_file', default_value='ur5e_cell_controllers.yaml',
            description='控制器配置文件名(share/a1_cell_sim/config 下)。默认'
                        '=定案 gain 2.0;判别实验变体如 '
                        'ur5e_cell_controllers_gain10.yaml(P4 G2 档)'),
        DeclareLaunchArgument(
            'world_file', default_value='sorting_cell.sdf',
            description='世界文件(share/a1_cell_sim/worlds 下的文件名)。默认 '
                        'sorting_cell.sdf = **布局 v2 四色四箱主线世界**'
                        '(world name 恒为 sorting_cell 以保证 ros_gz bridge '
                        '话题不变;旧 binC 判别变体已删,历史可考 '
                        '64cd236/3f02c2b)'),
        DeclareLaunchArgument(
            'gpu_offload', default_value='true',
            description='true 时把 gz 渲染 offload 到 NVIDIA 独显'
                        '(PRIME 环境变量;on-demand 模式下不加会退 llvmpipe 软渲染)'),
        OpaqueFunction(function=launch_setup),
    ])
