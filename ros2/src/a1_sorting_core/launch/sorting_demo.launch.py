"""A1 一键演示启动:工作站仿真 + MoveIt + 任务/记录/视觉节点。

用法(构建后):
    # 裸命令(2026-10-08 起默认档)= v2 双臂并联分色 8 件一轮:
    # 四色随机来料,红绿→arm2 南排、蓝黄→arm1 北排,GUI 弹出
    ros2 launch a1_sorting_core sorting_demo.launch.py
    # 自定义来料颜色(条件变化演示):
    ros2 launch a1_sorting_core sorting_demo.launch.py part_colors:="red,green,blue"
    # 单臂流水线(显式关双臂):自动收窄色池 blue,yellow
    ros2 launch a1_sorting_core sorting_demo.launch.py dual_arm:=false
    # 经典接力(任务1拓展,arm1 分拣 -> arm2 码放 bin_out):显式给空色池
    ros2 launch a1_sorting_core sorting_demo.launch.py arm2_colors:=""
    # legacy 一次性 batch 三件(回归基线,自动单臂):
    ros2 launch a1_sorting_core sorting_demo.launch.py pipeline:=false

自动档规则(dual_arm/random_colors/arm2_colors 留空时):
  dual_arm   = pipeline ? true : false(legacy 恒单臂,旧基线零差)
  arm2_colors= 双臂且未显式给 → red,green(v2 并联分色定案)
  random_colors= 双臂 → 四色全上;单臂 pipeline → blue,yellow
    (arm1 只服务北排两箱,四色裸跑红绿必 unknown_category FAILED)
demo_parts 默认 8(0=无限持续)。pipeline:=true 时追加加载
config/sorting_params_pipeline.yaml(后加载覆盖同名参数;legacy 默认值
不动),并把 spawn 模式切为 service(按需单件来料)。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration('ur_type')
    part_colors = LaunchConfiguration('part_colors')
    random_colors = LaunchConfiguration('random_colors')
    arm2_colors_cfg = LaunchConfiguration('arm2_colors')
    inject_failure_parts = LaunchConfiguration('inject_failure_parts')
    dual_arm = LaunchConfiguration('dual_arm')
    pipeline = LaunchConfiguration('pipeline')
    gpu_offload = LaunchConfiguration('gpu_offload')
    demo_parts = LaunchConfiguration('demo_parts')
    world_file = LaunchConfiguration('world_file')
    controllers_file = LaunchConfiguration('controllers_file')

    # 1) 仿真层(cell_bringup,含官方 ur_sim_control;dual 时为双臂整机;
    #    pipeline 时 spawn 模式切 service:常驻节点按 /parts/request_spawn
    #    逐件生成随机染色工件到带尾)
    from launch.utilities import perform_substitutions  # 局部导入避免顶部依赖
    # perform_substitutions 返回 join 后的单个字符串(不是列表)
    pipeline_on = perform_substitutions(
        context, [pipeline]).lower() in ('1', 'true', 'yes')
    # dual_arm 自动档(2026-10-08):auto = pipeline?true:false——裸命令即
    # v2 双臂并联演示;legacy(pipeline:=false)保持单臂 batch 零差。
    # 显式 true/false 覆盖自动值。dual_eff 为解析后的确定字符串,
    # 下方所有 dual 引用一律用它(Include/Condition/ParameterValue)。
    dual_raw = perform_substitutions(context, [dual_arm]).strip().lower()
    if dual_raw in ('', 'auto'):
        dual_eff = 'true' if pipeline_on else 'false'
    else:
        dual_eff = dual_raw
    dual_on_eff = dual_eff in ('1', 'true', 'yes')
    # 色池自动档(2026-10-08):random_colors 未给(auto) = 双臂时四色
    # 全上(红绿→arm2 南排/蓝黄→arm1 北排,按色路由两臂都有活干);单臂时
    # 自动收窄 blue,yellow——arm1 只服务北排两箱,四色裸跑会把红绿派给
    # arm1 判 unknown_category FAILED(README 旧警示的根治)。
    # 显式传空串 = 旧逃生门(spawn_parts 内置四色池)。
    random_raw = perform_substitutions(context, [random_colors]).strip()
    if random_raw.lower() in ('auto',):
        random_eff = 'blue,yellow,red,green' if dual_on_eff \
            else 'blue,yellow'
    else:
        random_eff = random_raw
    # arm2_colors 自动档:auto(未给)且双臂+pipeline → red,green(v2 定案
    # 并联);显式空串 = 经典接力逃生门(不填)。
    arm2_raw = perform_substitutions(context, [arm2_colors_cfg]).strip()
    if arm2_raw.lower() == 'auto':
        arm2_raw = 'red,green' if (dual_on_eff and pipeline_on) else ''
    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('a1_cell_sim'), 'launch',
             'cell_bringup.launch.py'])),
        launch_arguments={'ur_type': ur_type,
                          'part_colors': part_colors,
                          'random_colors': random_eff,
                          'dual': dual_eff,
                          'world_file': world_file,
                          'controllers_file': controllers_file,
                          'gpu_offload': gpu_offload,
                          'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                          'fast_sim_waits': LaunchConfiguration('fast_sim_waits'),
                          'spawn_mode': 'service' if pipeline_on
                                        else 'batch'}.items())

    # 2) MoveIt(本包自制 SRDF/控制器配置,见 cell_moveit.launch.py 头注释;
    #    等 /robot_description 话题出现后启动 move_group 与 RViz;
    #    dual 时单一 move_group 管 ur_manipulator + arm2_ur_manipulator 双组)
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('a1_cell_sim'), 'launch',
             'cell_moveit.launch.py'])),
        launch_arguments={'ur_type': ur_type,
                          'use_sim_time': 'true',
                          'launch_rviz': LaunchConfiguration('launch_rviz'),
                          'dual': dual_eff}.items())

    # 2b) 单变量判别实验透传(空=不覆盖,维持 yaml 定值;设置则在本
    #     Node 参数表尾追加覆盖,后加载覆盖前者)。Phase 0 抓取首败 EG 序列用:
    #     tcp_z_offset 减小=指尖更低/包覆更深(0.204 定档;URDF 合法域
    #     [0.201,0.231],勿低于 0.201);belt_settle 覆盖 dispatcher 的
    #     belt_settle_delay(定值 2.6)
    tcp_z_override = perform_substitutions(
        context, [LaunchConfiguration('tcp_z_offset')]).strip()
    belt_settle_override = perform_substitutions(
        context, [LaunchConfiguration('belt_settle')]).strip()
    executor_overrides = ({'tcp_z_offset': float(tcp_z_override)}
                          if tcp_z_override else {})
    dispatcher_overrides = ({'belt_settle_delay': float(belt_settle_override)}
                            if belt_settle_override else {})
    # 语言指挥模式(2026-10-06):auto_start:=false ⇒ 来料只登记不入队,
    # 完全由 /sort_task 服务(dispatch_single/dispatch_to,语音节点或手敲)
    # 驱动派发;默认 true 与 yaml 定值一致(legacy/pipeline 验收零差)
    auto_start = LaunchConfiguration('auto_start')

    # 2b') R1-P1/P3 楔死判死开关(2026-10-04 终审;默认 false=全链零行为差)
    # wedge:=true ⇒ 执行器判死即停/不重试/收尾跳过 + 调度器停轮停来料;
    # wedge_rearm:=true ⇒ 判死后下一件重建 MotionClient(配外层受控重启
    # move_group);tool_settle_fix:=true ⇒ 启用笛卡尔腿到位闸 latent bug
    # 修正(单独开关,默认关=保留旧时序,arm1 定案态零差)。
    wedge_on = perform_substitutions(
        context, [LaunchConfiguration('wedge')]).lower() in ('1', 'true', 'yes')
    rearm_on = perform_substitutions(
        context, [LaunchConfiguration('wedge_rearm')]).lower() in (
        '1', 'true', 'yes')
    settle_fix_on = perform_substitutions(
        context, [LaunchConfiguration('tool_settle_fix')]).lower() in (
        '1', 'true', 'yes')
    if wedge_on:
        motion_flags = {'wedge_detect': True,
                        'wedge_rearm': rearm_on,
                        'tool_settle_fix': settle_fix_on}
        executor_overrides.update(motion_flags)
        dispatcher_overrides['halt_on_wedge'] = True
    elif settle_fix_on:
        motion_flags = {'tool_settle_fix': True}
        executor_overrides['tool_settle_fix'] = True
    else:
        motion_flags = {}

    # 2c) 双臂并联分色(布局 v2,2026-10-04 G1):arm2_colors 非空时
    #     - dispatcher 按色路由:路由色(red,green)直接派 arm2,arm2 从带尾
    #       取件投南排 bin_red/bin_green;接力语义(relay_mode)关闭;
    #     - **不再覆盖 arm1 standby**:旧"arm1 停泊让位 +y 侧"约束随 v2 失效
    #       (旧高肩 arm2 伸带尾 (0.36,0) 时与 arm1 旧待机位 wrist_2 自碰撞,
    #       E0 实证需让位)。v2 两臂分居南北桌缘外 + standby 泊位方位错开
    #       (arm1 朝 +x、arm2 朝 -x),G0 互探实测 standby×standby
    #       +230.6mm、动臂锚×泊臂 standby 最紧 +230.6mm ≫ 8mm 判据,
    #       无需让位;arm1 standby 用 yaml 定档值(与 xacro 出生位同值)。
    #     - recorder expect_relay 关闭:arm1/arm2 的 PLACED 都直接回收
    #       (并联没有"arm1 中间态"语义)。
    dual_on = dual_on_eff
    arm2_colors = arm2_raw   # 已含自动档(auto 且双臂+pipeline → red,green)
    parallel_on = dual_on and bool(arm2_colors)
    if parallel_on:
        dispatcher_overrides['arm2_colors'] = arm2_colors
    # 布局 v2 料区名清单(台账用):arm1 北排 / arm2 南排 / 四箱全集
    north_bins = ['bin_blue', 'bin_yellow']
    south_bins = ['bin_red', 'bin_green']
    all_bins = north_bins + south_bins

    # 3) 任务层节点(共享参数文件;执行器通过 MoveGroup 动作与
    #    move_group 交互,无需在本进程加载 MoveIt 规划配置)。
    #    pipeline:=true 时追加 pipeline 覆盖 yaml(参数列表顺序 = 后加载
    #    覆盖前者;legacy 默认值不动)
    core_params = PathJoinSubstitution(
        [FindPackageShare('a1_sorting_core'), 'config', 'sorting_params.yaml'])
    pipeline_params_path = PathJoinSubstitution(
        [FindPackageShare('a1_sorting_core'), 'config',
         'sorting_params_pipeline.yaml'])
    core_params_list = [core_params] + ([pipeline_params_path]
                                        if pipeline_on else [])
    # relay_overlay:arm2(/pick_place_relay)专用单变量覆盖 yaml 的**绝对路径**。
    # 空=不覆盖(D1 之前的 64cd236 基线档)。z 阶梯各档写一份只含
    # /pick_place_relay 段的 yaml,由 run_experiment.py --relay-overlay 传入,
    # 做到"一次只动一个变量、且不改动库内定值文件"。
    relay_overlay_path = perform_substitutions(
        context, [LaunchConfiguration('relay_overlay')]).strip()
    relay_params_list = core_params_list + (
        [relay_overlay_path] if relay_overlay_path else [])
    dispatcher = Node(
        package='a1_sorting_core', executable='task_dispatcher_node',
        name='task_dispatcher', output='screen',
        parameters=[*core_params_list,
                    {'use_sim_time': True, 'fast_sim_waits': ParameterValue(LaunchConfiguration('fast_sim_waits'), value_type=bool),
                     # 双臂接力:dual_eff 已解析为确定字符串,这里直接给
                     # Python bool(旧代码传 LaunchConfiguration+value_type=
                     # bool 由 launch 延迟转型;字符串字面量会被当场拒收)
                     'relay_mode': dual_on_eff,
                     # 演示来料计划:0=无限持续(默认);>0 一轮 N 件
                     'demand_parts': ParameterValue(demo_parts,
                                                    value_type=int),
                     # 语言指挥模式(2026-10-06):默认 true=来料即入队
                     # (旧行为);false=只登记,派发完全由 /sort_task
                     # 服务(语音节点)驱动
                     'auto_start': ParameterValue(auto_start,
                                                  value_type=bool),
                     **dispatcher_overrides}])
    executor_node = Node(
        package='a1_sorting_core', executable='pick_place_executor_node',
        name='pick_place_executor', output='screen',
        parameters=[*core_params_list, {'use_sim_time': True, 'fast_sim_waits': ParameterValue(LaunchConfiguration('fast_sim_waits'), value_type=bool),
                                         'inject_failure_parts':
                                             inject_failure_parts,
                                         **executor_overrides}])
    # 3b) arm2 执行节点(同一可执行文件,参数区分:订阅 /task/command_arm2、
    #     取件位姿用逻辑相机真值;布局 v2 = 南排红/绿独立分拣臂,定值见
    #     sorting_params_pipeline.yaml 的 /pick_place_relay 段)
    relay_node = Node(
        package='a1_sorting_core', executable='pick_place_executor_node',
        name='pick_place_relay', output='screen',
        condition=IfCondition(dual_eff),
        # 楔死三开关同步给 arm2(D2b 实验对象就是 arm2);tcp_z 等臂1
        # 实验覆盖不传(与 64cd236 口径一致,relay 用自己的 yaml 段)
        parameters=[*relay_params_list,
                    {'use_sim_time': True, 'fast_sim_waits': ParameterValue(LaunchConfiguration('fast_sim_waits'), value_type=bool), **motion_flags}])
    # 3c) 静态规划场景(仅 pipeline):世界几何对规划器不可见会盲规划穿障
    scene_pub = Node(
        package='a1_sorting_core', executable='scene_publisher_node',
        name='scene_publisher', output='screen',
        condition=IfCondition(pipeline),
        parameters=[{'use_sim_time': True}])
    recorder = Node(
        package='a1_sorting_core', executable='result_recorder_node',
        name='result_recorder', output='screen',
        parameters=[*core_params_list,
                    {'use_sim_time': True, 'fast_sim_waits': ParameterValue(LaunchConfiguration('fast_sim_waits'), value_type=bool),
                     # ---- 布局 v2 台账料区(四箱,参数化;不再硬编码三箱)----
                     # 单臂/legacy:arm1 只服务北排 => 只计北排两箱;
                     # 双臂(dual_arm):四色四箱全部纳入计数与 RViz 标记。
                     # 旧口径 ['bin_A','bin_B','bin_C'] 随暖冷两箱 + bin_C
                     # 世界变体一并废止(binC.sdf 已删,历史可考 64cd236/
                     # 3f02c2b)。bin_names 只影响 recorder 判定/计数集合,
                     # 与两臂执行器各自的 bin_names(北排/南排)正交。
                     'csv_path': LaunchConfiguration('result_csv'),
                     'bin_names': (list(north_bins) if not dual_on
                                   else list(all_bins)),
                     # 并联分色无"arm1 中间态"语义:expect_relay 关闭,
                     # arm1/arm2 的 PLACED/MISPLACED 都直接回收腾格;
                     # 经典接力保持 true(arm1 中间 PLACED 等 arm2 终态)
                     'expect_relay': ParameterValue(
                         dual_on and not parallel_on, value_type=bool)}])
    # 注:入箱件回收(recycle_delay)与动态来料计划(demand_parts)均由
    # sorting_params_pipeline.yaml / demo_parts 参数控制,legacy 模式
    # 完全不加载覆盖文件,行为保持基线不变

    # 4) 视觉分类节点
    vision_params = PathJoinSubstitution(
        [FindPackageShare('a1_vision'), 'config', 'vision_params.yaml'])
    vision = Node(
        package='a1_vision', executable='vision_classifier_node',
        name='vision_classifier', output='screen',
        parameters=[vision_params, {'use_sim_time': True}])

    return [cell, moveit, dispatcher, executor_node, relay_node,
            scene_pub, recorder, vision]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fast_sim_waits', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('result_csv', default_value='/tmp/yuri_performance_results.csv'),
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('part_colors', default_value='red,green,red'),
        DeclareLaunchArgument(
            'inject_failure_parts', default_value="['']",
            description='注入一次首次抓取失败的工件列表,如 "[\'part_2\']"'),
        DeclareLaunchArgument(
            'dual_arm', default_value='auto',
            description='双臂模式(2026-10-08 改默认):**auto(默认)= '
                        'pipeline?true:false**——裸命令即 v2 双臂并联分色'
                        '演示;pipeline:=false(legacy 基线)时恒单臂(batch '
                        '三件语义零差)。显式 dual_arm:=false = 单臂流水线;'
                        'dual_arm:=true 强制双臂'),
        DeclareLaunchArgument(
            'pipeline', default_value='true',
            description='流水线模式:传送带逐件来料+真实包夹+触底放置。'
                        '2026-10-03 起为默认演示路径;pipeline:=false 回'
                        'legacy 一次性 batch 三件(验收回归基线用)'),
        DeclareLaunchArgument(
            'demo_parts', default_value='8',
            description='演示总来料数(透传 demand_parts;2026-10-08 起默认'
                        ' 8 件一轮自动收尾)。0=无限持续逐件来料(配合入箱件'
                        '延时回收,长期演示不堆料)'),
        DeclareLaunchArgument(
            'random_colors', default_value='auto',
            description='透传 cell_bringup 的 random 色池。**auto(默认)= '
                        '自动档**:双臂→四色全上(按色路由两臂),单臂 '
                        'pipeline→blue,yellow(arm1 只有北排箱)。显式传色'
                        '(如 red,green)覆盖;显式传空串 ""=旧逃生门(spawn_'
                        'parts 内置四色池,单臂裸跑红绿会 FAILED,仅判别实验用)'),
        DeclareLaunchArgument(
            'gpu_offload', default_value='true',
            description='透传 cell_bringup:true 时 gz 渲染 offload 到 NVIDIA 独显'
                        '(详见 cell_bringup.launch.py 头注释)'),
        DeclareLaunchArgument(
            'tcp_z_offset', default_value='',
            description='单变量实验:覆盖 /pick_place_executor 取件 TCP 偏移'
                        '(yaml 定值 0.208;减小=更深,合法域 [0.201,0.231])。'
                        '空=用 yaml 定值'),
        DeclareLaunchArgument(
            'belt_settle', default_value='',
            description='单变量实验:覆盖 /task_dispatcher belt_settle_delay'
                        '(yaml 定值 2.6)。空=用 yaml 定值'),
        DeclareLaunchArgument(
            'world_file', default_value='sorting_cell.sdf',
            description='透传 cell_bringup:世界文件名(share/a1_cell_sim/'
                        'worlds 下)。默认 sorting_cell.sdf = **布局 v2 四色'
                        '四箱主线世界**(binC 判别变体已删,历史可考 '
                        '64cd236/3f02c2b;world name 恒为 sorting_cell ⇒ '
                        'ros_gz bridge 话题不变)'),
        DeclareLaunchArgument(
            'controllers_file', default_value='ur5e_cell_controllers.yaml',
            description='透传 cell_bringup:控制器配置文件名(P4 gain 档用 '
                        'ur5e_cell_controllers_gain10.yaml;默认=定案 gain 2.0)'),
        DeclareLaunchArgument(
            'relay_overlay', default_value='',
            description='单变量实验:/pick_place_relay(arm2)专用覆盖 yaml 的'
                        '绝对路径,追加在参数表末尾(后加载覆盖前者)。空='
                        '用 sorting_params_pipeline.yaml 的 relay 定值'),
        DeclareLaunchArgument(
            'wedge', default_value='false',
            description='R1-P1 楔死判死闸(执行器判死即停+调度器停轮);'
                        '默认 false=定案态零行为差'),
        DeclareLaunchArgument(
            'wedge_rearm', default_value='false',
            description='R1-P3 续跑钩子:判死后下一件重建 MotionClient'
                        '(配外层受控重启 move_group);默认 false=停轮不续'),
        DeclareLaunchArgument(
            'tool_settle_fix', default_value='false',
            description='G-B latent bug 修正开关:笛卡尔腿 _tool_settled'
                        '引用不存在属性恒超时;默认 false=保留旧时序'),
        DeclareLaunchArgument(
            'arm2_colors', default_value='auto',
            description='双臂并联分色(布局 v2):逗号分隔颜色(如 red,green)'
                        '直接路由 arm2 执行(带尾取件投南排 bin_red/bin_green),'
                        '其余归 arm1(北排 bin_blue/bin_yellow)。'
                        '**auto(默认)**:双臂+pipeline 时取 red,green(v2 '
                        '定案并联);要经典接力请显式传 arm2_colors:=""'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='来料即入队自动派发(定案 true);语言指挥模式传 '
                        'false:来料只登记,派发完全由 /sort_task 服务'
                        '(语音节点 dispatch_single/dispatch_to)驱动'),
        OpaqueFunction(function=launch_setup),
    ])
