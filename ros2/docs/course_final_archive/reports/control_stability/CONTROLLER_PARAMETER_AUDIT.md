# 控制器参数、插件及单因素审计

| 项目 | 实测/审计结果 |
|---|---|
| 基线 | a21ee9b7b0d9631b86dbcca5e15f1b0e556de54e |
| 独立分支 | experiment/control-stability-gains |
| Gazebo / 步长 | Harmonic8.10.0；原fast_sim_1ms.sdf的max_step_size=.001，原生相邻积分1ms |
| 插件 | libgz_ros2_control-system.so创建controller_manager；gz_hardware_plugins中的GazeboSimSystem实现position接口 |
| gz_ros2_control | 1.2.20，原Pixi二进制，未更换依赖 |
| controller_manager / JSB / JTC | 4.49.0 / 4.43.0 / 4.43.0 |
| 控制频率 | 参数500Hz，最近位置读数按控制周期保持；native物理1000Hz |
| runtime gain | A=2.0、B=1.0、C=.5，运行期参数服务及硬件启动日志核对 |
| JTC命令/状态 | position / [position,velocity]，硬件接口清单另存 |
| open_loop_control | B/C/额外gain2探针运行期查询false；interpolate_from_desired_state=false |
| 原open_loop键 | 参数服务查询不可用，未改；不能依据原YAML注释宣称开环已启用 |
| stopped_velocity_tolerance / goal_time | .2 / 2.0（JTC原值）；独立READY仍.3rad/s/.01rad/.25秒/50戳 |

基线ur5e_cell_controllers.yaml原样保留。新增ur5e_cell_controllers_stability_gain2.yaml与其字节一致；gain1/gain0p5仅改/**.ros__parameters.position_proportional_gain。结构化单因素证明见audit/single-factor-proof.json，字节差异见GAIN_PARAMETER_DIFFS.diff，各配置SHA见audit/gain-configurations.json。

无工件：原cell_bringup+MoveIt+原scene_publisher；不运行派发、不请求生成工件；原Gazebo RGB/逻辑相机和桥接仍工作。诊断驱动实例化原PickPlaceExecutorNode/MotionClient，按臂串行使用原initial standby位姿、原carry_anchor1关节目标和原standby_joints。没有捏造part_id或伪发ARM_READY。每步Action结束后观察2sim秒，墙钟上限8秒；比较最后1sim秒，READY实际窗口不改。carry锚点仅使用原补偿门槛.55rad/.3rad/s，不把carry判据冒充.01rad的standby READY。

有工件：原sorting_demo、feed_policy:=rule、1ms/v3、X11 llvmpipe、LP_NUM_THREADS=2、无GUI/RViz；原安全状态机、相机分类、夹爪、规划、几何及碰撞判据均不修改。每轮独立HOME、ROS_DOMAIN_ID、GZ_PARTITION、CSV、ROS_LOG_DIR；复用同一Pixi环境。

只读trace世界从原1ms世界复制，仅增加观测插件；配置/源码中没有物理或传感器删改。v1 PostUpdate与v2 Pre/Post的原始文件及两个.so均保留。v2优先级100只确定观测在默认写命令之后；原控制/物理系统参数与相对顺序不改。对应binary SHA及maps见audit/loaded-binary-hashes.json与各summary.loaded_libraries。

启动期有一次ListControllers请求未返回，尚未发运动，保留为030208启动审计失败；此后增加控制器就绪屏障及有界只读RPC重试。A最初批量查询含未知open_loop导致JTC参数响应空列表，未当成参数不存在的证明；B/C与额外探针改为逐项查询，完整保存结果。以上没有改变控制变量。
