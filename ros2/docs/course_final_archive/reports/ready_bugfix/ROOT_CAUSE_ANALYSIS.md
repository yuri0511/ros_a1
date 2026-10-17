# READY / return_standby 续修根因分析

2026-10-17；本次起点510ef89d958af29634abb24af3a91876d8a96376，独立bugfix/physical-arm-ready分支。结论：**重复header戳造成的误停已修正；返回后物理速度仍超限，因此一件全流程验收未通过，未启动8件回归。** 不是完整安全修复验收成功。

原有报告、工具、审计输出已冻结在previous_510ef89/；历史runs未覆盖。此次没有训练、Q实验、语音整合、推送或PR操作；没有改变控制参数、物理步长、几何或安全阈值。

## 重复戳的来源与证据

旧短测20261017T013726-ready-short-1ms共11110条JointState，481对相邻重复戳；481对的位置与速度都不同，完全相同载荷0对。发布端点只有joint_state_broadcaster一个GID，话题/joint_states；没有同名关节合并、多发布者、header倒退或native stats仿真倒退的证据。不能把“同戳”说成同一DDS消息重复投递。有效区间消息约498.263条/仿真秒，递增戳约476.897个/仿真秒；接收monotonic间隔中位约1.291墙钟ms。1ms物理步与500Hz控制器意味着名义2ms控制采样；实测header间隔分布含0/1/2/3/4ms，不能用每帧固定2ms猜测时间。

安装版本：gz_ros2_control1.2.20、controller_manager4.49.0、joint_state_broadcaster4.43.0。已保存对应版本官方源码及哈希：

1. [gz插件](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.20/gz_ros2_control/src/gz_ros2_control_plugin.cpp) PostUpdate按native sim_time/控制周期read+update。
2. [controller_manager](https://raw.githubusercontent.com/ros-controls/ros2_control/4.49.0/controller_manager/src/controller_manager.cpp)调用controller trigger_update时传this->now()，而非直接传插件update的native时间参数。
3. [broadcaster](https://raw.githubusercontent.com/ros-controls/ros2_controllers/4.43.0/joint_state_broadcaster/src/joint_state_broadcaster.cpp)读取最新状态接口并把传入time写入header。

这条路径使控制状态更新与异步/clock缓存更新不必一一对应，可产生新数值而同header。源码路径与单发布者实测相符；未修改第三方库，也未把它宣称为二进制逐指令追踪。观察器自己收到的/clock同样是异步缓存，不能作为每步积分戳；DDS source_timestamp/接收monotonic均不能伪装成仿真header。详细分布与样本见audit/continuation-timestamp-audit.json。

旧误停的直接原因是监测器把重复戳视为永久异常，之后又改为每次重复清空窗口；后一策略在频繁同戳下也无法积累.25sim秒。初始待机原始值本可满足门槛。新同戳桶回放旧数据：Action成功后121个递增戳、.253sim秒通过，最大发布速度.046768、差分.093536rad/s；这是离线回放，不冒充现场验收。

## 本次真实运行与新的阻断点

先有一次启动失败20261017T022945-ready-dedup-one-1ms，来源信息被错误按对象读取，执行器在第一条JointState处退出，没有任务开始；所有原始记录保留。随后先做无Gazebo ROS接口预检：本机回调info是dict，只有时间/序号，没有publisher_gid，改用唯一ROS图端点GID；多/无端点拒绝，不以接收时间生成来源或时间戳。修订并完成单测后只进行一次真正的一件取放，没有挑选重跑物理失败轮。

实际运行f8e484ccc057638918762786956a21c1d63ca490，目录runs/20261017T023230-ready-native-one-1ms。初始standby Action SUCCEEDED/error_code0后，125个递增戳、.252sim秒、最大速度.007014rad/s，READY内部闸通过，随后才开始PICKING。证明重复戳不再导致该初始误停。

part_1 blue正确进入bin_blue，PART_PLACED保留。return_standby在wall1792175594.0035784报告SUCCEEDED/error_code0；之后8墙钟秒始终不通过物理闸，wall1792175602.0206723发布ARM_FAULT。末2sim秒996条样本中，wrist_3_joint：

- 目标-1.8822rad；位置[-1.88268933344,-1.88171066656]，误差约±.000489333rad，远小于.01位置门槛。
- 发布速度约±.489333rad/s，100%超过.3门槛；递增戳位置差分约±.978667rad/s。
- 净区间位移速度约-4.08e-10rad/s，**净位移近零不代表瞬时静止**。
- 控制器reference/output保持待机目标、reference速度0，feedback速度仍约±.489334；与JointState一致，并非只是重复戳载荷。

本轮没有精确复现历史2.886幅值，但复现了“Action成功、腕3持续非零速度、不能物理READY”的同类不一致。不得删掉/平均掉速度后通过。其他五关节本轮末段发布速度均<.3。

## Action为什么仍报告成功

[JTC对应源码](https://raw.githubusercontent.com/ros-controls/ros2_controllers/4.43.0/joint_trajectory_controller/src/joint_trajectory_controller.cpp) compute_error_for_joint仅在velocity或effort命令接口下计算速度误差；本项目为position命令接口。实际controller_state反馈腕3速度-.489334，而error.velocities全0。因此不能把配置中的stopped_velocity_tolerance或Action SUCCESS解释为独立物理静止保证。YAML的open_loop注释/键不能替代运行参数核实；本轮没有运行期参数dump，未断言open_loop真正生效。

[Gazebo系统源码](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.20/gz_ros2_control/src/gz_system.cpp)以采样位置误差×update_rate×gain生成速度命令；日志gain2、配置500Hz，倍数1000。位置缓存每控制周期读、写在每物理步，简化无负载离散式e[k+1]≈(1-gain)e[k]，gain2位于交替振荡边界。这与本轮±误差/±速度比例及旧2.886/误差比例相符，是**有源码和数据支持的候选机制，不是已完成因果干预验证**。没有改增益/控制频率/轨迹或放宽门槛来消除读数。

下一步需Yuri优先审查底层position采样保持/离散稳定性及JTC物理终态语义；决定是否修订控制实现后，另行做受控安全验证。仅修观察器不能使真实超限速度合法。两臂并发READY、完整8件、安全恢复尚无本轮物理通过证据。
