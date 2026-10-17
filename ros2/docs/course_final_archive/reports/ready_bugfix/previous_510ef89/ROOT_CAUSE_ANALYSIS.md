# READY/return_standby根因审计

日期2026-10-17（Asia/Shanghai）。只在bugfix/physical-arm-ready独立工作树修复，基于审核PR #1的81e55c29241c7e8dc089e73a10409e965b439838；正式ros2-dual-parallel仍72ab，原083a工作区及全部历史证据保留。没有推送、合并、训练或16组配对。

## 已确认的代码缺陷

原MotionClient在FollowJointTrajectory status=SUCCEEDED时直接True，未独立要求真实关节稳定，也未核对该Action的error_code=0。失败status还存在saw_settled/settled_fn兜底，可把失败按成功处理。MoveIt的plan_only成功只证明规划结果，控制器SUCCESS证明其Action契约，并不等于仿真关节连续静止。

原executor return_standby True即_at_standby=True；返回False后finally仍_flush_terminal_status，且初始standby异常被告警吞掉后仍置真。Dispatcher把COMPLETED/FAILED送RelaySequencer.on_terminal释放本臂槽。ZONE_CLEAR原本只放取件区锁，没有直接放臂槽；真正问题是终态同时承载任务结果与物理可复用语义。

## 历史2.886读数核对

原始目录：/home/yu/yuri_gazebo_performance/yuri_review/diagnostic/runs/20261016T171634-diag-rule/instance-1。

故障wall=1792142237.332762，ROS sim36.921s，关节缓存年龄约1.073ms。按name精确匹配arm1的wrist_3_joint，位置-1.87931432767，目标-1.8822，误差+.00288567233rad；velocity=+2.88567233263rad/s。原采样names唯一，不是arm2_wrist_3_joint；JointState消息定义对转动关节单位为rad/rad/s。控制器日志显示return_standby完成，_at_standby=True；当时保护阻止终态flush，未释放该臂槽。第二臂在故障前已由合法ZONE_CLEAR接到part2，不是故障后新增派发。

5Hz原始连续采样：sim36.819到37.052，位置-1.88376882→-1.87931433，净差分约.019118rad/s，但最后发布速度2.885672。区间平均速度不等于瞬时速度，5Hz无法还原高频信号，不能以此把2.886忽略或判成假数据。全部逐行数值在audit/existing-evidence.json。

本机gz_ros2_control=1.2.20。对应[官方版本源码](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.20/gz_ros2_control/src/gz_system.cpp)的read读取Gazebo JointPosition/JointVelocity组件；position写入通道把位置误差乘update_rate和position_proportional_gain换成速度命令。实测日志gain=2，原配置update_rate=500，比例量级1000；历史abs(v/error)也约1000。此吻合支持控制离散行为的调查方向，但不是已证明的因果或源码与二进制逐指令等价。没有修改该插件、增益或控制频率。

## 本次一次短时真实验证

运行20261017T013726-ready-short-1ms，1ms/v3/headless/X11 llvmpipe/LP2、单实例、最多一件Rule。实际运行源码快照a03bbdc，后续最终修订不与该现场版本混称。

初始standby控制器原始结果status=4（SUCCEEDED）、error_code=0，尚未进入抓取/return路径。监测器在8秒有界观察后拒绝READY，原因nonincreasing_joint_stamp。观察器收到FAILED和ARM_FAULT，暂停服务确认data:true；没有ARM_READY，没有放置结果，没有继续完整8件。该FAILED行属于截断诊断，不计正式Rule失败率。

高频原始记录JointState=11110帧、controller_state=18443帧、Action状态=2条，单一joint_state_broadcaster/GID，所有joint names唯一。发现481对相同header stamp（未发现倒退）；部分同戳位置不同，时间戳不能充当每个物理积分步的精确标签。初版把重复戳永久锁故障过严，最终改为重置稳定窗口、不计新证据/不刷新新鲜度；真正倒退仍锁故障。

最终两sim秒该次初始standby腕3位置范围约2.49e-7rad，速度约±.000124608rad/s，有小幅交替振荡；六关节均接近目标。**这不是历史长距离return的同一运动，不是再次复现2.886。** 不能从本轮推断历史高速读数就是某个确定的数值振荡。

修订后只做原数据离线回放，即使乐观假定控制器已无活跃目标，也没有通过.25sim秒/至少50新戳的连续窗口。最终快照未新增Gazebo验收，不能宣称真实修复通过。

## 结论分级

- 确认：历史读数来自arm1同名JointState；接口成功/标志真与物理确认混淆的代码缺陷；本次单发布者重复戳；失败后保持臂槽/停派发的新纯逻辑测试。
- 尚未最终确认：历史2.886的完整物理/数值因果、每积分步真实速度与发布戳的同步、最终代码的八件可靠性。
- 优先补证据：控制器update时间与/clock、JointState header、原生ECM积分戳的同步关系；同一return轨迹的高频关节/参考/实际/effort。需另行授权诊断，不修改增益或容差来绕过。
