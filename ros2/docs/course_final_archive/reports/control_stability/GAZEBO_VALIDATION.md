# 有限物理回归：gain1完整8件Rule通过

目录runs/20261017T031609-gain1-rule-eight，实际运行源码ed76c35（完整SHA在metadata与FINAL_STATE）。无GUI/RViz、1ms、500Hz、v3、X11 llvmpipe、LP_NUM_THREADS=2、feed_policy:=rule，独立实例。原几何/规划/控制阈值/相机/夹爪/安全状态机不变；仅控制器gain从2改1。

## 结果

- 原结果记录器：8/8 PLACED，blue/red/yellow/green各2，四个对应箱各2，无放错/失败。
- 抓取失败/重试日志0；控制器结构化Action结果148条，全部status4/error_code0，未发现Action失败；MoveIt规划错误另有原launch日志可审。
- PART_PLACED 8次、ZONE_CLEAR 8次、ARM_READY 8次，ARM_FAULT/FAILED/PART_REJECTED 0次；执行槽释放8次，全部trigger=ARM_READY。
- 同一臂未在前一件READY前开始下一件；两臂各4件，全局取件区释放与执行槽释放分别记录，未混淆；已有单元测试继续覆盖失败后锁存及迟到COMPLETED无权释放。
- 8次READY均有成功return Action、无活跃Goal、至少50个递增戳/.25sim秒、原位置/速度/新鲜度门槛。各次ROS原始数据与native1ms数据独立复核均通过；全轮READY区间原生最大速度0.000923170rad/s，小于.3，最大目标位置误差2.025e-13rad，小于.01。没有放宽或跳过验证。
- 第一次PICKING到最后ARM_READY：239.410墙钟秒、143.177仿真秒；完整启动/清理生命周期268.975墙钟秒。supervisor.measure_end是旧CSV时间口径，不用它冒充最终物理READY耗时。高频记录有额外负载，不据本轮声称性能收益。

物料顺序原样：blue,red,yellow,green,blue,red,yellow,green；非强制首色的Q实验，完全未运行Q。

## 接触、传感器和清理

contact CSV共581333行，finger_part 565079、finger_conveyor 2754、finger_bin 13500；没有interarm/body_bin记录，也没有对应guard触发。finger_bin最大.000094m，finger_part最大.014026m；仍保留既有接触段姿态偏差与抓取穿深限制，不宣称无碰撞风险或几何安全认证。

Gazebo RGB1569帧、ROS RGB422帧、逻辑相机1571条由probe记录，8条来料；视觉和物理未关闭。高频证据含78414条JointState、154644条ControllerState、296条Action状态、64条任务/结果事件。node日志含全部控制器结果，namespace/分区环境见launch-environment.json。

正常达到8次READY后由监督器按原停止路径SIGINT该轮launch并回收辅助记录器，summary.residual_session_cleanup=[]；最终无仿真残留。部分Python节点会把SIGINT记作KeyboardInterrupt/exit -2，不将其写成运行期控制器崩溃，也不谎称全部节点自然exit0。

## 所有轮次与范围

1. 030208 gain2启动参数审计请求超时：未发运动，保留失败记录。
2. 030406 gain2空载：初始/搬运通过，arm1返回持续超速，FAULT即停；未继续arm2。
3. 030503 gain1空载：两臂三阶段均通过。
4. 030615 gain.5空载：两臂三阶段均通过。
5. 030953 gain2额外命令探针：修正观测相位，carry持续振荡即停，不作为成功重跑。
6. 031110 gain1两件：2/2正确、两臂READY独立复核通过，无重试。
7. 031609 gain1八件：本报告，唯一一次完整8件；通过后没有再跑。

本轮验证了候选在当前VM、当前八件需求及一次回归上的行为，不是长期可靠率、16组统计、Q策略收益或真实硬件安全认证。旧失败证据均保留，未将历史失败改为成功。原物理READY故障阻断机制和单元测试保留。

独立逐件复核见audit/loaded-20261017T031609-gain1-rule-eight.json；完整命令、环境、原始stats/JointState/ControllerState/native CSV/contacts/results均在上述run目录。
