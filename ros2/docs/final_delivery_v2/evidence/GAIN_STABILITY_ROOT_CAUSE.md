# gz_ros2_control位置控制稳定性：本机根因验证

2026-10-17，独立分支experiment/control-stability-gains，基于a21ee9b7b0d9631b86dbcca5e15f1b0e556de54e。原ready_bugfix、原工程、历史性能/RL/Q表、正式分支与Draft PR均保留。

**结论：在本机所测锚点工况下，gain=2是持续振荡的可干预控制因素，已由单因素变化、原生命令/物理响应和源码共同证实；gain=1配合已有物理READY协议完成本轮8件Rule。** 不是仅根据理论推断，也不是宣称任何轨迹/任何模型均已验证。

## 证据链

实际安装gz_ros2_control1.2.20，实际进程maps加载原Pixi环境libgz_ros2_control-system.so和libgz_hardware_plugins.so；后者的gz_ros2_control/GazeboSimSystem接受JTC position命令。已保存库SHA、插件XML、包版本、对应版本官方源码。未替换/重编译官方插件。

GetParameters运行时确认A/B/C的/gz_ros_control.position_proportional_gain分别2.0/1.0/.5，controller_manager.update_rate均500、use_sim_time=true。A的4个硬件系统启动日志均gain2；B/C相同检查。native记录实际积分间隔1ms；原URDF、惯量、阻尼、摩擦、规划/缩放、锚点、轨迹目标、READY阈值不变。

[官方Jazzy文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html#notes-on-the-command-interface)说明position通道以误差生成速度目标；[1.2.20实际对应源码](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.20/gz_ros2_control/src/gz_system.cpp)与之相符：

```
v_cmd = gain * (q_target - q_sampled) * 500
```

控制器每约2ms读位置、物理每1ms积分，位置接口在PreUpdate重复使用最近控制读数。理想采样保持模型e[k+1]≈(1-gain)e[k]；gain2的离散极点约-1，可产生不衰减的交替响应（实际DART约束、阻尼/浮点会修饰幅度）。这段模型用于解释观测，不替代测量。

## 直接观测，不仅是ROS发布现象

A的无工件arm1返回后，原生速度峰值1.699040rad/s，位置偏差约.00169904rad，主频约249.75Hz（1秒FFT约1Hz分辨率）。Action已SUCCESS，但READY正确拒绝，停止该轮，未继续arm2。

第一版记录器在PostUpdate读取JointVelocityCmd，读到的全0不能代表控制器下发值；该字段原样保留，分析将其标为不可用，未用零值声称无命令振荡。随后额外进行**一次命令采样诊断**：只读系统以priority100在默认命令写入器之后、Physics Update之前采命令，再于PostUpdate采物理状态；没有修改任何组件/力/速度/实体，也没有修改其他系统的相对执行顺序。

该额外gain2诊断在空载carry保持段出现：

- JTC位置目标固定为wrist3=-4.41639rad，不是JTC位置目标来回摆动。
- native位置误差峰值0.002656321rad，实际速度峰值2.656321062rad/s。
- PreUpdate实际速度目标峰值2.656321064rad/s，与物理速度相关系数1.000000。
- 用实际JTC末点与最近2ms采样保持位置重建速度命令，窗口内RMSE=0rad/s；这不是用接收墙钟捏造仿真时间。
- 主频249.75Hz，native速度相对native位置相位约44.96°。1ms离散积分在250Hz下的该相位关系合理；不能直接套连续时间90°。

因此：JTC位置参考静止→底层比例位置接口产生交替速度目标→Gazebo真实位置/速度随之振荡→ROS发布该状态。重复header会造成混叠/误停，但不是全部振荡的来源。降低gain到1/.5后同一组目标的原生振荡消失，空载两臂均通过。

历史2.886rad/s与.002886rad误差的1000倍比例支持同一机制，但本轮没有复制其完整同一轨迹并复现完全相同幅值；不能把2.656或1.699写成2.886精确复现。初始条件与采样相位会改变边界振荡的幅度和首先失败的阶段。

## Action SUCCESS与物理READY

JTC使用position命令接口；对应4.43.0源码的速度误差计算受命令接口条件限制，SUCCESS不构成物理静止证明。运行时open_loop_control=false，原YAML的open_loop键查询不可用；原注释不能当实际生效配置，本轮未把它改名或改变行为。

继承的协议仍要求成功Action/error0、无活跃目标、新鲜唯一关节数据、.01rad位置、发布及差分速度.3rad/s、至少50递增戳/.25sim秒。没有忽略速度、扩大容差、缩短窗口或直接设置待机标志。gain1完整8件里每次真实READY均有原生数据独立复核。

所有原始目录、启动失败、故障轮、额外探针均列在CASE_SUMMARY.csv和audit/native-response-analysis.json；未重跑筛选成功轮。额外命令诊断因采样相位修正而进行，明确不与首轮当同一测量混算。
