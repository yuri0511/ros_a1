# 停止链路与观察器复核

旧62987d0先结束观察器、再等ROS停止、后暂停世界，接触到HALTED约2.119s；原-6仅有terminate报错，没有C++栈，根因不能唯一确认。fd6已将世界暂停前置。本轮观察器进一步在Python退出前设置stopping、显式unsubscribe全部GZ话题、析构native Node/gc后关闭文件，避免解释器清理期间native回调/线程仍存活。未改接触阈值。

## 实测

三次均为空闲世界，零任务、零Action目标、零生成，非抓放复验。第一/二轮只运行仿真层，真实世界暂停成功，但ROS停止服务不存在，3秒超时如实记录；它们不是完整ROS停止成功。

最终轮 `idle-20261017T083902-04a4ec` 在末段启动真实task_dispatcher的voice模式，仅提供停服务，不启动执行器/MoveIt，不伪造硬件READY。pause_world耗时0.302s，随后真实 `/voice_batch/stop` success=True，日志HALTED；未提交计划。原数据 stop-timeline.jsonl、pause.log、stop-service.log、dispatcher.log。

native首次paused=true距请求0.301s；之后4.609s连续报告sim_time固定57.405，证明暂停已实际生效而不只相信RPC返回。JointState停止增长与仿真暂停一致；native stats持续至清理前。观察器0、GUI0、scene0，gzip完整校验成功，记录28116 JointState、53990 ControllerState、536stats，无contact点（空闲）。不是使用暂停后的墙钟伪造新关节样本。

**剩余问题：真实dispatcher SIGINT退出码1**，原main在rclpy信号处理已shutdown后再次shutdown，抛`rcl_shutdown already called`。服务此前已HALTED且所有进程已退出；不能把该非零退出描述成所有ROS节点优雅正常退出。本轮保留其源码行为和原错误，后续可独立将main最终清理改成try_shutdown并做生命周期回归，不涉及运动语义。

观察器连续三次空闲退出0，支持显式native析构方案在本环境有效，但不能证明旧-6的唯一根因，也不等于活动轨迹中停机/故障都通过。没有接触事件的空闲测试不能复验接触流停止后的尾部完整性；只证明本轮stats持续与关节停止时刻相符。新链路不是硬实时急停，CLI启动仍约0.3秒，活动运动需另经批准受控验证。

## 有界失败路径

离线注入：暂停超时仍尝试ROS halt；两命令不存在也分别记录失败；所有RPC有2秒请求/4秒进程、3秒ROS进程上限；子进程按自有进程组SIGINT→SIGTERM→SIGKILL清理。原子SHUTDOWN_INTENT阻止新的提交/故障覆盖，但观察器继续记录；SUPERVISOR_STOP最后触发。未全局pkill。

测试日志audit/ui-stop-tests-final.log共7通过。失败RPC不能宣称已暂停；当前兜底是停止自有仿真进程并保存ACK失败，不能继续派发。空闲脚本异常路径也先请求暂停再清理。最终无Gazebo/MoveIt/观察器残留；ROS daemon单独清理。
