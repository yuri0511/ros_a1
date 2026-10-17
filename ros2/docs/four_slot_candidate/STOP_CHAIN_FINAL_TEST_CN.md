# 停止链路修复与证据边界

保留先暂停世界、再调用/voice_batch/stop、最后有序关闭所属进程，观察器最后退出的继承方案。gz暂停请求2秒协议超时/4秒子进程上限，ROS停止3秒上限；不是“保证1秒内”。不全局pkill。

本轮修复dispatcher主函数finally使用rclpy.try_shutdown，避免SIGINT已关闭context后重复shutdown异常。独立ROS_DOMAIN_ID=94，无Gazebo、无控制器、auto_start=false、Rule调度器实际运行后SIGINT退出0。见dispatcher-sigint.json/log。这是真实ROS进程清理验证，不是实际Gazebo暂停验证。

停止请求每次记录独立attempt、日志文件、墙钟起止及单调时长，重复调用不覆盖第一次原始pause/stop日志；时间线追加保留。4项离线故障注入通过：暂停优先、暂停失败后仍尝试阻断、命令缺失、重复停止证据保留。

历史观察器退出-6的显式解绑补丁继承未删除。本轮没有重做Gazebo空闲或带载暂停，因第二关未通过，遵守顺序不进入第三关。此前空闲约0.30秒不是本候选成绩；此前带载2.12秒问题不能宣布彻底解决。后续必须测风险事件→native暂停ACK→sim_time停止，并持续保存观察器直到服务器退出，目标<1秒但不得预填结果。


状态：独立候选工程，第二关未完整通过，禁止动态抓放。此次没有启动 Gazebo，没有新的实际 PLACED/READY 或接触测量。原工作树 e4dad347d6ef50d5faa4000fbb7888042e815df6 保持不动。
