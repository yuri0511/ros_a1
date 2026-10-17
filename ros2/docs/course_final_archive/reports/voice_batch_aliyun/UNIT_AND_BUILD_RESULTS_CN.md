# 构建与测试

复用/home/yu/ros_a1/ros2/.pixi/envs/default，未升级ROS/Gazebo/MoveIt/Pixi/锁文件。新工作树四包colcon构建全部完成；最终audit/build-final.log。新service由rosidl正常生成。

最终pytest：**329通过、4项历史失败**，见audit/final-tests.log。原已通过244项无新增失败，新模块与批次测试增85项；旧语音18项契约原样通过。四项历史失败仍为camera h_fov断言、relay键集合差异和两个缺失G0锚表断言，未伪造、跳过或扩大通过范围。

新增覆盖：真实原话及黄色的/黄的、另一自然表达、关闭式JSON/重复键/无穷/布尔/负数/小数/超量/首色/顺序/未知控制字段；同Key两个模型、未授权零网络、4次预算/失败计数/无自动重试、私有权限/重定向拒绝；readiness/plan先后次序、未确认零送料、原dispatcher入口并发只发一次、RGB过期等待、强首黄完成/READY/槽释放、同箱回收成功+新鲜无旧件帧、错误视觉、额外实体、FAULT后迟到事件、独立墙钟超时、真实ResultBook五件CSV统计与多件/重复/放错拒绝、主计划未通过几何门时拒绝提交。

另有真实ROS通信smoke（无Gazebo/无运动）：SubmitBatchPlan拒绝未确认/主计划绿3风险/重复请求，安全子集仅受理为pending-readiness，硬件未ready始终0送料，旧SortTask被隔离，stop锁存。audit/ros-transport-smoke.log。它不是主计划物理成功或用户任务确认。

开发中首轮适配测试出现旧测试mock对象缺少新_voice_mode属性；以默认false兼容读取修复。最早原始日志first-integration-tests.log保留。FCL首编译被pkg-config尾部C++11标志覆盖，调整最后显式C++17后编译成功，无新依赖；最终mesh-build.log。均不属于真实机器人运行失败。

没有新增Gazebo轮次；不能把本测试结果当原Rule/Q重新物理验收。原动力学、轨迹、夹爪、READY阈值和Q表文件未改，默认voice模式关闭，静态/单元兼容性成立；真实新模式可靠性尚待安全修复后检验。
