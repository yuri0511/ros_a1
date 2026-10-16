# 一次受控短时真实验证（未通过）

日期2026-10-17。运行目录：/home/yu/yuri_gazebo_performance/ready_bugfix/runs/20261017T013726-ready-short-1ms。运行源码：a03bbdc（完整SHA见audit/run-source.json），基于81e55c2。最终代码在此之后继续修正，未重复物理测试。

命令：
```
PERF_POLICY=rule PERF_CONTACT=1 PERF_PARTS=1 python3 /home/yu/yuri_gazebo_performance/ready_bugfix/tools/run_case.py ready-short-1ms 1 true 1 160
```

1ms、v3、无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2、单实例、全新HOME/ROS_DOMAIN_ID/GZ_PARTITION/CSV。没有改变几何、控制、相机、Q表；只允许最多一件，不是八件吞吐样本。

初始standby控制器结果SUCCEEDED(status4)、error_code0，初版物理监测器因重复关节header戳锁存异常，8墙钟秒到期拒绝READY。尚未开始抓取，也没有进入return_standby。观察器原样记录FAILED与ARM_FAULT，CSV一条FAILED；这不是正式Rule失败率或2.886重新复现。

fault wall=1792172274.518041，ROS sim21.966/21.967s；pause服务data:true确认wall=1792172274.924434，延迟约0.406秒。完整生命周期42.023墙钟秒，包含清理，不是批次完成耗时。原生接触CSV0点，不能据此证明后续取放轨迹无碰撞。scene_publisher关闭超时由launch升级信号清理，不当作运动运行中崩溃。

高频证据：high-rate.jsonl.gz含11110个JointState、18443个控制器状态、2个Action状态、2个任务事件；单发布者，481对同戳，未发现倒退。完整原始名字、位置、速度、effort、header、wall、monotonic、观察到的ROS clock、controller reference/feedback和Goal UUID状态均存。低干扰probe另存native stats/传感器证据；无丢帧补造。

发现初版永久重复戳锁存策略过严后，最终改为重复戳中断稳定窗口、不刷新/不计数；倒退仍锁故障。**只做离线回放，没有补跑。** 回放仍未通过连续窗口，最后稳定片段约.105sim秒；controller_ok/no_active_goal在回放中作乐观假定，即便如此仍拒绝。此为保守性检查，不能替代真实控制状态验证。

最终版本还新增原recorder真值确认、双源消息乱序处理、命令原子切换和.01rad READY位置门槛，均未实测。**没有执行完整8件规则回归，没有Q轮，没有16组。** 因而不能声明原8件功能、性能或长期可靠性已保持。

停止路径：ARM_FAULT/FAILED观察器写guard-fault→监督器20Hz检测→原分区/world pause→SIGINT该launch会话→有界TERM/KILL回收本轮残留；只操作本轮PID/session，最终无残留。恢复仅支持保存证据、完整停轮、人工复核后新世界/新进程，不支持清FAULT继续同一不明姿态。

下一步应先审查重复header戳与物理积分戳同步/有效连续样本定义，再在明确授权下重新验证最终版。不能放宽速度、位置或减少窗口以获得成功结果。
