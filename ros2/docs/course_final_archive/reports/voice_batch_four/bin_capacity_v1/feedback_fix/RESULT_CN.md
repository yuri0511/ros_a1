# JointState反馈集中修复结果

## 根因边界

旧故障58ms源时间戳跳跃内，独立观察器有26条消息；消费者两条回调相距0.692ms。已确定缺口属于特定消费者链，不能归因于Gazebo整体停发。旧代码JointState回调在默认MutuallyExclusive回调组/同一SingleThreadedExecutor里执行每消息图查询、ReleaseEvidence、锁紧窗口、完整字典复制和故障写盘；100Hz保持控制定时器及其他订阅共用该Executor。`last`覆盖是旧锁紧窗口轮询缺样的已知原因，但58ms事件的ReleaseEvidence本来逐回调调用，不能再次用last覆盖解释它。

旧记录没有DDS publication sequence、回调执行耗时、队列水位、GIL调度或中间件丢弃事件，仍不能唯一分解DDS丢失/KEEP_LAST覆盖/Executor或GIL积压。空爪结束后动态导入arm_probe在另一个冷进程实测约127ms（非原故障进程），是可复核风险，尚非已证实唯一触发原因。未声称某次图查询就是根因。

## 实际修复

- `joint_receiver.py`独立节点、独立Executor线程，仅JointState订阅；回调只保留真实原始消息、源信息和真实接收/入队时间。
- DDS订阅仍BEST_EFFORT、KEEP_LAST depth=100，没有用加深DDS队列规避问题。独立Python有界2048缓冲**不覆盖**，满即计数并故障；后台逐条消费，记录seq、源stamp、接收墙钟/单调时间、入队/处理/完成时间、队列长度和丢弃数。诊断中的publication sequence可以复核中间件可见缺口。
- ROS图查询移到独立发现线程，每0.1秒检查唯一来源；Jazzy回调info不含GID时沿用已验证的唯一端点GID，缓存过期0.5秒拒绝。身份缓存有轮询时间分辨率，不能宣称有逐消息GID证据。
- 安全判断、字典处理和日志写盘移出接收线程。last仅作为便利快照；ReleaseEvidence与LockWindow均处理每条真实入队样本。保留原接收时间，不能用处理时间刷新过期证据。
- `calibrate_fingers.py`接入上述路径，并将运动消息模块导入提前到rclpy启动前。`calibrate_idle.py`新增模块进入源码哈希/快照。
- 原50ms源间隔、0.5秒新鲜度、50个唯一样本、>=0.1sim秒、0.12sim秒滚动窗、<2mm范围不变。受控夹爪动作和几何/规划/控制器参数未改。

## 测试实证

1. 500–1000Hz真实ROS压力：10,156条接收=处理=参考，DDS发布序号无缺口，覆盖/丢弃0，最大队列3，最大处理延迟3.13ms。人为91.15ms源断流触发原observation_gap。
2. Python/GIL负载压力：10,253条接收=处理=参考，内部覆盖/丢弃0，最大队列14，最大处理延迟12.30ms。人为90.60ms断流仍触发故障。精确指标见RESULTS.json。
3. 只启动一次空闲Gazebo（不启动GUI、无夹爪/机械臂命令发布器、无Action客户端、无工件或送料）：25秒接收窗口，9,762条接收=处理，DDS发布序号连续、内部覆盖/丢弃0、最大队列3、最大处理延迟5.45ms，最大源stamp间隔8ms。独立参考回调总数9,777与9,762差异在加入/退出边缘，共同比较区间没有缺失；旧独立观察器共同比较区间少129个唯一样本，新消费者不缺观察器任何样本。实际唯一源stamp约492Hz、中位间隔2ms，不能由500Hz配置推断每stamp恰好2ms。
4. 场景暂停确认耗时0.344秒，观察器/场景/probe退出均0。校准未启动/voice_batch/stop服务，该服务3秒有界超时如实记录；不能写成ROS停止服务复验通过。没有运动状态下的暂停证据。
5. 30项定向单元/回放/停止测试通过；含队列溢出保留原样本并故障、真实断流、过期处理不续鲜、原锁紧窗口、停止链路。py_compile通过。一次新增测试的假消息缺少position/name字段已修正测试桩，再跑30项通过。仅研究脚本变化，不需要重建ROS消息或机器人包。

## 下一次验收入口及尚有门控

`concentrated_acceptance.py`已准备“一个绑定提交的新明确授权→一次带载校准→完整结果审查→黄色前置检查→新世界黄色入口”的无重试调度骨架；默认--check无动作。未来批准文件scope必须为red4-then-yellow-single-once，当前本轮空闲授权不能复用。校准失败立即停止。

但不能将它称为已具备完整黄色执行能力：现有four_slot_candidate仍motion_validated=false，executor仍有four_slot_exit_clearance_not_calibrated硬阻断；旧语音观察/执行入口仅接受四件任务，尚无合格黄色单件入口。这些与本轮反馈修复无关，已保留，没有改标志或绕过。下一次可以申请带载校准；要申请两阶段集中验收，还须在授权前补齐独立黄色路径/退出证据和黄色单件任务接线，而非在成功后再撰写研究计划。

准确检查命令（不启动任何仿真）：
`python research_tools/four_slot/concentrated_acceptance.py --check`
未来新授权文件准备后统一执行命令：
`python research_tools/four_slot/concentrated_acceptance.py --approval-file /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/feedback_fix/NEXT_ACCEPTANCE_APPROVAL.json`

本轮没有生成该未来批准文件，没有带载/抓放、送料、API或GitHub操作。第二关仍BLOCKED。本轮核心接收模块与实际空闲测试快照哈希一致；最终calibrate_fingers接线和导入提前仅经静态/定向测试，未将空闲测试冒称实际夹爪保持/带载验证。

## 原始证据

本目录stress-500-1000hz/、stress-with-gil-load/、idle-20261017T114006-ce2309/保留原始入队/处理记录、ROS参考时间戳、DDS发现QoS、native stats、旧观察器原始ROS、接触和暂停日志。发布者为joint_state_broadcaster，发现API报告reliability=RELIABLE；history/depth报告UNKNOWN/0，不推测其真实写缓存深度。新消费者BEST_EFFORT depth100，旧观察器sensor_data BEST_EFFORT depth5。数据、源码和测试未改原Rule/Q分支。
