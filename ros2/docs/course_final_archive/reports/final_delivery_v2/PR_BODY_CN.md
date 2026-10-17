本 PR 是 **A1 双机械臂分类搬运课程项目的实验审核交付**，保持 Draft，不合并。基于 `72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd`；正式分支、原始工作区、旧 Q 表和历史实验保留。实际修改作者 Ping（AI 辅助），认证账号 yuri0511 不等于代码作者。

## 本次最终整合

- **READY 错误释放任务槽已修复**：`PART_PLACED`只确认放置；`ZONE_CLEAR`只释放共享取件区；严格控制器结果和新鲜连续物理稳定窗口通过后才发`ARM_READY`/释放对应臂槽。`ARM_FAULT`锁存，旧`COMPLETED`不能覆盖故障。TaskStatus、MotionClient、executor、dispatcher、sequencer、recorder及测试整体升级。
- 重复时间戳去重，不增加样本/窗口或伪造新鲜度；暂停/时钟异常及墙钟超时有界失败。位置/速度阈值没有放宽，取件互锁和每臂槽保留。
- 本机单因素及原生速度命令/物理数据证实gain2是约250Hz持续振荡的控制因素。**新增显式gain1配置**，保留原gain2 YAML；不改变1ms、500Hz、几何、惯量、摩擦、轨迹缩放、碰撞容差。
- gain1＋READY **一次真实Rule八件**：8/8正确、四箱各2、148底层轨迹Action全部成功、8次独立物理READY/8次匹配槽释放、0重试/0 ARM_FAULT。运行`ed76c35839ffd878cb8134e5bdc6de5f6c050364`到报告提交`705d1af`无ROS运行源码/参数变化，交付核心/参数一致。
- 提供 `ros2/research_tools/final_system/run_rule.sh`：强制gain1/Rule/1ms/v3/headless/X11 llvmpipe/LP_NUM_THREADS=2，独立结果与有界清理。便携世界删除了本机绝对路径的只读观测插件；不把新增包装脚本冒充原已执行的诊断命令。

## Q-learning 的真实状态与异常

保留真实离线Q-learning、V1/V2和DP工具，不重新训练，不将未充分验证V2表替换正式默认表。`deployment_ready=false`、`overlap_validated=false`。

本轮只执行 **一次V2通用Q真实八件功能诊断**，gain1＋完整物理READY：8/8正确、149底层轨迹控制器结果全成功、8次READY独立复核通过、8次Q决策全部命中、0未知回退、0WAIT；实际红蓝绿黄红蓝绿黄，首色未强制。

**不是零异常验收**：第3件MoveIt出现空JointState日志及起始状态预测碰撞拒绝（双臂腕/夹爪模型），MoveGroup Action ABORTED，原有界恢复后同任务重试1次。底层JTC无失败；没有任务槽提前交给另一任务。此问题的起始状态/规划场景同步原因尚未确认，不隐藏、不筛选性重跑。旧简单审计把同件重试PICKING算成新任务，原输出保留，并附逐任务复核。

**Q仍未证明真实Gazebo提速**：历史72ab三组低干扰配对Rule184.59秒、Q187.96秒，均24/24正确，重试3对7；V1模型最优、V2均值模型和DP不是物理收益证明。本轮高频诊断与既有Rule耗时不可直接比较，不开展16组。

## 性能历史与仍存限制

- 1ms事件等待/无GUI传感器保留/双实例已真实验证；2ms严重抓取失败，不采用；8da双实例短测整体吞吐优于三实例（后者OUTSIDE），不把不同提交数据混用。
- V2仍缺失败恢复、物理READY与新gain1时序的充分校准；所有50,000回合为离线模型训练，非Gazebo回合。
- Rule放置阶段约0.137rad肩偏差gain2/gain1均存在，指尖接触/穿深仍需审查。本轮Q记录10个腕/夹爪基座-工件接触点（最大0.146mm）及指尖最大14.909mm；没有记录物理臂间/主体撞箱类别，不等于无碰撞或长期认证。
- 最新一轮是有限物理回归；历史2.886rad/s截断诊断及Q未启动记录仍保留，不被改写为失败率正式测量。
- 语音只有结构化JSON契约及18项模拟测试；未取得可核验完整ASR/LLM源码，**尚未真实集成**。不替换Yuri双臂执行器、不虚构part_id、不猜left/right箱、不绕READY/FAULT、不杀运动进程做stop。

## 验证与材料

4个ROS2包构建完成；**244测试通过、4项既有失败**（camera断言、relay旧键集合与缺失G0锚表），未伪造或跳过。Q静态检查165种有效8件需求合法完成，模型1822决策有4未知回退，与真实本轮8决策0回退分开统计。

先读 [最终系统状态](https://github.com/yuri0511/ros_a1/blob/review/yu-gazebo-performance-rl-v2/ros2/docs/final_delivery_v2/FINAL_SYSTEM_STATUS_CN.md)、[修复报告](https://github.com/yuri0511/ros_a1/blob/review/yu-gazebo-performance-rl-v2/ros2/docs/final_delivery_v2/READY_GAIN1_FIX_REPORT_CN.md)、[Rule/Q诊断](https://github.com/yuri0511/ros_a1/blob/review/yu-gazebo-performance-rl-v2/ros2/docs/final_delivery_v2/RULE_Q_VALIDATION_CN.md)、[完整交接](https://github.com/yuri0511/ros_a1/blob/review/yu-gazebo-performance-rl-v2/ros2/docs/final_delivery_v2/FINAL_WORK_HANDOFF_CN.md)。同目录有课程报告、8页PPT提纲、2–3分钟视频方案、语音边界、小型原始CSV/关键日志摘录及SHA256索引。

大型原始接触CSV、全量高频流、旧82.95MiB证据ZIP、Pixi/build/install/log不上传Git，原本机目录完整保留。请Yuri优先审核状态协议、规划起始状态异常和接触限制；不自动合并、不自动开展新实验。
