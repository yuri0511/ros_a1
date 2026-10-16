# READY 与 gain1 修复审核说明

## 问题与根因边界

旧流程把任务逻辑完成和物理可接任务混在一起：return_standby失败仍可能发布COMPLETED并释放槽，控制器SUCCESS也不证明静止。历史wrist_3_joint观测2.886rad/s并非可忽略读数。后续原生Gazebo与500Hz位置接口命令探针、单因素增益实验，把本机约250Hz持续振荡定位到gain2控制因素；没有精确重现历史2.886幅值。位置误差约0.002886rad与2×500=1000关系支持调查，真正结论还依赖实测命令与原生响应。

## 整体协议

- PLACEMENT_FINISHED触发recorder按原判定记录，PART_PLACED仅证明实际放置结果。
- ZONE_CLEAR只释放共享取件区；每臂执行槽仍占用。
- return_standby必须MoveIt/Action严格成功，无活动控制目标；随后独立物理稳定观察。
- ARM_READY需新鲜单来源JointState、正确关节名/目标、位置和发布速度及位置差分速度同时合格；至少50个严格递增时间戳、0.25仿真秒连续窗口。
- 原位置0.01rad、速度0.3rad/s阈值不放宽。重复时间戳不增加样本/窗口或刷新新鲜度，同桶保留最不利载荷；暂停/倒退/重置分别处理，有独立墙钟超时。
- ARM_FAULT锁存并停止新派发；COMPLETED不能覆盖故障或释放槽。真实放置CSV保留，恢复需保存证据/停止本轮后在新世界人工复核，非自动重试无限等待。

同步改动：TaskStatus.msg契约、physical_ready.py、motion_client.py、pick_place_executor_node.py、relay_sequencer.py、task_dispatcher_node.py、result_recorder_node.py及生命周期/物理窗口测试。消息字段未增减，但语义契约变化，必须整体部署。启动已有gain2配置不会自动变为gain1，推荐仅使用明确入口。

## 单因素与物理验收

A gain2保留故障证据；B gain1和C gain0.5空载两臂通过。只有gain1做了带件与完整八件，不能把0.5说成已验证抓放。1ms步长、500Hz、几何/惯量/阻尼/轨迹缩放及安全阈值不变。

唯一完整Rule运行：control_stability/runs/20261017T031609-gain1-rule-eight；运行源码ed76c35839ffd878cb8134e5bdc6de5f6c050364。8/8入正确箱、四箱各2；148 Action全部成功；8 READY及8匹配释放；0重试/FAULT。READY原生窗口速度最大约0.000923rad/s，逐次独立复核通过。任务首PICKING到末READY为239.41墙钟秒、143.177仿真秒，包含高频诊断干扰，不用于旧低干扰性能对比。

705d1af仅新增报告/离线分析/图证，无运行源码变化。交付核心aggregate SHA256=7e03965549dabec1f53aae3215f1c9e5b071b27827e90958683d4b6a30d198f3；gain1配置SHA256=c96a32325ac2d66b958cb4c75ac82b794e1fe64802e25394c717436121b9a5b9。原gain2配置保持。新便携入口使用fast_sim_1ms.sdf，不携带绝对.so路径，除去只读观测插件后世界XML语义相同。

本修复使故障显式阻断，属于有有限仿真验收的候选；没有长期可靠性、硬件安全或无接触认证。
