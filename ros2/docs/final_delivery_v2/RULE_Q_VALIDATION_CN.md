# Rule / Q 最终真实验证记录

## Rule演示基线（不新增重跑）

使用control_stability/runs/20261017T031609-gain1-rule-eight既有唯一完整回归：8/8正确，四箱各2；148轨迹控制器Action成功、8物理READY及8对应槽释放、0重试、0 ARM_FAULT。源码ed76c35839ffd878cb8134e5bdc6de5f6c050364；核心/参数与最终整合相同。高频诊断墙钟239.41秒、仿真143.177秒不能与旧低干扰基线直接比较。

## 唯一一次V2通用Q功能诊断

运行目录：/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight。独立新世界、单实例，源码39e6d64f9856b0f92f76ff934f1d9011eaa23ed6（干净），gain1、物理READY、1ms、500Hz、v3、无GUI/RViz、X11/llvmpipe、LP_NUM_THREADS=2；没有第二次尝试、没有重新训练。

实际加载：codex_rl_v2/dual_arm_rl；schema=2，prefeed=false。配置规范哈希ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b；配置文件574a24115d663c8a73c30f897f6b824bd32a6357803147e4a35e76b6339f6b01。Q表路径/home/yu/yuri_gazebo_performance/final_delivery_v2/worktree/ros2/codex_rl_v2/artifacts/v2/q-v2-general.json，SHA256=18c4b343b0c359b2c740d9d6eeabef273be3d93e4e573ee28a32b53216977e2c。日志Q_AUDIT_LOAD、已加载Q表及逐步Q_DECISION互证，不是只看feed_policy=q。

| 指标 | 本次结果 |
|---|---|
| 入箱 | 8/8正确，四箱各2，放错0 |
| Q决策/命中/未知回退/WAIT | 8 / 8 / 0 / 0；全部合法 |
| 真实出料顺序 | red, blue, green, yellow, red, blue, green, yellow；未强制首色 |
| 轨迹控制器Action | 149全部SUCCESS/error_code=0 |
| READY/槽释放 | 8/8；ROS及native原始窗口独立复核全部通过，无提前跨任务派发 |
| ARM_FAULT | 0 |
| 重试 | 1次，part_3 arm2 pre_grasp_failed，恢复后同任务重试 |
| 首PICKING至末READY | 155.080927墙钟秒 / 147.829仿真秒 |
| READY窗口native最大速度 | 1.07932e-13rad/s，未复现持续腕振荡 |

**结论：本次Q策略完成了真实八件功能闭环，但不是零异常验收，不能声称比Rule更快或当前Q表生产就绪。** 同为gain1的两轮并非预注册配对，墙钟/监测/宿主负载不可控，不以155.08对239.41计算策略收益。历史72ab三组低干扰配对仍是Rule184.59秒、Q187.96秒的独立结论。

## 不能隐藏的规划与接触问题

part_3第1次pre_grasp，MoveIt记录Found empty JointState message，CheckStartStateCollision报告arm2_gripper_base/gripper_base、arm2_wrist_2/wrist_2、arm2_wrist_3/wrist_3三对预测接触，MoveGroup Action ABORTED(status=6, code=99999)。这是规划起始状态拒绝，失败规划未发送该次轨迹；底层JTC没有失败结果。原有限恢复/重试产生第二次PICKING，同part/arm仍占同任务槽，未被新任务占用；随后实际成功。不能把这个MoveIt错误写成“所有Action都成功”，这里149成功仅指底层轨迹控制器。

空JointState与预测臂间碰撞同时出现，可能涉及MoveIt起始状态/规划场景同步；本轮未研究其因果，不把预测碰撞当作已发生物理臂间碰撞，也不将其忽略。若后续要求任何MoveGroup规划拒绝即停轮，应单独明确该诊断策略；本轮保留原有有界抓取恢复语义，没有放宽碰撞阈值或筛选性重跑。监督器对底层控制器失败、ARM_FAULT和不合法READY停轮；本次均未触发。

native接触613978个点行（不是独立碰撞次数）：finger_part595633最大0.014909m；finger_conveyor2043最大0.000001m；finger_bin16292最大0.000094m；另10个robot_part点，part_1与arm2腕/夹爪基座固定碰撞体接触，sim27.181–27.185秒，最大0.000146m。未记录臂间/主体撞箱类别，但不能据此宣称无接触或已消除所有碰撞风险。READY稳定窗口合格不证明抓放全过程无接触风险。

初始简易审计slot_order_ok=false来自把同件重试PICKING视为新任务；原派生JSON原样保留。独立逐任务复核q-lifecycle-review.json记录这一限制、同件重试、8次匹配READY/释放，校正后未发现跨任务早释放。没有修改原始JointState/CSV/日志来得到通过结果。

## 静态模型与协议边界

只读静态检查165种有效8件需求均合法完成，1822次模型决策中4次未知状态合法回退；这与本轮真实8决策0回退是不同样本。Q映射使用已观察状态/ETA、不读取未来；ARM_READY/FAULT由真实协议控制合法空闲槽，Q不能越过互锁。旧模型对物理READY/失败/新gain1时序仍不完整；DP是模型内信息上界，表标志deployment_ready=false/overlap_validated=false不改。

本轮未运行16组，不以一次Q完成作性能统计或长期可靠性认证。Rule已验收基线有效，建议Yuri先审核预测起始状态碰撞、接触限制、历史测试缺口，再另行决定扩展实验。
