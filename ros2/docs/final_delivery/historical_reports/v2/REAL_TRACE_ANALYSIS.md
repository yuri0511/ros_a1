# 真实运行轨迹诊断

本轮没有启动新Gazebo；只读取integration_72ab6f2的8轮原始日志、native stats、TaskStatus及CSV。64件逐件时间在CALIBRATION_DATA.csv，1177条带源日志行号事件在data/TRACE_EVENTS.csv。6个低干扰轮次用于策略诊断；两个精密轮单列。

## 实际差异在哪里

| 关键路径分量 | Q−Rule平均墙钟秒 | Q−Rule平均仿真秒 |
|---|---:|---:|
| 首次请求→PICKING（含回位与派发） | -0.856 | +0.036 |
| 最后一次PICKING→离区（成功尾尝试） | -5.531 | -1.972 |
| 首次至末次PICKING（失败尝试＋恢复） | +13.417 | +13.879 |
| 离区→后续送料回调 | +0.003 | +0.002 |
| 最后离区→末终态 | -2.818 | -3.477 |

关键路径合计墙钟差+4.215s；再加启动差-0.843s、终态至CSV差+0.000s，得到整批+3.373s，与前报告184.59/187.96秒一致。仿真时长从首送料至末终态的差为+8.468s，不与墙钟混算。

Q7次、Rule3次重试；额外4次所对应的“记录前缀总差”跨3批为40.252墙钟秒，平均13.417s/批，大于净差3.37s/批。量级足够解释本轮慢，但成功尾段、RTF/启动和最终收尾抵消了一部分。前缀中包含失败动作和恢复，不能把它全部叫作松爪恢复时间。

将记录的前缀作纯代数扣除后，Q−Rule残差为-9.202墙钟秒/批、-5.410仿真秒/批。这不是“消除重试后真实Q一定更快”的因果结论：恢复改变下一次起始姿态、工件状态和后续时序，不能保持其他段不变来宣称真实反事实。

## 动作是否按模型执行

全部实际8轮出料顺序与上一阶段相应模型策略的顺序一致。低干扰Q24次决策全部命中，另精密轮8次，均无未知回退/WAIT。Rule为blue/red/yellow/green/blue/red/yellow/green；旧Q为red/blue/green/yellow/green/blue/red/yellow。未发现“Q没接入”或错用规则回退能解释耗时。

## 重试原因分布

| 策略 | 颜色 | 原始原因 | 次数 |
|---|---|---|---:|
| q | green | pre_grasp_failed | 3 |
| q | green | grasp_not_verified | 2 |
| q | red | grasp_not_verified | 1 |
| rule | red | pre_grasp_failed | 2 |
| rule | red | part_not_under_gripper | 1 |
| q | red | part_not_under_gripper | 1 |

三组样本不能区分偶然重试、出料顺序相关的抓取条件变化与控制/感知时延。颜色需求相同，但不能因此认定失败独立同分布，也不能直接把相关性归为策略因果。优先证据是模型成本显著偏低且漏掉恢复；行动模型在当时的简化代价下最优，并不证明真实物理动作顺序最优。

## 时间证据与口径

为防止混合ROS旧clock快照和native插值产生负等待，持续时间统一使用同一native stats插值轴；每个精确收到的ROS header/首送料clock仍保存在独立ros_observed_sim列。以日志事件墙钟在约10Hz native stats中线性定位，不伪称精确物理事件戳；与保存的ROS戳最大观察偏差为0.052s，这不是严格误差上界。墙钟日志原值独立保留。原TaskStatus的STAMP也只是节点当时观察的/clock。

推板“停稳”只有回到x阈值、发送停板并允许派发的日志，没有连续零速度证据；放置逻辑结束也不等于工件物理落稳。return_start是规划结束开始控制的点，规划耗时已计入整体return_tail。缺失的独立恢复起始点不补造。

## 精密轮异常重新分类

Q精密轮part_6 return_standby因MoveIt预测arm2_gripper_base与gripper_finger_left_link碰撞而拒绝规划，仍发布COMPLETED；part_8随后初始standby发生goal_time超时。此前将其合称“回待命超时”不够准确。这是规划防碰拒绝与下一任务的控制中止两件事，不是已发生双臂Gazebo物理接触的证据。原native接触记录无两臂接触，但有箱壁接触；两者不能混淆。
