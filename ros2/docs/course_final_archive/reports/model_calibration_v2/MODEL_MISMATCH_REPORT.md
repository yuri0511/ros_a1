# 模型与ROS事件边界审计

| 项目 | V1 | 实际ROS / V2处理 |
|---|---|---|
| 送料 | 4.2–4.44s均值＋后续1s假设 | 本机约6s至首次PICKING；保留生成、advance、回位home gate、合法派发/初始standby子成本。没有固定1s后续等待 |
| pick | 8.09–10.36s | 成功尾尝试约11.5–11.7s；失败尝试＋恢复另计，期间取件区与送料槽仍占用 |
| 离区 | 模型称safe exit | ROS是carry_clear后dispatcher释放锁，不代表任意姿态几何安全；V2只复现此门控语义 |
| POST/READY | 固定post到期必然READY | 记录ZONE→放置逻辑结束→return尝试→终态；READY定义为实际调度槽可用，不冒充物理已到standby |
| 失败 | 无 | 使用训练轮真实整件profiles携带重试前缀；不制造颜色失败概率。部分颜色零观察失败不等于p=0 |
| 重叠 | 一个pick区、两臂可post并行 | 单送料槽、单pick_owner、每臂一任务、prefeed=false保持；不模拟碰撞几何或证明全部重叠安全 |
| WAIT | next queued event | ROS只在ZONE_CLEAR/终态重新询问。新增内部放置事件不得唤醒Q；V2 WAIT跳过内部检查点至任务槽释放 |
| q_feed dummy1s | 临时ARM_READY队列 | 仅使WAIT合法，ROS不执行Model.step、不按该1s造真实终态；V2离线等待使用真实模拟完成时间 |
| 观测ETA | 固定post估计减经过时间再分箱 | V2仍用校准均值做代理，绝不把抽到的真实剩余时间或未来失败结果给Q |
| 精确DP | 可看完整队列 | 确定均值模型分别求time与time+.05idle最优；随机场景DP还知道预抽未来样本，只是额外信息上界 |

实际默认批次Q为0次WAIT，因此dummy1s并不是本轮耗时差的实测来源。V1决策点主要剩ARM_READY事件；V2新增内部检查点后必须跳过它们，避免引入额外决策。

q_feed本身未改。使用实验独立package_dir时，原加载接口可导入schema2模型/表；32条真实观察通过同一decide_feed映射，ETA key改变0、回退0。旧表metadata没有伪装成schema2，只在评估中按新合法动作过滤其原q_values作历史迁移参考。

关键缺口：原代码回待命失败仍终态释放槽，初始standby失败后也会继续。当前10字段观测没有独立fault/physical-ready状态。V2复现“调度合法”，不能校准或认证其物理安全性。不能靠提高训练回合数补上这些不可观测状态。

