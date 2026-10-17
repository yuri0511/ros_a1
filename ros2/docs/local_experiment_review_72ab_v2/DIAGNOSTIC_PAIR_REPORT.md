# V2最小Gazebo诊断配对报告

本轮只启动一次Rule完整仿真，触发安全保护后停止；未启动Q轮，未重跑失败轮，未启动16组。不是吞吐样本，也不是完整Rule/Q策略性能对比。

## 环境与候选

上游72ab + 已归档V2快照e2a82c9，独立诊断工作树。1ms、v3事件等待、无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2、单实例。原几何/碰撞/控制参数/成功判据保留。Rule显式feed_policy:=rule，启动确认“退回谁闲推谁”。

V2 general schema2、prefeed=false、canonical配置哈希ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b，与表一致；实际Python模块来自本次诊断codex_rl_v2。37项相关回归通过；165种8件需求动作合法且模型完成；一般库存4次未知回退已如实保留。初始默认库存纯接口选择red且命中，未强制首色。以上均为前置检查，**本轮Q在Gazebo内决策为0，加载成功尚无本轮物理证据**。没有重新训练。

## 实际结果

| 项目 | Rule | q_v2_general |
|---|---|---|
| 运行 | 20261016T171634-diag-rule | 安全条件触发，未启动 |
| 8件验收CSV | 0行；0个完整成功批 | 无物理结果 |
| 已请求来料 | blue、red（第二件在故障前已按ZONE_CLEAR合法进入取件阶段） | 无 |
| 抓取重试 | 0（截止停轮，第二件尚未完成抓取） | 无 |
| 控制器/返回 | return_standby报告成功，但新鲜速度未静止 | 未验证 |
| 新增批次耗时比较 | 不适用：提前安全停止 | 不适用 |

part1蓝件已有待收尾COMPLETED日志，但未发布TaskStatus COMPLETED、CSV仍仅表头。原生最后位置约(0.277869,0.236795,0.745000)，位于蓝箱；最后1仿真秒位置范围见JSON。这不是完整8件验收或READY证明。

## 触发点：成功接口与物理静止不一致

故障墙钟Unix 1792142237.332762003，ROS仿真时刻36.921s；从启动到触发42.777墙钟秒。arm1返回控制器成功后，_at_standby=True，而wrist_3_joint速度 **2.885672rad/s**，原静止限值0.3rad/s。该关节位置误差仅0.002886rad，关节缓存年龄约1.073ms；位置已接近目标，不能把此结果称为大幅偏离待机或本轮规划失败。

独立5Hz原始JointState：触发前约0.092s速度2.999634rad/s，触发后约0.108s速度2.885672rad/s。说明不止警报日志自己报告一个值；但采样仍不足以区分高频数值振荡、速度报告语义或持续运动。不得推断一定发生碰撞或控制器硬故障。本轮没有记录return_standby规划拒绝/goal_time abort；此类失败在历史72ab Q精密轮中另有证据。

诊断保护在return方法边界同步阻塞该任务线程，**没有flush终态/释放本臂执行槽**，监督器暂停服务返回data:true。故障到暂停确认0.390墙钟秒，最后原生pose仿真时刻37.201s；新请求在故障后为0。另一臂先前已在执行，保护不宣称零检测延迟。完整进程生命周期56.660墙钟秒包含关闭清理；不可当作完整批次耗时。后台父进程正常停止，scene_publisher关闭超时由原launch升级信号；这不是运动控制超时。

## 接触、推板与落稳证据

记录原生接触点55064行；双臂接触0观测、非夹爪臂身-箱体接触0观测。仍有夹爪手指-箱壁接触：最大0.299mm，最长观测接触段2.310仿真秒；手指-工件最大深度11.448mm，以及手指-输送结构接触。数值原样保留，不据此宣布接触安全。接触点数不是独立碰撞数；段间<=40ms连接只作观测跨度。

推板真实世界pusher_link pose、有限差分速度、速度命令、home阈值日志均保存。原SDF没有推板JointStatePublisher，本轮未改插件，因此未获得精确native推板关节速度；不以速度命令或home日志冒充零速度证明。工件实际pose轨迹保存，结果记录器仍保留原12秒回收逻辑。

## 审阅建议与下一步

不推荐16组正式配对。先让Yuri审阅：①控制器SUCCESS与实际速度静止的关系，必要时设计有界连续新鲜采样的READY条件；②返回/初始待机失败后的明确FAULT、禁止槽位释放及复位恢复；③真实关节高频速度及夹爪-箱壁接触的可接受标准。不能只扩大容差或忽略异常；正式状态机修复需另行审核，本轮没有擅自修复控制语义。

V2通用Q是否快于Rule仍未知；未完成的诊断不支持任何新策略性能收益结论。deployment_ready/overlap_validated均为false。

可复核文件：diagnostic/audit/diagnostic-result.json、preflight.json、guard-tests.json、tests.log；diagnostic/runs下launch.log、guard-fault.json、diagnostic.jsonl、observations.jsonl、stats.jsonl、接触CSV与原始ROS日志（大文件在独立Evidence ZIP）。
