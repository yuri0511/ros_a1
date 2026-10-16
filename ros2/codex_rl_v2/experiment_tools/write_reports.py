import json,statistics,math,csv,hashlib,datetime
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/model_calibration_v2');I=R.parent/'integration_72ab6f2';a=json.loads((R/'data/ASSESSMENT.json').read_text());c=json.loads((R/'training/config.v2.json').read_text());e=json.loads((R/'training/EVALUATION_V2.json').read_text());t=json.loads((R/'training/TRAINING_MANIFEST.json').read_text());rows=json.loads((R/'data/parts.json').read_text());batches=json.loads((R/'data/batches.json').read_text());manifest=json.loads((R/'data/SOURCE_MANIFEST.json').read_text())
def write(name,text):(R/name).write_text(text+'\n')
def f(x,n=3):return f'{x:.{n}f}'
def h(s):return hashlib.sha256(s.read_bytes()).hexdigest()
D=a['critical_chain_q_minus_rule'];stage={'feed':'首次请求→PICKING（含回位与派发）','pick_nominal':'最后一次PICKING→离区（成功尾尝试）','retry_prefix':'首次至末次PICKING（失败尝试＋恢复）','request_gaps':'离区→后续送料回调','terminal_tail':'最后离区→末终态'}
lines=['# 真实运行轨迹诊断','','本轮没有启动新Gazebo；只读取integration_72ab6f2的8轮原始日志、native stats、TaskStatus及CSV。64件逐件时间在CALIBRATION_DATA.csv，1177条带源日志行号事件在data/TRACE_EVENTS.csv。6个低干扰轮次用于策略诊断；两个精密轮单列。','','## 实际差异在哪里','','| 关键路径分量 | Q−Rule平均墙钟秒 | Q−Rule平均仿真秒 |','|---|---:|---:|']
for k,label in stage.items():lines.append(f'| {label} | {D["wall"][k]:+.3f} | {D["sim"][k]:+.3f} |')
lines+=['',f'关键路径合计墙钟差{D["wall"]["total"]:+.3f}s；再加启动差{D["startup_wall_s"]:+.3f}s、终态至CSV差{D["recorder_tail_wall_s"]:+.3f}s，得到整批{D["csv_measurement_wall_s"]:+.3f}s，与前报告184.59/187.96秒一致。仿真时长从首送料至末终态的差为{D["sim"]["total"]:+.3f}s，不与墙钟混算。','',f'Q7次、Rule3次重试；额外4次所对应的“记录前缀总差”跨3批为{D["wall"]["retry_prefix"]*3:.3f}墙钟秒，平均{D["wall"]["retry_prefix"]:.3f}s/批，大于净差3.37s/批。量级足够解释本轮慢，但成功尾段、RTF/启动和最终收尾抵消了一部分。前缀中包含失败动作和恢复，不能把它全部叫作松爪恢复时间。','',f'将记录的前缀作纯代数扣除后，Q−Rule残差为{D["wall"]["total_without_recorded_retry_prefix"]:+.3f}墙钟秒/批、{D["sim"]["total_without_recorded_retry_prefix"]:+.3f}仿真秒/批。这不是“消除重试后真实Q一定更快”的因果结论：恢复改变下一次起始姿态、工件状态和后续时序，不能保持其他段不变来宣称真实反事实。','','## 动作是否按模型执行','','全部实际8轮出料顺序与上一阶段相应模型策略的顺序一致。低干扰Q24次决策全部命中，另精密轮8次，均无未知回退/WAIT。Rule为blue/red/yellow/green/blue/red/yellow/green；旧Q为red/blue/green/yellow/green/blue/red/yellow。未发现“Q没接入”或错用规则回退能解释耗时。','','## 重试原因分布','','| 策略 | 颜色 | 原始原因 | 次数 |','|---|---|---|---:|']
for x in a['retry_reasons']:lines.append(f"| {x['policy']} | {x['color']} | {x['reason']} | {x['count']} |")
lines+=['','三组样本不能区分偶然重试、出料顺序相关的抓取条件变化与控制/感知时延。颜色需求相同，但不能因此认定失败独立同分布，也不能直接把相关性归为策略因果。优先证据是模型成本显著偏低且漏掉恢复；行动模型在当时的简化代价下最优，并不证明真实物理动作顺序最优。','','## 时间证据与口径','','为防止混合ROS旧clock快照和native插值产生负等待，持续时间统一使用同一native stats插值轴；每个精确收到的ROS header/首送料clock仍保存在独立ros_observed_sim列。以日志事件墙钟在约10Hz native stats中线性定位，不伪称精确物理事件戳；与保存的ROS戳最大观察偏差为'+f'{max(x["native_minus_ROS_stamp_max_abs_s"] for x in manifest.values()):.3f}'+'s，这不是严格误差上界。墙钟日志原值独立保留。原TaskStatus的STAMP也只是节点当时观察的/clock。','','推板“停稳”只有回到x阈值、发送停板并允许派发的日志，没有连续零速度证据；放置逻辑结束也不等于工件物理落稳。return_start是规划结束开始控制的点，规划耗时已计入整体return_tail。缺失的独立恢复起始点不补造。','','## 精密轮异常重新分类','','Q精密轮part_6 return_standby因MoveIt预测arm2_gripper_base与gripper_finger_left_link碰撞而拒绝规划，仍发布COMPLETED；part_8随后初始standby发生goal_time超时。此前将其合称“回待命超时”不够准确。这是规划防碰拒绝与下一任务的控制中止两件事，不是已发生双臂Gazebo物理接触的证据。原native接触记录无两臂接触，但有箱壁接触；两者不能混淆。']
write('REAL_TRACE_ANALYSIS.md','\n'.join(lines))
write('MODEL_MISMATCH_REPORT.md','''# 模型与ROS事件边界审计

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
''')
L=['# V2模型：有限时序校准报告','','基线72ab6f2；新独立分支perf/model-calibration-v2。实验源码位于worktree/ros2/codex_rl_v2/dual_arm_rl。原集成和性能工作树、旧Q、标记及历史数据均保留。schema_version=2；prefeed=false；单送料槽、取件区互斥及每臂单任务保持。没有新Gazebo。','','## 数据划分与参数','','先固定整轮划分再拟合：pair1/pair2双方共4轮32件训练，每色8件；pair3双方2轮16件留出；两个接触轮16件只诊断。留出结果在上阶段已公开，不是完全事前未知盲测；本轮未使用其数值拟合参数，也没有看其误差后调参。','','| 颜色/固定执行臂 | n | 请求至PICK | 成功尾尝试至离区 | 重试前缀均值 | 离区至COMPLETED |','|---|---:|---:|---:|---:|---:|']
for color,arm in zip(c['color_order'],('arm1','arm1','arm2','arm2')):
 d=c['stage_summary'][color];L.append(f"| {color}/{arm} | 8 | {d['feed']['mean']:.3f} | {d['pick_nominal']['mean']:.3f} | {d['retry_prefix']['mean']:.3f} | {d['post']['mean']:.3f} |")
L+=['','单位均为本机native插值仿真秒；每个参数保留source_case/part、n/min/max和原始事件行号。stage_summary同时有推进、回位home、放置逻辑结束和返回尾段。第一/第二次颜色出现位置的数据均保留，但模型当前使用颜色边际分布；不把第3次及以后放置位置当作已校准。','','## 事件与观测模型','','```mermaid','flowchart LR',' R[请求并占送料槽] --> A[生成及推进] --> H[回位位置门控] --> P[合法派发及PICKING]',' P --> V[失败尝试与恢复前缀] --> Z[carry_clear释放取件区和送料槽]',' Z --> N[若有合法空闲臂 可请求下一件]',' Z --> D[放置逻辑结束] --> B[回待命尝试] --> C[终态释放本臂任务槽]','```','','READY只指上述软件槽位语义；物理安全离区、真实零速度/standby证明未得到充分校准。V2将内部推进/home/place检查点与外部决策事件分开，WAIT只到真实槽位释放，Q观测仍为库存/READY或POST/ETA档/活动色十字段。ETA使用均值而非未来抽样成本；相关不泄漏及原q_feed兼容测试通过。','','## 三种情景的边界','','- nominal：纯敏感性，扣除观察到的失败前缀，保留最终成功尝试成本；不是物理上“如果没有失败会怎样”的真实反事实。','- mean：各色完整观察成本的均值，包含训练数据中的平均前缀。','- empirical_profiles（训练默认）：从每色8条真实整件记录均匀重采样，保留该件feed/pick/retry/post相关性。1/8只是明确的经验重采样假设，不是已可靠估计的物理失败概率；跨件/跨轮及顺序依赖未校准。','', '所有情景保留原idle_weight=.05。回调反应建模为瞬时，实际整批后续请求间隔仅数毫秒且低于插值分辨率；没有保留旧1s假设，也不声称真实延迟严格为0。随机profile取值不超训练记录范围，因此缺失的重尾/永久失败也不在模型里。','','## 按整轮留出验证','','给定真实已发生的出料顺序，用冻结参数预测整批首送料至终态，不使用留出重试数量或成本。','','| 集合/模型 | 轮数 | MAE仿真秒 | 有符号偏差 | 最大绝对误差 |','|---|---:|---:|---:|---:|']
for split,g in a['validation'].items():
 for mode,v in g.items():L.append(f"| {split}/{mode} | {v['n']} | {v['mae_sim_s']:.3f} | {v['bias_sim_s']:+.3f} | {v['max_abs_error_sim_s']:.3f} |")
L+=['','留出Rule实际156.031s，均值预测168.488s；留出旧Q实际163.507s，均值预测164.015s。误差下降支持主要时序校准有用，但不能证明失败过程准确，也不能仅因两个误差较小宣称一般化完成。','', '另用真实每件成本强制回放，只验证事件账本：6轮残差约3–7ms（主要为请求回调间隔）。这是使用真实结果的teacher forcing，不是留出预测；没有用它来宣称模型泛化精度。','','## 仍不足的数据','','没有连续pusher零速度证据；ZONE_CLEAR仅既有几何轨迹门控；没有可靠的物理READY故障状态/转移模型；仅7个训练失败尝试、四轮，不足以分离策略/颜色/位置/负载的失败概率，且部分颜色0次失败。未观测第3+同色件、其他需求、首件yellow或任意新的交互轨迹；通用表这些场景依赖外推假设。需先补测，不把模型最佳当成Gazebo认证。']
write('MODEL_V2_REPORT.md','\n'.join(L))
L=['# V2离线训练与独立评估','','两张表从空表分别训练50,000回合，seed=42，alpha=.2、gamma=1、epsilon线性.5→.05，未追加回合、未筛选种子重训。训练情景为经验整件profile重采样；没有运行真实Gazebo训练。','','| 表 | 状态数 | 回合 | 训练墙钟秒 | 配置canonical SHA256 |','|---|---:|---:|---:|---|']
for d in t:L.append(f"| {d['name']} | {d['states']} | {d['episodes']} | {d['wall_s']:.2f} | {d['config_sha256']} |")
L+=['',f'配置文件字节SHA256：`{t[0]["config_file_sha256"]}`。两张表各自文件哈希、参数和源文件哈希见TRAINING_MANIFEST.json及SOURCE_CODE_HASHES.json。deployment_ready/overlap_validated保持false；旧表未覆盖。','','## 独立种子/场景','','评估使用202700..202899共同场景，独立重复使用303700系列及313700系列；与训练seed不同。每色同一ordinal的未来profile预抽并对普通策略隐藏。共同场景对比控制随机样本；独立场景只作随机波动敏感性。','','| 策略 | 共同200场景平均仿真秒 | 标准差 | 未知/决策 |','|---|---:|---:|---:|']
for name,v in e['empirical_common_scenarios'].items():L.append(f"| {name} | {v['mean']:.3f} | {v['sd']:.3f} | {v.get('unknown_fallbacks','—')}/{v.get('decisions','—')} |")
L+=['','默认V2表在该共同场景下与Rule相同时间，没有模型提速；通用V2表与旧Q相近，并未超过旧Q。本次默认表均值场景未达DP最短时间，尚无收敛证明；不通过继续训练/挑种子制造优胜。通用表最短时间较好也不能当作物理提速或自动改为生产默认。','','## 其他8件需求与合法性','','165种总数8的库存全部完成模型任务。','', '| 表 | 未知回退/决策 | 回退率 |','|---|---|---:|']
for name in ('q_v2_demo','q_v2_general'):
 g=[x for x in e['coverage'] if x['name']==name];u=sum(x.get('unknown_state_fallback_steps',0) for x in g);n=sum(x.get('decision_steps',0) for x in g);L.append(f'| {name} | {u}/{n} | {100*u/n:.3f}% |')
L+=['','全部评估动作均断言属于当前合法动作；这只证明资源门控合法，不证明几何安全。通用表第3+同色件虽能执行，但对应放置/恢复时长未有实测支持。32条旧ROS观察经原decide_feed与新schema2包映射一致，ETA key未变、回退0；这是适配器观察测试，不是新策略轨迹的Gazebo回放。','','复现：python3 tools/freeze_model.py；python3 tools/train_v2.py；python3 tools/evaluate_v2.py；python3 tools/adapter_audit.py；python3 tools/assess.py。代码生成在临时目录重建后字节一致，见audit/REPRODUCIBLE_GENERATION.json。普通训练/选择只读state()/legal_actions，未来样本与精确事件队列只由环境推进和oracle DP访问。']
write('TRAINING_V2_REPORT.md','\n'.join(L))
L=['# V2模型 DP 与信息条件','','主要表为确定的完整观察成本均值情景；耗时与训练奖励目标time+.05×demand-idle分别求解，不把二者最优混称。','','| 策略 | 仿真耗时 | 加权代价 | 距最短时间DP | 距加权DP |','|---|---:|---:|---:|---:|']
for name,v in e['mean_models']['mean'].items():L.append(f"| {name} | {v['time']:.3f} | {v['cost']:.3f} | {v['gap_makespan']:.3f} | {v['gap_weighted_cost']:.3f} |")
L+=['','V2默认表未胜Rule，均值时间落后最短时间DP4.472s；通用表达到164.015s的最短时间，但不是169.693加权代价的最优。旧Q按新合法动作过滤后也达到164.015s，因此不能宣称新训练击败旧Q。加权DP耗时164.108s，稍长于time-DP，是idle权重的真实取舍，不是数据错误。','', 'DP复用V2全部合法动作与step，包含WAIT、单槽/单pick-owner/每臂状态、完整内部事件相对时间、活动profile及逐色ordinal。不是只枚举颜色顺序。小批次独立无缓存穷举与DP一致，包含强制WAIT和主动WAIT；完整状态缓存不使用Q的ETA离散key。','','## 随机场景 oracle 的额外信息','','20个独立固定场景中，oracle还知道每个颜色/ordinal将抽到的未来profile及精确事件队列，而Q不知道。默认Q平均距该最短时间oracle约'+f'{statistics.mean(x["gap"] for x in e["clairvoyant_oracles"]):.3f}'+'s。这个差距包含额外信息价值，不能全当作Q学习误差；oracle不是普通观测条件下的最优可部署策略。每场景完整记录在EVALUATION_V2.json及ORACLE_GENERAL_AUDIT.json。','','均值场景/nominal前缀扣除敏感性/经验随机场景分别归档；不能把确定均值模型DP当作带未知失败和物理碰撞系统的全局最优，也没有求出原POMDP在所有可能真实失败分布下的最优。','','## 观测与合法性测试','','- 不同未来profile银行在相同当前观察下的state/合法动作相同。','- POST ETA用校准均值而非抽到的真实返回时间；不向Q泄漏未来失败。','- WAIT跳过放置等内部事件，至真实ARM_READY才重新决策。','- 全部评估动作属于资源合法集合；native adapter32条观察重建一致。','- 原table/config不改metadata伪装成schema2；旧Q只是明确标记的Q值迁移参考。']
write('DP_V2_REPORT.md','\n'.join(L))
N5=math.ceil(((1.96+.8416)*a['pilot_sd_wall_s']/5.0)**2)
write('NEXT_GAZEBO_TEST_PLAN.md',f'''# 下一轮最小补测及物理验证计划（本轮不执行）

目前不建议以“新Q必然更快”为前提直接大量采样。默认V2表没有模型收益；通用表只在模型内较好，未超旧Q，且完整故障/安全READY未校准。

## 先补1个诊断配对，2个批次

单实例Rule/待测Q各8件，独立高精度记录，排除出正式吞吐统计。先冻结候选和schema2/config/table hash。通用表首件red在旧数据有覆盖；默认表首件yellow尚未覆盖，若选择它必须单独功能/接触验收。保持1ms、v3、无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2，不改任何几何/容差/规划/控制参数。

只增加有界、可观测事件记录：

1. 送料请求、spawn、推进停止、pusher真实位置/速度及零速保持、stop命令、允许派发，均带发出节点的sim+monotonic戳；不再靠跨节点插值判断停稳。
2. 每次attempt起止、失败原因、recover起止、锁保持区间；把失败动作与恢复分开，保留整件/整轮关联。
3. carry_clear、调度锁释放、放置逻辑结束与工件物理落稳分别记录。连续关节/接触证据确认覆盖轨迹，不把carry_clear语义扩大。
4. return计划/控制开始结束、目标误差、at_standby和终态；记录“终态但物理未就绪”，不能默默当READY。发现同类规划拒绝/standby失败则停止新Q推广，先修订可观测故障门控及重新建立对照；不改控制器容差掩盖。
5. 每次Q观察/ETA/合法动作/选择/WAIT起点及真实唤醒事件；确认原adapter dummy1s从未当成现实定时器推进。

本诊断对只能补通道和边界，不足以给可靠失败概率，尤其不能以零次异常认证安全。

## 若继续检验性能：建议至少16个新的低干扰配对

三组已有配对差值标准差{a['pilot_sd_wall_s']:.3f}墙钟秒。基于双侧5%、80%功效的粗略正态规划：目标6.09s（旧模型约3.3%）需约{a['normal_approx_pairs_for_80pct_power']}对，目标5s（约2.7%，接近V2通用表预测幅度）需约{N5}对。考虑小样本t检验和仅3对估计方差的脆弱性，分别保守安排12或16对；这不是保证功效的精确最小样本定理。

因此当前建议：先1个诊断对；通道/READY语义明确后，若仍值得检验约5秒的改善，再预注册16个新的正式对。8对Rule→Q、8对Q→Rule，随机顺序seed=2027，独立新世界/同一初始SDF/同一四色库存；控制并记录可用随机种子、CPU/RTF，承认OS调度无法完全固定。单实例策略对照，双实例仅保留给后续吞吐实验。

冻结模型后这些新正式轮次全部作前瞻留出，不再回填本轮训练。前6对只可盲看方差/数据质量按预先规则重估样本量，不按Q是否领先调参/挑种子/加回合；保留失败、重试、超时、OUTSIDE和清理时间。报告完整批次墙钟、统一仿真边界、成功率、恢复成本和接触异常，不能只选成功快轮。

## 未校准与风险

- 仅4个训练轮的7次失败尝试，集中4个工件；同件可多次失败，7/32不是独立抓取失败概率。跨轮、先后顺序、箱格/姿态和负载依赖未知。
- 现有反馈READY不是物理到位保证，实际发生过防碰拒绝后继续任务；这属于优先安全缺口。DP与Q没有几何碰撞模型。
- 新首色或相邻颜色组合会改变轨迹交互；已有上臂–箱壁接触和规划碰撞预测不能忽略。
- 通用8件中的第3+同色件无对应实测位置成本；超出默认2222只能称模型可执行，不称已校准。

本轮没有修改deployment_ready/overlap_validated，也没有自动启动本计划的任何Gazebo实验。
''')
write('data/DICTIONARY.md','''# 原始轨迹字段说明

CALIBRATION_DATA.csv每行一件，共64行。split为calibration/holdout/diagnostic_contact_only，完整轮次从不拆到不同集合。

*_wall为原节点日志墙钟epoch；*_sim是由native stats在日志时间处线性定位的统一仿真估计；*_ros_observed_sim仅在有精确保存的TaskStatus/首送料clock时填写。*_sim_source与*_log_line给出处；TRACE_EVENTS.csv另保留插值包围跨度，不能把它当成严格精度保证。缺失return_controller_start可表示规划未成功，不能填0。

picking_success_attempt是最后一次PICKING，而非第一遍成功反事实。retry_prefix=首次PICKING→末次PICKING，含失败尝试与恢复。单独恢复起点并无完整埋点，不能给每类编造恢复秒数。zone_clear用dispatcher实际锁释放日志，zone_status保留执行器状态消息；跨线程publish/log顺序不同，不混为同一物理瞬间。

place_finished是待收尾的逻辑COMPLETED日志；completed是真正发布的终态。pusher_home是位置阈值/停板命令，pusher_velocity_zero_evidence=false。所有持续时间另列wall_s/sim_s，不换算混加。
''')
print('Seven reports generated')
