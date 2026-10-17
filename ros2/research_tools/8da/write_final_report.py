import json,csv,statistics,datetime
from pathlib import Path
from zoneinfo import ZoneInfo
R=Path('/home/yu/yuri_gazebo_performance/fast_sim_8da998b');cases=[]
for p in sorted((R/'runs').iterdir()):
 if (p/'analysis.json').exists() and (p/'extra-analysis.json').exists():
  a=json.loads((p/'analysis.json').read_text());a['meta']=json.loads((p/'metadata.json').read_text());a['extra']=json.loads((p/'extra-analysis.json').read_text());cases.append(a)
by={x['label']:x for x in cases}
def ok(a):return a['reason']=='completed' and a['successful_batches']==a['n'] and not a['swapout_delta'] and not a['oom_delta'] and not any(i['interlock_violations'] or i['vision_mismatches'] or i['isolation']['ros_foreign'] or i['isolation']['gz_foreign'] for i in a['instances'])
def fmt(x,n=2):return '—' if x is None else f'{x:.{n}f}'
def mm(x):return fmt(None if x is None else x*1000,3)
low=[a for a in cases if a['contact_mode']=='0' and not a['meta'].get('fixture')=='1'];single=[a for a in low if a['n']==1 and ok(a)]
fast=min(single,key=lambda a:a['experiment_wall_s']);v3single=[a for a in single if a['label'].startswith('B3-')];groups={}
for a in low:
 if a['label'].startswith('P') and '-v3-' in a['label']:groups.setdefault(a['n'],[]).append(a)
qualified={n:g for n,g in groups.items() if len(g)>=2 and all(ok(a) for a in g)}
if len(v3single)>=2:qualified[1]=v3single
pooled=lambda g:3600*sum(a['successful_batches'] for a in g)/sum(a['experiment_wall_s'] for a in g)
if qualified:best_n,best_g=max(qualified.items(),key=lambda kv:pooled(kv[1]))
else:best_n,best_g=1,v3single
best_rate=pooled(best_g);life_rate=3600*sum(a['successful_batches'] for a in best_g)/sum(a['lifecycle_wall_s'] for a in best_g)
A,B=by['A-1ms-original'],by['B-1ms-event'];gain=1-B['experiment_wall_s']/A['experiment_wall_s']
S=[]
def add(s=''):S.append(s)
add('# 8da998b 高速仿真优化与数据有效性实测报告')
add();add('本机 Ubuntu 24.04.5 / 12 vCPU / 24 GB VMware 虚拟机实测。日期：'+datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y-%m-%d %H:%M %Z')+'。没有启动强化学习，没有重训Q表。')
add();add('## 主要结论')
add();add(f"- 共同基线为完整 `8da998b963eed3c70fda076329f04a75c8627ef3`，独立分支 `perf/fast-simulation-8da998b`。原 `083a515` 工作区、旧实验和锁定环境保留，最终完整性检查见 `audit/original-integrity-final.json`。")
add('- Q接口及q_feed.py保留；实验显式将三个Q路径置空，q_batch=[2,2,2,2]，日志确认“规则：谁闲推谁；Q表显式关闭”。不是依赖缺文件自动回退。')
add(f"- 初始匹配对照：A {A['experiment_wall_s']:.1f}s → B {B['experiment_wall_s']:.1f}s，8/8均正确，**墙钟缩短 {A['experiment_wall_s']-B['experiment_wall_s']:.1f}s（{gain*100:.1f}%）**；稳态RTF {A['instances'][0]['actual_rtf']:.3f}/{B['instances'][0]['actual_rtf']:.3f} 接近。它是本轮一次成功配对的观测，不能当作所有机器/负载的固定增益。")
add('- 2ms在低干扰完整批次中，原等待仅1/8正确，事件等待0/8正确。RTF虽提高，**成功完整批次均为0**；独立接触诊断又出现永久失败。本轮拒绝将2ms作为可用高速配置，未通过放宽阈值、改锚点/增益/速度修补结果。')
add(f"- 已成功完整验证的单实例最快观察值：**{fast['experiment_wall_s']:.1f}s**（{fast['label']}，稳态RTF {fast['instances'][0]['actual_rtf']:.3f}）。v2清理临时RGB订阅后完成两轮8/8，但精密轮启动检查超时、未送料；v3有界重试和独立发现核验后重新验收。最快观察值不等于当前v3的保证值，版本和探针分组不混算。")
add(f"- 已完成复测且全部批次正确的最佳已测配置：**{best_n}实例、1ms、事件等待、无GUI/RViz、X11/llvmpipe、LP_NUM_THREADS=2**。{len(best_g)}轮共成功{sum(a['successful_batches'] for a in best_g)}批，合计{sum(a['experiment_wall_s'] for a in best_g):.1f}s，**{best_rate:.2f}成功批次/小时**；含退出/采样清理为{life_rate:.2f}/小时。")
add(f"- 相比同提交A的{A['episodes_per_hour']:.2f}/小时，上述实测率高{(best_rate/A['episodes_per_hour']-1)*100:.1f}%。这包含实例并行与等待策略；不能全部归因于某一个Python改动。未连续运行一整小时，不声称长期极限或零故障。")
add();add(f"与旧083a完整GUI历史244.04秒（{3600/244.04:.2f}批/小时）相比，最佳已测吞吐提高{(best_rate/(3600/244.04)-1)*100:.1f}%；这是跨提交、GUI、等待和并行设置的整体对比，不能当作单因素因果收益。")
add();add('## 版本、环境与基线保护')
add();add('远端检查时分支已进一步前进到72ab6f2，按用户指定固定8da998b。原验收目录 `/home/yu/ros_a1/ros2` 仍为083a515；新目录为 `/home/yu/yuri_gazebo_performance/fast_sim_8da998b/worktree/ros2`。Git HEAD表示完整上游基线，本地优化尚未提交；实际代码以每轮source_hash和v1/v2/v3差异为准。没有git reset --hard、切换运行中的工作树或GitHub推送。')
add();add('复用 `/home/yu/ros_a1/ros2/.pixi/envs/default`，独立build/install，四包编译通过。ROS2 Jazzy、Gazebo Harmonic8.10.0、MoveIt2.12.4、Pixi0.82.0及锁文件未升级。原世界1ms保留；实验world只改目标RTF为4及候选步长1/2ms。机器人、夹爪、碰撞/接触参数、控制器500Hz/增益、规划参数、轨迹目标、速度/加速度缩放、传感器和判据不改。')
add();add('完整读取了README_RUN第6节、q_feed/test_q_feed、HANDOFF/REVIEW、CONTACT_ROUNDS、record_contacts和mesh_probe。Q接口尚非Gazebo Q策略部署验收；外部JSON不在该分支。文档叙述overlap_validated=true与旧JSON false不同；未修改任何deployment_ready/overlap_validated字段。')
add();add('回归：原始资料检查145通过、8失败；4项缺外部dual_arm_rl/Q JSON，3项缺G0锚点证据JSON，1项旧relay键集合测试与现有split_plan_execute/controller_action配置冲突。保留原测试和错误证据；有前置资料的子集及新增事件等待安全测试150通过、8项明确排除。v2、v3修正后均再次150通过。缺资料不解释为Q算法损坏，也不造假表补齐。')
add();add('## 测量方法')
add();add('共同目标RTF=4、LP线程2、Fast DDS、无GUI/RViz、X11 llvmpipe。每轮新进程/新世界，四色各2件。完整成功需唯一part_1..8全部PLACED、最终箱等于目标箱、四箱各2件。结果记录器会提前多次打印“本轮结束”，实验器只按8条真实CSV与终态判定。')
add();add('吞吐率=3600×真实成功完整批次数/首实例启动至最后终态的整组墙钟。包含慢实例拖尾；含退出清理另列。失败件数、未完成诊断和CPU保护中止不冒充完整成功。native stats记录sim_time/real_time/iterations/step_size；稳态RTF为每实例启动30秒后至终态的差分，全stats区间另存，5秒窗口P10/P90描述波动。不是目标值或多个RTF之和。')
add();add('低干扰组不运行高频接触CSV/位姿跟踪；仅native stats、图像/逻辑相机/隔离计数，每轮约第40秒额外10秒采样ROS时钟/关节。仍有小量观测开销。精密接触组独立运行，在Gazebo之前订阅并记录首个时钟、心跳、接触及回收前原生位姿；其墙钟不并入性能结论。低探针clock=0代表未连续订阅，事件sim时间使用消息stamp或native gz_clock，不能误读为仿真停钟。')
add();add('原记录器在成功后12个仿真秒回收工件，保持不变。低干扰组全件位置取原记录器完成时XY（毫米舍入），高精度组取原有回收前最后原生XYZ；并不是批次末尾八件同时在场。只有成功PLACED计入放置精度，失败目标/位置单独保留。')
add();add('## 完整性能对照')
add();add('| 运行 | 步长/等待/版本 | 批次成功 | 正确件/记录件 | 整组墙钟s | 成功批次/h | 各实例稳态RTF | CPU均值/P95 | 自动重试 |')
add('|---|---|---:|---:|---:|---:|---|---|---:|')
for a in low:
 if a['meta'].get('fixture')=='1':continue
 count=sum(sum(r['result']=='PLACED' and r['final_bin']==r['target_bin'] for r in i['results']) for i in a['instances']);total=sum(i['row_count'] for i in a['instances'])
 version='v3' if a['label'].startswith('B3') or '-v3-' in a['label'] else ('v2' if a['label'].startswith('B2') or '-v2-' in a['label'] else 'v1')
 add(f"| {a['label']} | {a['step_ms']}ms/{'事件' if a['fast_waits']=='true' else '原始'}/{version} | {a['successful_batches']}/{a['n']} | {count}/{total} | {a['experiment_wall_s']:.1f} | {a['episodes_per_hour']:.2f} | {', '.join(fmt(i['actual_rtf'],3) for i in a['instances'])} | {fmt(a['cpu_mean_percent'],1)}/{fmt(a['cpu_p95_percent'],1)}% | {sum(i['retries'] for i in a['instances'])} |")
add();add('每实例墙钟、批次仿真时长、全区间/稳态RTF、5秒P10/P90、CPU/内存/Swap、位置、原生相机/关节频率及所有故障见EXPERIMENT_MATRIX.csv、analysis.json、extra-analysis.json。单进程CPU的100%为一vCPU，整机100%为12vCPU；RSS有共享页，不应直接相加当作真实物理占用。')
add();add('初始A/B由日志近似配对的MoveIt规划墙钟中位数约0.60/0.59毫秒、P95约1.6/1.5毫秒；同轮控制器目标执行中位数约0.649/0.644秒。v3单实例约0.49毫秒规划/0.59秒控制器，首轮双实例约0.62–0.66毫秒规划/0.79–0.80秒控制器。日志配对并非完整分布式追踪；这些样本不支持将规划搜索认定为主要瓶颈。双实例RTF降至约0.70–0.72，原生相机仍约10仿真Hz，关节观察约498仿真Hz；并行增加资源竞争和恢复成本。物理与软件渲染同处Gazebo进程，进程CPU不能再精确拆成两者的独立耗时。')
add();add('2ms的C/D实测RTF比1ms高，但失败后实际运动负载也改变，因此不能把全部差异因果归于步长。错误包括跟随方向/位移不符、工件不在夹爪下、偏心及Pilz关节加减速/IK规划失败；C在未启用Python优化时也失败，不能归咎于事件等待一项。500Hz控制周期与2ms相等并不证明数值/时序兼容。')
add();add('## 五处等待的实测与替换证据')
add();add('| 等待 | A次数/墙钟合计s | B次数/墙钟合计s | 处理 |')
add('|---|---|---|---|')
notes={'first_feed':'20秒改为全部就绪才触发，60秒墙钟上限；不就绪拒绝送料','dispatch_delay_path':'默认并联来料本已事件驱动；未触发的1秒不计收益','grasp_extra':'原样本/时间跨度/新鲜相机证据充分且原GraspVerifier通过才提前；不足仍等满原1.5秒','release_settle':'保留0.4秒；仅达到52mm间隙不足以证明释放稳定','drop_return_branch':'默认标定放置不走该分支；原1.5秒保留，不计收益'}
for label,note in notes.items():
 x=A['extra'][0]['waits'][label];y=B['extra'][0]['waits'][label]
 add(f"| {label} | {x['count']} / {x['wall_sum_s']:.4f} | {y['count']} / {y['wall_sum_s']:.4f} | {note} |")
add();add('每次WAIT_METRIC原始记录给出实际墙钟、样本数、真实采样跨度、不同相机时间戳数，以及是否由充分证据结束。正常探针已完成后，采样从闭爪前累积，满足至少ceil(span/0.05)+1对、跨度≥span、至少ceil(span×10)个不同相机帧且新鲜，原验证器通过，才结束追加静止尾段；未删除探针/验证/在场闸/重试。')
add();add('MotionClient另将Future完成与执行槽状态改成Event通知，仍保留所有受理/执行/排空/执行槽墙钟超时及原判据。不把A/B总差额强行分摊为某个轮询收益。原attach跟随40Hz墙钟、闭爪阶段及其他条件/故障等待仍存在；没有把故障超时全部换成仿真时钟。每处改动的状态依据、上限和失败路径见CHANGELOG.md。')
add();add('v2仅在就绪后释放自身临时RGB检查订阅，日志STARTUP_OBSERVER_RELEASED确认；相机传感器、原视觉分类和性能探针继续接收。此修正旨在消除诊断开销，不以两轮波动宣称它单独带来固定提速。')
add();add('v2精密轮未发料即60秒超时，计为启动失败。独立诊断复现Python GZ topic_info漏报订阅；v3增加2秒控制器RPC超时重试、3秒新鲜回复要求及同分区有界CLI真实订阅核验，保留10项条件和60秒总上限。此问题由本地优化的就绪检查引入，不能算作原Q/物理代码损坏。失败轮、空载诊断和源码差异完整保留。')
add();add('## 接触、位置与可靠性')
add();add('本机1/2ms各自做了两臂肩部网格正向对照，临时方盒产生真实接触并已删除，记录从首个native时钟开始。原Yuri两轮接触及全网格正向证据单独汇总（audit/yuri-*-summary.json），不与本机轮次合并。正式场景没有这些方盒。')
add();add('| 精密批次 | 结果 | 接触点/坏行 | 两臂接触点 | 手指-件最大mm | 手指-带最大mm | 手指-箱最大mm | 臂/底座-箱最大mm | 成功放置最大XY误差mm | 原生控制中止次数 |')
add('|---|---|---|---:|---:|---:|---:|---:|---:|---:|')
for a in cases:
 if a['contact_mode']!='1' or a['meta'].get('fixture')=='1':continue
 for idx,i in enumerate(a['instances']):
  p=Path(i['path'])/'contacts-task-summary.json';co=json.loads(p.read_text()) if p.exists() else {};cat=co.get('categories',{});e=a['extra'][idx]
  d=lambda key:cat.get(key,{}).get('max_depth_m')
  add(f"| {a['label']} | {sum(r['result']=='PLACED' for r in i['results'])}/{i['row_count']}件PLACED；{a['reason']} | {co.get('rows','未采')}/{co.get('bad_rows','—')} | {cat.get('interarm',{}).get('points',0) if co else '未采'} | {mm(d('finger_part'))} | {mm(d('finger_conveyor'))} | {mm(d('finger_bin'))} | {mm(d('body_bin'))} | {mm(e.get('max_xy_error_m'))} | {e.get('controller_abort_count','—')} |")
add();add('接触行是接触点，不是独立碰撞次数。手指-工件的毫米级压入属于原模型夹持接触，不能冒称两臂相撞。接触CSV时间戳为毫秒舍入，分段按不超过40ms间隔的观察时刻连接；实际可见1ms相邻戳，不用戳数乘名义50Hz周期计算持续时间。观察段可能包含未观测间隙，完整pair列表含最大深度、坐标、时刻与持续段；不能凭采样证明任意未来轨迹无碰撞。缺类别表示本轮未记录到，不是所有物理条件都已验证。')
add();add('初始A/B接触对照无新类别/无两臂接触；深度并非相等：指-带0.061→0.110mm、指-箱0.192→0.244mm，臂/底座-箱0.119→0.025mm。B保留原判据和微小接触，未通过扩大容差掩盖差异。原A精密组也有一次standby控制器goal_time超时中止，随后仍8件分对；这是原等待组已经出现的现象。原生中止事件数与执行器错误日志行数分开保存，避免一次故障计两次。')
add();add('v3精密轮另记录到原A/B单轮没有出现的gripper_base–part_2瞬态接触：18个接触点，最大0.616mm，原生时间戳52.041–52.055s，观测跨度14ms。它发生于arm2腕3固定关节合并的夹爪底座碰撞体，属于机器人–工件，不是两臂互撞。v3指–件最大14.298mm、指–带0.047mm、指–箱0.295mm；本轮没有body_bin。没有因此放宽任何判据，亦不声称物理/接触完全等价或所有优化轨迹均安全。少量对照不能确定瞬态接触由某个等待改动单独造成；后续训练前需匹配时序并复核。')
add();add('C/D精密诊断均在首次永久FAILED后停止（低干扰组已各完整尝试8件并失败），只用于诊断，未冒充完整8件接触验收。失败后残留工件会影响后件，所以整轮后续失败不能当独立伯努利样本。2ms未得到等价或可训练的认证。')
add();add('位置低干扰与精密组的观测时点/分辨率不同，不跨组拼均值。初始低干扰A/B最大XY偏差8.6/19.2mm，精密A/B36.4/32.3mm；少量样本没有一致的单向结论，不能宣称优化后位置完全相同。分拣成功率按原箱体判定，不改标准。')
add();add('v3单实例两轮虽然16件都放对，第一轮有一次回待命goal_time超时；不能称全程控制器无错。原等待A精密组也有同类中止，这说明该错误类别并非只在优化组出现，但少量试验无法证明两者故障概率相同。对长期采样仍须保留控制器错误率和恢复成本，不能把箱体正确率当作机械臂全状态可靠性。')
add();add('## 多实例、历史对比与推荐运行')
add();add('每实例独立ROS_DOMAIN_ID=81/82/83、GZ_PARTITION、HOME、ROS_LOG_DIR和CSV；相同世界名限定在各分区，控制器/相机/clock来源与隔离标记持续检查。共享同一锁定依赖及编译产物。旧083a阶段已实际做暂停、重置、停止一个实例而另一个继续；本轮复用机制，未将world reset误作完整分拣状态机重置。每轮启动前拒绝任何已有Gazebo，检测外来Gazebo即停当前实验，退出回收完整session并核查残留。')
if (R/'three-instance-gate.json').exists():
 gate=json.loads((R/'three-instance-gate.json').read_text())
 add();add('三实例扩容闸实际结果：'+json.dumps(gate,ensure_ascii=False)+'。只有双实例提高同版本整体吞吐且CPU与控制器条件满足时才扩容；未通过时明确记为未测试，不估算三实例性能。')
add();add('本轮v3三实例实测完成24条结果，其中23件PLACED、instance-3的part_3黄色件为OUTSIDE（原记录器观测XY=(0.837,0.220)m，原目标(0.870,0.230)m）。只有两个独立8件批次全对；413.2秒计2批=17.43批/小时，不能按3批计算。稳态RTF约0.469–0.477，CPU均值88.3%、P95约90.2%，无Swap/OOM或持续CPU保护中止，也无本轮控制器中止/视觉颜色串线。按原判据保留失败，不扩大箱界或调整目标修补结果。故不推荐本机三实例作为可靠吞吐配置，也不开展更大规模。')
add();add('旧083a历史：GUI244.04秒=14.75批/小时；双实例380.0秒成功2批=18.95/小时；三实例CPU保护中止，无完整成功。旧234.3秒等受残留进程干扰疑点的单实例数据仍排除。以上跨提交/配置，只作历史背景，不混入8da矩阵因果比较。')
add();add(f"推荐最大已测吞吐运行：`~/yuri_gazebo_performance/fast_sim_8da998b/run_fast.sh {best_n}`。每批独立启动，保留CPU/Swap/OOM保护。3实例若保护中止/未完成，不估算其理论批次率；不超过3个。脚本默认单实例，参数可选1/2/3，实际推荐数量以本报告验证结果为准。")
add();add('正常停止整轮：在日志打印的CASE目录创建STOP；单独停止：`python3 tools/instance_control.py CASE 1 stop`。只作用于该轮会话。回收/重置及故障保护日志保留，不使用全局pkill。大量采样仍需以包括启动、退出及必要复位的真实调度墙钟计算成功率和吞吐，不能用本次短轮速率保证长期产量。')
add();add('## 资源、传感器与同版本复测')
add();add('| v3低干扰轮次 | 有效批次 | 整组s | RTF各实例(P10–P90) | RGB原生/仿真Hz | RAM最低可用GiB | 控制中止/回待命失败 |')
add('|---|---:|---:|---|---|---:|---|')
for a in low:
 if not (a['label'].startswith('B3') or '-v3-' in a['label']):continue
 rates=', '.join(fmt(i['sensors']['gz_rgb'].get('sim_hz'),2) for i in a['instances'])
 rt=', '.join(f"{fmt(i['actual_rtf'],3)}({fmt(i['rtf_5s_p10'],3)}–{fmt(i['rtf_5s_p90'],3)})" for i in a['instances'])
 err=', '.join(f"{e.get('controller_abort_count',0)}/{e.get('return_standby_failures',0)}" for e in a['extra'])
 add(f"| {a['label']} | {a['successful_batches']}/{a['n']} | {fmt(a['experiment_wall_s'],1)} | {rt} | {rates} | {fmt(a['min_mem_available_gib'],2)} | {err} |")
add();add('native相机预期10仿真Hz；ROS图像回调因软件渲染/桥接/消息调度可低于native，实际频率、消息仿真年龄和延迟样本数在每轮analysis中；10秒窗口及部分RGB延迟样本较少，不能当作所有节点的完整延迟分布，不以单个观察端丢帧断言视觉节点丢帧。所有统计只支持本轮覆盖范围，未宣称无未来碰撞或长期零故障。逐进程资源见process-summary.json，原始每核、RSS及Swap在system.jsonl/processes.csv。')
add();add('## 数据兼容性与恢复')
add();add('新数据可作为重新校准timing及失败/恢复成本的原始材料，但必须按1/2ms、原/事件等待、v1/v2/v3、实例数和探针模式分开。旧feed仅生成至停推进、ZONE_CLEAR仅竖直抬高、COMPLETED包含回待命尝试等边界不能自动当成模型READY/完全安全离区。12秒回收也不代表堆叠任务。详细可用项与仍需测量项见DATA_COMPATIBILITY.md。')
add();add('旧Q表对状态/动作/库存约束仍有参考价值，不能直接声称对新timing仍最优。若后续采用新策略，应先冻结事件边界、重新校准匹配配置，再训练/验证新版本表；本轮没有训练、没有实际Q控制、没有改JSON标记。规则成功和Q接口存在都不是deployment_ready证据。')
add();add('恢复8da规则模式原始时序/物理：在实验环境使用 `fast_sim_waits:=false world_file:=sorting_cell.sdf`（原1ms、目标1x、原20/15/1.5/0.4秒路径）；三个Q路径继续显式为空。GUI/RViz可按需要打开，CSV仍唯一。若需严格源码原样另建干净8da工作树，不在此丢弃优化文件；原验收083a启动脚本未改，仍可独立使用。')
add();add('## 问题归属')
add();add('- 原始提交已有现象：原等待A精密轮出现回待命goal_time超时；现有relay键集合测试与提交内配置不一致。样本不足以认定其故障概率或具体根因。')
add('- 本地资料/环境：外部Q表/dual_arm_rl及锚点证据JSON未提供；VMware传感器渲染需已验证的X11 llvmpipe路径。缺资料不等于算法损坏，本轮不尝试修改宿主机或锁版本。')
add('- 本地优化或实验引入：v2就绪发现检查漏报导致未送料超时，v3有界重查修复并重新实测；2ms候选在原/事件等待下均失败；三实例产生OUTSIDE。保留全部证据，未通过改锚点、增益、阈值或策略字段掩盖。')
add();add('## 交付索引')
add();add('REPORT.md、CHANGELOG.md、EXPERIMENT_MATRIX.csv、DATA_COMPATIBILITY.md；PYTHON_CHANGES.diff、SDF_CHANGES.diff、RUN_COMMANDS.txt；audit/为版本/测试/原工程完整性和原接触证据汇总；runs/每轮metadata、环境、source/config哈希、原始stats、每核/进程CPU与内存/Swap、ROS/控制器/OGRE日志、逐件CSV、全部阶段事件、等待记录、接触CSV/gzip/心跳及位姿/误差。未删除旧产物，未提交或推送。')
(R/'REPORT.md').write_text('\n'.join(S)+'\n')
(R/'CONCLUSIONS.json').write_text(json.dumps({'base_commit':'8da998b963eed3c70fda076329f04a75c8627ef3','initial_matched_pair_python_wall_saved_s':A['experiment_wall_s']-B['experiment_wall_s'],'initial_matched_pair_wall_reduction_percent':gain*100,'fastest_observed_single_case':fast['label'],'fastest_observed_single_s':fast['experiment_wall_s'],'fastest_observed_single_rtf':fast['instances'][0]['actual_rtf'],'recommended_instances':best_n,'recommended_physics_ms':1,'pooled_successful_batches_per_hour':best_rate,'including_cleanup_batches_per_hour':life_rate,'successful_batches_in_recommended_group':sum(a['successful_batches'] for a in best_g),'rounds_in_recommended_group':len(best_g),'step2_qualified':False,'no_q_training':True},indent=2))
print(R/'REPORT.md')
