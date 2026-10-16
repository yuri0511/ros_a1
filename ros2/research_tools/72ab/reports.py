import json,statistics,csv,hashlib,datetime
from pathlib import Path
R=Path('/home/yu/yuri_gazebo_performance/integration_72ab6f2');T=json.loads((R/'training/training-manifest.json').read_text());D=json.loads((R/'training/model-evaluation.json').read_text());C=json.loads((R/'training/config/timing.prefeed-false.json').read_text());cases=[]
for p in sorted((R/'runs').iterdir()):
 if not (p/'analysis.json').exists():continue
 a=json.loads((p/'analysis.json').read_text());a['meta']=json.loads((p/'metadata.json').read_text());a['extra']=json.loads((p/'extra-analysis.json').read_text());a['policy_audit']=json.loads((p/'policy-analysis.json').read_text())[0];cases.append(a)
def write(name,s): (R/name).write_text(s+'\n')
def n(v,k=2):return '—' if v is None else f'{v:.{k}f}'
common='完整基线72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd；本地分支perf/integration-72ab6f2。所有性能数字来自该提交时才标为本轮实测。'
write('INTEGRATION_REPORT.md',f'''# 72ab6f2 本地整合报告

{common}

远端fetch与ls-remote确认目标分支仍为72ab6f2，远端默认HEAD/main为125c8d2，未自动换成其他版本。新工作树`{R}/worktree/ros2`。原`/home/yu/ros_a1`保持083a515；原perf/fast-simulation-8da998b分支及未提交修改原样保留，没有提交/重置/推送。旧源文件tar、二进制差异及2,560项文件SHA256清单在audit/；收尾再次核验。

迁移采用8da原始/8da优化/72ab新文件的逐文件三方合并，重叠处保留新版相对Q加载器。迁移v3有界就绪、样本证据等待、MotionClient事件、GUI透传和唯一CSV。没有旧文件整文件覆盖新版本。新增policy_params仅为实验参数覆盖；原feed_policy默认q仍保留，所有rule实验/本地启动器明确传feed_policy:=rule，日志要求“退回谁闲推谁”。原YAML、q_feed.py、codex_rl/、measure/、pixi.lock均与72ab一致，见audit/protected-upstream.json。

物理固定1ms、目标RTF4；保留500Hz控制器、增益、几何、规划参数、相机/视觉、判据和互锁。无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2。独立HOME/ROS_DOMAIN_ID/GZ_PARTITION/ROS_LOG_DIR/CSV与原低干扰探针/停止机制保留。启动器支持1/2实例，但本轮策略对照均为单实例；72ab双实例吞吐未额外测量，不能直接引用8da的24.29批/小时作为新版本实测。没有创建2ms正式实验。共享原锁定Pixi环境、独立build/install；四包构建通过。

回归全量157通过、4失败：3项依赖缺失G0_anchors_v3.json（含相机FOV证据），1项既有relay键集合测试与配置不一致。未伪造文件、改阈值或扩大通过范围；明确排除这4项后157通过。旧版缺Q资料的测试在新提交已提供真实包/表后通过。完整日志和XML保留。

离线两张表各50,000回合（默认292状态、通用1,806状态）；默认2222新Q为122.32模型秒，达到同模型DP最优，rule为126.49模型秒。3组真实低干扰配对双方各24件全对，rule平均184.59秒、Q平均187.96秒，Q慢1.83%，未显示稳定物理提速。另有独立功能精密轮；Q精密轮虽8件放对，但发生一次回待命控制器中止、并记录上臂–箱壁接触，不能称全部动作可靠性已通过。三组低干扰配对无控制器中止。所有4轮Q共32次决策命中新表，未知回退/WAIT均0。本地默认继续rule，标记保持false。

训练与DP结果见TRAINING_AUDIT.md、DP_BASELINE_REPORT.md；真实验收与配对见GAZEBO_POLICY_COMPARISON.md。首次启动器因未创建runs目录在启动任何Gazebo前退出，补建输出目录后重试；该本地工具失误日志保留，不算分拣失败。

最终完整性与进程清理见audit/final-integrity.json（生成后有效）。未修改deployment_ready或overlap_validated，也没有安装自定义Skills或新ROS依赖。
''')
rows=[]
for x in T:rows.append(f"| {x['label']} | {x['episodes']} | {x['seed']} | {x['states']} | {x['training_wall_s']:.2f} |")
grid=[]
for name in ('new_q_demo','new_q_general','old_q_legal_filtered_historical'):
 g=[x for x in D['other_batches'] if x['policy']==name];steps=sum(x.get('decision_steps',0) for x in g);u=sum(x.get('unknown_state_fallback_steps',0) for x in g);grid.append(f'| {name} | {len(g)} | {steps} | {u} | {100*u/steps:.3f}% |')
write('TRAINING_AUDIT.md',f'''# 离线重训练审计

{common}

旧配置allow_prefeed_busy_arm=true；ROS q_feed.decide_feed复制配置并强制false。旧训练动作空间比实际多，虽加载校验通过，动作合法性语义并不一致。此次独立配置只修正该布尔值、计时版本及限制说明；原feed/pick/post_pick数值、overlap_validated=false原样保留。不覆盖旧配置或Q表。

| 表 | 离线回合 | seed | 状态数 | 本机训练墙钟s |
|---|---:|---:|---:|---:|
{chr(10).join(rows)}

两张表各自从空表训练50,000回合，不是相互续训。使用上游原训练器、alpha=.2、gamma=1、epsilon线性.5→.05。demo固定[2,2,2,2]供本次Gazebo；general为随机8件多重集供其他需求研究。**合计100,000个模型回合，不是Gazebo运行100,000回合。**

新配置canonical SHA256：`{T[0]['config_sha256']}`。
配置文件字节SHA256：`{T[0]['config_file_sha256']}`。

'''+ '\n'.join(f"- {x['label']}表文件SHA256：`{x['table_sha256']}`。" for x in T)+f'''

qtable.load已针对新配置严格校验两张新表；旧表对新配置被拒绝，不能仅改JSON标签冒充重训练。training-manifest.json记录确切路径/时间/训练参数。新表deployment_ready=false，overlap_validated=false；旧文件及其字段未改。

默认2222新表评估8次决策全部命中，未知回退0，第一件由实际Q最大值选择red，未强制颜色。其他165种总数8的有效库存均用相同修正模型完成：

| 表 | 库存种类 | 决策数 | 未知回退 | 回退率 |
|---|---:|---:|---:|---:|
{chr(10).join(grid)}

因此demo表适合指定演示需求，不能对其他需求宣称全面覆盖；general表覆盖更广仍有少数未知状态，保留合法动作回退。实物运行中的ETA/事件分布不同，实际回退必须另从Q_DECISION日志统计。

数据来源：feed(4.20–4.44)、pick(8.09–10.36)、post_pick(7.66–11.83)为Yuri历史Gazebo成功样本均值，非本机72ab/v3重新测量。request_delay=1s、ETA边界[5,10,20]、idle_weight=.05及重叠许可属于模型假设。推板回位/settle、安全离区、失败恢复、待命失败成本缺失或边界不全。原is_placeholder=false仅继承原均值出处，不代表该新配置已完整校准。逐项见DATA_COMPATIBILITY.md。
''')
modelrows=[]
for name,v in D['policies'].items():modelrows.append(f"| {name} | {v['makespan_model_s']:.3f} | {v['weighted_cost']:.4f} | {v['gap_to_dp_makespan_s']:.6f} | {v['first_color']} |")
write('DP_BASELINE_REPORT.md',f'''# 同模型 DP 最优基线

共同配置hash `{D['config_sha256']}`，库存[2,2,2,2]，prefeed=false。模型总时间与加权代价（time+.05×demand-idle）分别求最优。

| 策略 | 模型耗时s | 加权代价 | 距最短时间差s | 第一色 |
|---|---:|---:|---:|---|
{chr(10).join(modelrows)}

新Q与模型最短时间差为浮点精度内0；rule为126.49秒，新Q为122.32秒，模型内缩短{(1-122.32/126.49)*100:.3f}%。旧Q按新合法动作过滤后在该批次同样达到122.32秒，故不能声称重训练在该模型批次战胜旧Q；此次重训首先修正语义/配置对应关系。

ROS rule实际使用choose_idle_feed：空闲臂交替、同臂颜色按已送料数平衡；上游离线rule_action按ETA及颜色序号。两者都单列，不能把名称相同当作完全同策略。默认库存在本模型上两者耗时恰好相同。

DP复用未经改动的Model.legal_actions和Model.step，对每个状态的所有合法FEED及WAIT_NEXT_EVENT递归。记忆化键包含库存、忙闲/活动色、相位开始相对时间、完整事件队列相对时间与顺序、送料槽/取件区所有者、完成和请求计数。不是只枚举颜色排列。默认加权DP探索{D['policies']['dp_reward']['states']}个完整事件状态、{D['policies']['dp_reward']['wait_edges']}条WAIT边；8项小批次/两目标无记忆化穷举交叉检查全部相等。

DP可访问精确内部事件时间，Q只能看到ETA离散观测，因此它是当前简化模型的信息上界；默认批次Q恰好达到它不代表其他状态/批次最优，更不是Gazebo物理全局最优。模型没有失败、回位完整成本或真实安全离区证明。

历史表先按旧配置严格验证，评估时只将其未修改q_values通过choose的合法动作过滤器作用于修正后的同一Model。没有伪造新元数据、没有把旧表当成对新配置可直接加载；这是明确标记的历史迁移参考。全部轨迹、feed顺序、评估/训练版本在training/model-evaluation.json，复现命令`python3 tools/dp_baseline.py`。
''')
write('DATA_COMPATIBILITY.md','''# Q模型与Gazebo逐项对照

| 项目 | 当前模型/旧表 | ROS/Gazebo事实 | 本轮处理/剩余限制 |
|---|---|---|---|
| 忙臂预送 | 旧true | decide_feed强制false | 新配置false且重新训练；不改旧表 |
| 送料完成 | spawn→stop advance平均值 | 还必须推板回位、settle与窗口合法才派发 | 原均值不补造数据；新实测阶段日志保留，尚未校准为完整模型 |
| 1秒请求延迟 | 仿真秒模型假设 | 默认并联事件路径并非固定等1墙钟秒 | 保留原模型以完成受控重训，明确不等价 |
| ZONE_CLEAR | PICK_EXIT允许下一次送料/取件 | 原carry_clear主要竖直抬高，手仍在取件点上方 | 保留互锁；接触只验证覆盖组合，不把它扩展为任意轨迹的安全证明 |
| Arm READY | post_pick到期即成功READY | COMPLETED包含尝试回待命，原代码可能待命失败后仍发终态 | 单列控制器中止/返回失败，不用放对代替全状态就绪 |
| 重试/失败 | 确定时长、无失败状态 | 真实有规划/抓取/控制器重试、OUTSIDE | 各轮保留；模型缺失成本，不伪造概率 |
| 双臂重叠 | allow_arm_overlap=true但overlap_validated=false | 取件区单所有者、单送料槽、每臂单任务；其他搬运可能重叠 | 不改标记；新策略需独立接触验证，不能泛化到全部轨迹 |
| 时间 | 历史成功样本仿真秒 | v3仍含墙钟采样/释放/回调超时；实际RTF随负载变 | 不混仿真/墙钟，不把模型收益套到Gazebo |
| 事件等待 | 历史均值无本机v3标签 | v3缩短首件/验证尾等待，其他阈值保留 | 新表标unrecalibrated；本轮新数据可作后续重新校准原料 |
| 需求/分色 | 默认2222；通用表随机总数8 | 每臂固定颜色路由，实际视觉仍分类 | 默认库存匹配；其他库存单独测覆盖/回退 |
| 状态 | remaining+READY/POST_PICK+ETA档+活动色 | 真实ETA由carry_clear stamp估计；时延/恢复会改变落档 | 实际Q命中/回退与WAIT逐条记录；加载成功不等于策略命中 |
| 工件回收 | 每个件末尾完成 | 原记录器成功后12仿真秒删除 | 保留；数据不能直接校准长期箱内堆叠 |

历史8da性能数据、新72ab规则/Q功能验收、低干扰配对、精密接触与离线模型严格分组。原Q表仍具历史参考价值，prefeed不一致已修正，但新计时仍未完整校准，因此不能直接称deployment_ready。下一次可用本机新阶段数据定义完整feed/真实可接单/安全离区/恢复成本，冻结配置后再次训练与物理验收。本轮不擅改deployment_ready或overlap_validated。
''')
S=['# 真实 Gazebo 策略比较','',common,'','全部1ms、v3事件等待、目标RTF4、同一SDF/模型/增益/规划/相机/互锁/判据，LP线程2，X11 llvmpipe，无GUI。每次新进程/新世界，四色各2件，独立域/分区/HOME/日志/CSV。未固定OS调度；默认Gazebo/规划器随机种子没有额外修改，颜色由策略选择，不强制Q先红。','', '功能验收为独立高精度接触轮；不混入正式低干扰策略耗时。随后3组配对顺序rule→q、q→rule、rule→q，结果失败也保留，不通过重跑筛选成功。','','| 轮次 | 类型 | 策略 | 正确件/8 | 墙钟s | 首送料至末终态仿真s | 稳态RTF | 重试 | 控制中止 | 首色 | Q命中/未知/WAIT |','|---|---|---|---:|---:|---:|---:|---:|---:|---|---|']
flat=[]
for a in cases:
 i=a['instances'][0];e=a['extra'][0];p=a['policy_audit'];co=Path(i['path'])/'contacts-task-summary.json';contact=json.loads(co.read_text()) if co.exists() else None
 correct=sum(x['result']=='PLACED' and x['final_bin']==x['target_bin'] for x in i['results'])
 S.append(f"| {a['label']} | {'精密功能' if a['contact_mode']=='1' else '低干扰配对'} | {a['meta']['feed_policy']} | {correct}/8 | {a['experiment_wall_s']:.1f} | {n(e.get('full_batch_sim_s'))} | {n(i['actual_rtf'],3)} | {i['retries']} | {e['controller_abort_count']} | {p['first_color']} | {p['known_state_decisions']}/{p['unknown_state_fallbacks']}/{p['wait_decisions']} |")
 flat.append({'case':Path(a['case']).name,'policy':a['meta']['feed_policy'],'contact_mode':a['contact_mode'],'successful':i['successful'],'correct_parts':correct,'wall_s':a['experiment_wall_s'],'sim_task_s':e['batch_task_sim_s'],'sim_full_feed_to_terminal_s':e.get('full_batch_sim_s'),'world_last_terminal_sim':e.get('world_last_terminal_sim'),'rtf':i['actual_rtf'],'retries':i['retries'],'controller_aborts':e['controller_abort_count'],'standby_failures':e['return_standby_failures'],'placement_max_xy_error_m':e['max_xy_error_m'],'placement_mean_xy_error_m':e['mean_xy_error_m'],'vision_errors':len(i['vision_mismatches']),'interlock_errors':len(i['interlock_violations']),'ros_foreign':len(i['isolation']['ros_foreign']),'gz_foreign':len(i['isolation']['gz_foreign']),'first_color':p['first_color'],'q_known':p['known_state_decisions'],'q_unknown':p['unknown_state_fallbacks'],'q_wait':p['wait_decisions'],'config_sha256':p['q_loads'][0]['config_sha256'] if p['q_loads'] else None,'table_file_sha256':p['q_loads'][0]['table_file_sha256'] if p['q_loads'] else None,'runtime_hash_match':p.get('matches_new_training_hashes'),'source_hash':a['meta']['source_hashes'],'contact_points':contact['rows'] if contact else None,'interarm_points':contact['categories'].get('interarm',{}).get('points',0) if contact else None})
S+=['','仿真批次时长明确取首次送料消息时钟至最后TaskStatus终态；另保存从首次PICKING起的执行段时长与世界末终态绝对仿真秒。墙钟结束点为最后真实CSV结果，两个端点略有差别，不混用时钟。','','只有唯一part_1..8全部PLACED、目标/实际箱一致且四箱各2件，才计成功批次。控制器、视觉、互锁、未知回退另列。没有把8da的169.4秒或24.29批/小时当作72ab本轮数据。']
for policy in ('rule','q'):
 g=[a for a in cases if a['label'].startswith('pair') and a['meta']['feed_policy']==policy]
 if g:S+=['',f"{policy}低干扰：{len(g)}轮，成功{sum(a['successful_batches'] for a in g)}，平均墙钟{statistics.mean(a['experiment_wall_s'] for a in g):.2f}s，含失败尝试的成功批次率{3600*sum(a['successful_batches'] for a in g)/sum(a['experiment_wall_s'] for a in g):.2f}/小时。"]
pairs=[]
for k in (1,2,3):
 g={a['meta']['feed_policy']:a for a in cases if a['label'].startswith(f'pair{k}-')}
 if len(g)==2:
  delta=g['q']['experiment_wall_s']-g['rule']['experiment_wall_s'];pairs.append(delta);S+=['',f"配对{k}：Q−rule墙钟{delta:+.2f}s；两侧批次成功{g['rule']['successful_batches']}/{g['q']['successful_batches']}。"]
if pairs:S+=['',f'完整配对数{len(pairs)}，Q−rule平均墙钟差{statistics.mean(pairs):+.2f}s。样本很少且存在重试/资源/RTF波动；这只支持本轮观察，不能套用模型3.30%或宣称长期稳定收益。']
pr=[a for a in cases if a['label'].startswith('pair') and a['meta']['feed_policy']=='rule'];pq=[a for a in cases if a['label'].startswith('pair') and a['meta']['feed_policy']=='q']
if len(pr)==len(pq)==3:
 rw=statistics.mean(a['experiment_wall_s'] for a in pr);qw=statistics.mean(a['experiment_wall_s'] for a in pq)
 rs=statistics.mean(a['extra'][0].get('full_batch_sim_s') or 0 for a in pr);qs=statistics.mean(a['extra'][0].get('full_batch_sim_s') or 0 for a in pq)
 S+=['',f'三组主要观察：rule平均墙钟{rw:.2f}s，Q平均{qw:.2f}s，Q相对变化{(qw/rw-1)*100:+.2f}%；首次送料至终态仿真时长均值rule={rs:.3f}s、Q={qs:.3f}s。',f'自动重试总数rule={sum(a["instances"][0]["retries"] for a in pr)}、Q={sum(a["instances"][0]["retries"] for a in pq)}；该差别只是小样本观测，不能证明策略导致固定的失败概率变化。','结论：本轮不足以证明新Q有稳定Gazebo性能收益；保持rule为本地默认。新Q完成约束一致的离线重训与有限物理功能验证，仍需匹配v3的完整时序/失败模型和更多受控重复，不能改为deployment_ready。']

S+=['','最终逐行审计：Q精密功能轮发生一次joint_trajectory_controller回待命goal_time超时中止，执行器standby status=6；仍8件放对。此前阶段性口头说明曾漏报，最终表/日志已纠正。三组低干扰配对两侧均无控制器中止。历史8da原等待组也出现同类待命问题，不能单凭本轮确定根因或宣称Q全动作可靠性已通过。','','## 独立接触对照','','| 功能精密轮 | 点数/坏行 | 两臂点数 | 指-件最大mm | 指-带最大mm | 指-箱最大mm | 臂-箱最大mm/最长观测段s |','|---|---:|---:|---:|---:|---:|---|']
for a in cases:
 if a['contact_mode']!='1':continue
 cp=Path(a['instances'][0]['path'])/'contacts-task-summary.json'
 if not cp.exists():continue
 co=json.loads(cp.read_text());cat=co['categories']
 depth=lambda k:n(cat[k]['max_depth_m']*1000,3) if k in cat else '未记录'
 S.append(f"| {a['label']} | {co['rows']}/{co['bad_rows']} | {cat.get('interarm',{}).get('points',0)} | {depth('finger_part')} | {depth('finger_conveyor')} | {depth('finger_bin')} | {depth('body_bin')}/{n(cat.get('body_bin',{}).get('max_segment_span_s'),3)} |")
S+=['','本轮Q上臂–黄箱壁最大0.197mm、最长观察段4.012仿真秒（145.363–149.375s），另有上臂–红箱壁6微米/1ms；rule轮没有body_bin，记录到夹爪底座–件0.152mm/2ms。历史8da有类似上臂–箱壁接触，但不同提交/轮次不能作数值等价证明。缺少长期或任意重叠组合证据，Q功能成功不等于碰撞行为完全相同或已通过部署认证。没有改任何碰撞参数/判据。']
if (R/'POLICY_EXPERIMENTS_DONE').exists():S+=['','全部计划实验完成；最终结论必须结合逐轮接触、回退与错误情况。']
else:S+=['','状态：实际验收/配对仍在进行，未完成项没有性能结论。']
S+=['','接触点不是独立碰撞次数；缺类别只表示本轮未记录。每轮contacts-task-summary.json含全部类别、最大深度、真实戳和观察分段；不据此改任何容差或认证所有未来轨迹。低干扰位置取原记录器完成时XY（1mm舍入），精密组缓存原生位姿至原回收时刻。','', '运行入口：`./run_policy.sh rule` 或 `./run_policy.sh q`；默认rule。新Q使用training/q-params.yaml独立覆盖，日志Q_AUDIT_LOAD记录路径/配置与文件哈希，Q_DECISION记录实际状态、合法动作、命中/回退、WAIT及选择；单纯“已加载”不算命中。']
write('GAZEBO_POLICY_COMPARISON.md','\n'.join(S))
if flat:
 with (R/'GAZEBO_RESULTS.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
print('Reports updated, cases',len(cases))
