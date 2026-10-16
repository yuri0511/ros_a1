from pathlib import Path
import json,collections,csv,hashlib,difflib
R=Path('/home/yu/yuri_gazebo_performance/control_stability');W=R/'worktree/ros2';A=json.loads((R/'audit/native-response-analysis.json').read_text());F=json.loads((R/'audit/loaded-20261017T031609-gain1-rule-eight.json').read_text());T=json.loads((R/'audit/loaded-20261017T031110-gain1-loaded-two.json').read_text());K=json.loads((R/'audit/gain1-eight-contacts.json').read_text());proof=next(x for x in A['joint_metrics'] if 'commandprobe' in x['case'] and x['phase']=='empty_carry' and x['joint']=='wrist_3_joint');both=all(x['validated'] for x in F['ready_audits']);assert both and len(F['rows'])==8 and len(F['ready_audits'])==8 and F['slot_order_ok'] and not F['faults'] and not F['controller_failed_results']
R.joinpath('GAIN_STABILITY_ROOT_CAUSE.md').write_text(f'''# gz_ros2_control位置控制稳定性：本机根因验证

2026-10-17，独立分支experiment/control-stability-gains，基于a21ee9b7b0d9631b86dbcca5e15f1b0e556de54e。原ready_bugfix、原工程、历史性能/RL/Q表、正式分支与Draft PR均保留。

**结论：在本机所测锚点工况下，gain=2是持续振荡的可干预控制因素，已由单因素变化、原生命令/物理响应和源码共同证实；gain=1配合已有物理READY协议完成本轮8件Rule。** 不是仅根据理论推断，也不是宣称任何轨迹/任何模型均已验证。

## 证据链

实际安装gz_ros2_control1.2.20，实际进程maps加载原Pixi环境libgz_ros2_control-system.so和libgz_hardware_plugins.so；后者的gz_ros2_control/GazeboSimSystem接受JTC position命令。已保存库SHA、插件XML、包版本、对应版本官方源码。未替换/重编译官方插件。

GetParameters运行时确认A/B/C的/gz_ros_control.position_proportional_gain分别2.0/1.0/.5，controller_manager.update_rate均500、use_sim_time=true。A的4个硬件系统启动日志均gain2；B/C相同检查。native记录实际积分间隔1ms；原URDF、惯量、阻尼、摩擦、规划/缩放、锚点、轨迹目标、READY阈值不变。

[官方Jazzy文档](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html#notes-on-the-command-interface)说明position通道以误差生成速度目标；[1.2.20实际对应源码](https://raw.githubusercontent.com/ros-controls/gz_ros2_control/1.2.20/gz_ros2_control/src/gz_system.cpp)与之相符：

```
v_cmd = gain * (q_target - q_sampled) * 500
```

控制器每约2ms读位置、物理每1ms积分，位置接口在PreUpdate重复使用最近控制读数。理想采样保持模型e[k+1]≈(1-gain)e[k]；gain2的离散极点约-1，可产生不衰减的交替响应（实际DART约束、阻尼/浮点会修饰幅度）。这段模型用于解释观测，不替代测量。

## 直接观测，不仅是ROS发布现象

A的无工件arm1返回后，原生速度峰值1.699040rad/s，位置偏差约.00169904rad，主频约249.75Hz（1秒FFT约1Hz分辨率）。Action已SUCCESS，但READY正确拒绝，停止该轮，未继续arm2。

第一版记录器在PostUpdate读取JointVelocityCmd，读到的全0不能代表控制器下发值；该字段原样保留，分析将其标为不可用，未用零值声称无命令振荡。随后额外进行**一次命令采样诊断**：只读系统以priority100在默认命令写入器之后、Physics Update之前采命令，再于PostUpdate采物理状态；没有修改任何组件/力/速度/实体，也没有修改其他系统的相对执行顺序。

该额外gain2诊断在空载carry保持段出现：

- JTC位置目标固定为wrist3=-4.41639rad，不是JTC位置目标来回摆动。
- native位置误差峰值{proof['max_position_error_rad']:.9f}rad，实际速度峰值{proof['velocity_peak_rad_s']:.9f}rad/s。
- PreUpdate实际速度目标峰值{proof['velocity_cmd_peak_rad_s']:.9f}rad/s，与物理速度相关系数{proof['velocity_command_correlation']:.6f}。
- 用实际JTC末点与最近2ms采样保持位置重建速度命令，窗口内RMSE={proof['best_sample_hold_fit']['command_rmse']:.3g}rad/s；这不是用接收墙钟捏造仿真时间。
- 主频{proof['dominant_velocity_hz']:.2f}Hz，native速度相对native位置相位约{proof['velocity_vs_position_phase_deg']:.2f}°。1ms离散积分在250Hz下的该相位关系合理；不能直接套连续时间90°。

因此：JTC位置参考静止→底层比例位置接口产生交替速度目标→Gazebo真实位置/速度随之振荡→ROS发布该状态。重复header会造成混叠/误停，但不是全部振荡的来源。降低gain到1/.5后同一组目标的原生振荡消失，空载两臂均通过。

历史2.886rad/s与.002886rad误差的1000倍比例支持同一机制，但本轮没有复制其完整同一轨迹并复现完全相同幅值；不能把2.656或1.699写成2.886精确复现。初始条件与采样相位会改变边界振荡的幅度和首先失败的阶段。

## Action SUCCESS与物理READY

JTC使用position命令接口；对应4.43.0源码的速度误差计算受命令接口条件限制，SUCCESS不构成物理静止证明。运行时open_loop_control=false，原YAML的open_loop键查询不可用；原注释不能当实际生效配置，本轮未把它改名或改变行为。

继承的协议仍要求成功Action/error0、无活跃目标、新鲜唯一关节数据、.01rad位置、发布及差分速度.3rad/s、至少50递增戳/.25sim秒。没有忽略速度、扩大容差、缩短窗口或直接设置待机标志。gain1完整8件里每次真实READY均有原生数据独立复核。

所有原始目录、启动失败、故障轮、额外探针均列在CASE_SUMMARY.csv和audit/native-response-analysis.json；未重跑筛选成功轮。额外命令诊断因采样相位修正而进行，明确不与首轮当同一测量混算。
''')
R.joinpath('CONTROLLER_PARAMETER_AUDIT.md').write_text('''# 控制器参数、插件及单因素审计

| 项目 | 实测/审计结果 |
|---|---|
| 基线 | a21ee9b7b0d9631b86dbcca5e15f1b0e556de54e |
| 独立分支 | experiment/control-stability-gains |
| Gazebo / 步长 | Harmonic8.10.0；原fast_sim_1ms.sdf的max_step_size=.001，原生相邻积分1ms |
| 插件 | libgz_ros2_control-system.so创建controller_manager；gz_hardware_plugins中的GazeboSimSystem实现position接口 |
| gz_ros2_control | 1.2.20，原Pixi二进制，未更换依赖 |
| controller_manager / JSB / JTC | 4.49.0 / 4.43.0 / 4.43.0 |
| 控制频率 | 参数500Hz，最近位置读数按控制周期保持；native物理1000Hz |
| runtime gain | A=2.0、B=1.0、C=.5，运行期参数服务及硬件启动日志核对 |
| JTC命令/状态 | position / [position,velocity]，硬件接口清单另存 |
| open_loop_control | B/C/额外gain2探针运行期查询false；interpolate_from_desired_state=false |
| 原open_loop键 | 参数服务查询不可用，未改；不能依据原YAML注释宣称开环已启用 |
| stopped_velocity_tolerance / goal_time | .2 / 2.0（JTC原值）；独立READY仍.3rad/s/.01rad/.25秒/50戳 |

基线ur5e_cell_controllers.yaml原样保留。新增ur5e_cell_controllers_stability_gain2.yaml与其字节一致；gain1/gain0p5仅改/**.ros__parameters.position_proportional_gain。结构化单因素证明见audit/single-factor-proof.json，字节差异见GAIN_PARAMETER_DIFFS.diff，各配置SHA见audit/gain-configurations.json。

无工件：原cell_bringup+MoveIt+原scene_publisher；不运行派发、不请求生成工件；原Gazebo RGB/逻辑相机和桥接仍工作。诊断驱动实例化原PickPlaceExecutorNode/MotionClient，按臂串行使用原initial standby位姿、原carry_anchor1关节目标和原standby_joints。没有捏造part_id或伪发ARM_READY。每步Action结束后观察2sim秒，墙钟上限8秒；比较最后1sim秒，READY实际窗口不改。carry锚点仅使用原补偿门槛.55rad/.3rad/s，不把carry判据冒充.01rad的standby READY。

有工件：原sorting_demo、feed_policy:=rule、1ms/v3、X11 llvmpipe、LP_NUM_THREADS=2、无GUI/RViz；原安全状态机、相机分类、夹爪、规划、几何及碰撞判据均不修改。每轮独立HOME、ROS_DOMAIN_ID、GZ_PARTITION、CSV、ROS_LOG_DIR；复用同一Pixi环境。

只读trace世界从原1ms世界复制，仅增加观测插件；配置/源码中没有物理或传感器删改。v1 PostUpdate与v2 Pre/Post的原始文件及两个.so均保留。v2优先级100只确定观测在默认写命令之后；原控制/物理系统参数与相对顺序不改。对应binary SHA及maps见audit/loaded-binary-hashes.json与各summary.loaded_libraries。

启动期有一次ListControllers请求未返回，尚未发运动，保留为030208启动审计失败；此后增加控制器就绪屏障及有界只读RPC重试。A最初批量查询含未知open_loop导致JTC参数响应空列表，未当成参数不存在的证明；B/C与额外探针改为逐项查询，完整保存结果。以上没有改变控制变量。
''')
R.joinpath('JOINT_RESPONSE_ANALYSIS.md').write_text(f'''# 关节响应与负载核对

GAIN_COMPARISON.csv保存102个关节/阶段测量行，统一取每次Action结束后2sim秒窗口的最后1sim秒。没有够数据的启动失败/未到达阶段不补零，见CASE_SUMMARY.csv。

| gain | arm1返回腕3速度峰值rad/s | arm2返回 | 结论 |
|---|---:|---|---|
| 2.0 | 1.699040，约250Hz | arm1故障后未继续 | 不通过 |
| 1.0 | 5.96e-14 | 1.68e-14rad/s，两臂通过 | 候选，后完成带件及8件 |
| .5 | 5.96e-14 | 1.68e-14rad/s，两臂通过 | 仅空载通过，未验证抓放/8件 |

这些接近零的值是本仿真position接口的数值结果，不是传感器测量精度或现实机器人指标。频谱仅在信号高于1e-8分析噪声底时报告；该分析噪声底不参与READY验收，物理门槛未改变。

额外gain2命令探针在carry段速度/目标均约2.656321rad/s，位置峰差.002656321rad，原生命令与速度同相（相关≈1），相对于位置约44.96°，采样保持模型命令重建RMSE0。初始JTC逆解末点可能与参数关节目标有极小差异，命令模型使用真实下发轨迹末点，位置验收仍使用原standby目标。native实测与ROS数据共同排除单纯发布者错误；不将PostUpdate被清零的命令字段作为控制输入证据。

![Native joint response](JOINT_RESPONSE.png)

图前三格比较arm1返回，第四格为额外gain2的carry命令诊断，阶段不同已标明，不能把不同阶段的幅值当同一条件重复样本。PDF版本JOINT_RESPONSE.pdf。

## 负载、接触段与实际跟踪

gain1先运行两件Rule（blue/arm1、red/arm2），两件PLACED、2次真实READY、38次控制器结果成功、0重试/故障。两臂空载锚点与返回原生位置误差约1e-13量级，未见新增空载下垂。

保持目标数据中arm1肩关节在一次放置接触段偏差.136977675rad；历史gain2相同参考构型也有.136615267rad，差约.000362407rad。其他相同筛选保持段未见该偏差。该现象不能被漏报，也不能仅凭同阶段相关性断言其全部来自箱壁或全部来自重力。它不是降低gain后才出现的全新自由承载下垂模式；未改变原接触/补偿门槛来掩盖。

两件记录的finger_bin最大穿深.000136m，历史gain2蓝件诊断为.000180m；仅作有限对照，不能宣称各颜色完整统计等价。两件和8件没有interarm/body_bin类别，抓取接触原样保留。完整8件finger_part最大{K['categories']['finger_part']['max_depth_m']:.6f}m、finger_bin最大{K['categories']['finger_bin']['max_depth_m']:.6f}m。接触点行不是独立碰撞次数，不能把无严重类别写成无接触/安全认证。

选择gain1：在所测空载目标上与.5均稳定，且gain1只需将原2减半、符合文档非振荡范围；随后只对gain1进行了真实带件/完整8件。没有声称已证明gain1全局优于.5或.5带载同样可用；gain10未测试。
''')
R.joinpath('GAZEBO_VALIDATION.md').write_text(f'''# 有限物理回归：gain1完整8件Rule通过

目录runs/20261017T031609-gain1-rule-eight，实际运行源码ed76c35（完整SHA在metadata与FINAL_STATE）。无GUI/RViz、1ms、500Hz、v3、X11 llvmpipe、LP_NUM_THREADS=2、feed_policy:=rule，独立实例。原几何/规划/控制阈值/相机/夹爪/安全状态机不变；仅控制器gain从2改1。

## 结果

- 原结果记录器：8/8 PLACED，blue/red/yellow/green各2，四个对应箱各2，无放错/失败。
- 抓取失败/重试日志0；控制器结构化Action结果148条，全部status4/error_code0，未发现Action失败；MoveIt规划错误另有原launch日志可审。
- PART_PLACED 8次、ZONE_CLEAR 8次、ARM_READY 8次，ARM_FAULT/FAILED/PART_REJECTED 0次；执行槽释放8次，全部trigger=ARM_READY。
- 同一臂未在前一件READY前开始下一件；两臂各4件，全局取件区释放与执行槽释放分别记录，未混淆；已有单元测试继续覆盖失败后锁存及迟到COMPLETED无权释放。
- 8次READY均有成功return Action、无活跃Goal、至少50个递增戳/.25sim秒、原位置/速度/新鲜度门槛。各次ROS原始数据与native1ms数据独立复核均通过；全轮READY区间原生最大速度{max(x['native_max_speed'] for x in F['ready_audits']):.9f}rad/s，小于.3，最大目标位置误差{max(x['native_max_position_error'] for x in F['ready_audits']):.4g}rad，小于.01。没有放宽或跳过验证。
- 第一次PICKING到最后ARM_READY：{F['task_wall_seconds']:.3f}墙钟秒、{F['task_sim_seconds']:.3f}仿真秒；完整启动/清理生命周期268.975墙钟秒。supervisor.measure_end是旧CSV时间口径，不用它冒充最终物理READY耗时。高频记录有额外负载，不据本轮声称性能收益。

物料顺序原样：blue,red,yellow,green,blue,red,yellow,green；非强制首色的Q实验，完全未运行Q。

## 接触、传感器和清理

contact CSV共{K['rows']}行，finger_part {K['categories']['finger_part']['points']}、finger_conveyor {K['categories']['finger_conveyor']['points']}、finger_bin {K['categories']['finger_bin']['points']}；没有interarm/body_bin记录，也没有对应guard触发。finger_bin最大.000094m，finger_part最大.014026m；仍保留既有接触段姿态偏差与抓取穿深限制，不宣称无碰撞风险或几何安全认证。

Gazebo RGB1569帧、ROS RGB422帧、逻辑相机1571条由probe记录，8条来料；视觉和物理未关闭。高频证据含78414条JointState、154644条ControllerState、296条Action状态、64条任务/结果事件。node日志含全部控制器结果，namespace/分区环境见launch-environment.json。

正常达到8次READY后由监督器按原停止路径SIGINT该轮launch并回收辅助记录器，summary.residual_session_cleanup=[]；最终无仿真残留。部分Python节点会把SIGINT记作KeyboardInterrupt/exit -2，不将其写成运行期控制器崩溃，也不谎称全部节点自然exit0。

## 所有轮次与范围

1. 030208 gain2启动参数审计请求超时：未发运动，保留失败记录。
2. 030406 gain2空载：初始/搬运通过，arm1返回持续超速，FAULT即停；未继续arm2。
3. 030503 gain1空载：两臂三阶段均通过。
4. 030615 gain.5空载：两臂三阶段均通过。
5. 030953 gain2额外命令探针：修正观测相位，carry持续振荡即停，不作为成功重跑。
6. 031110 gain1两件：2/2正确、两臂READY独立复核通过，无重试。
7. 031609 gain1八件：本报告，唯一一次完整8件；通过后没有再跑。

本轮验证了候选在当前VM、当前八件需求及一次回归上的行为，不是长期可靠率、16组统计、Q策略收益或真实硬件安全认证。旧失败证据均保留，未将历史失败改为成功。原物理READY故障阻断机制和单元测试保留。

独立逐件复核见audit/loaded-20261017T031609-gain1-rule-eight.json；完整命令、环境、原始stats/JointState/ControllerState/native CSV/contacts/results均在上述run目录。
''')
R.joinpath('FINAL_RECOMMENDATION_CN.md').write_text('''# 最终建议：gain1 + 物理READY协议，交Yuri审阅

1. **运行时gain确为2.0吗？** 是。原gain2配置、GetParameters与4个硬件系统启动日志一致，500Hz和1ms亦已核实。
2. **gain2是振荡原因吗？** 在本机测试锚点上，单因素变化、实际速度命令/原生物理250Hz响应及对应源码共同证实它是持续振荡的控制因素。没有把所有历史失败都归因于gain2，也未精确重现旧2.886幅值。
3. **gain1和.5的变化？** 两者空载均消除所测持续腕振荡，两臂待机/搬运/返回通过。仅gain1随后完成带工件和完整8件；.5未验证抓放，不当正式推荐。
4. **持续速度异常解决了吗？** gain1此次两个带件与完整8件的10次READY均满足原阈值，未复现返回持续超速；gain2故障轮仍原样保留，未被忽略速度或改阈值掩盖。
5. **双臂抓放是否受影响？** gain1两件及8件均正确，无抓取重试/控制器失败，未见新增自由承载下垂或严重接触类别。放置接触段约.137rad肩误差在gain2已存在，夹爪接触/穿深仍保留为限制，不能宣称全部接触安全已解决。
6. **8件规则是否通过？** 是，一次完整8/8、四箱各2、8次物理READY/匹配槽释放、0故障/重试，逐次原生数据独立复核通过。没有第二次筛选重跑。
7. **能否交Yuri采用？** 可以提交本机有限验证通过的仿真候选供审阅：保留READY代码协议，并显式选gain1配置。不能只复制gain1就删除ARM_FAULT/物理窗口，也不能把它当长期可靠性或硬件生产认证。

## 推荐应用边界

继续使用1ms、500Hz、v3、headless、X11/llvmpipe、LP_NUM_THREADS=2、feed_policy:=rule。实验配置通过已有controllers_file参数启用，原ur5e_cell_controllers.yaml仍gain2未覆盖；不要同时改变惯量/阻尼/规划/速度缩放或open_loop_control来混入第二因素。

Yuri需审查并同步采用此前MotionClient严格结果检查、executor物理门槛、PART_PLACED/ARM_READY/ARM_FAULT、dispatcher/RelaySequencer故障槽与结果ACK、recorder真实放置确认、TaskStatus契约及相关测试。不要混用新旧节点。配置主改动仅gain2→1；原正式默认Q表及deployment_ready/overlap_validated标志均不变。

研究记录器仅为只读诊断，不是正式控制器；新世界文件含本机绝对.so路径，不建议直接当通用发行配置。迁移时保留正常1ms世界及必要诊断，减少高频记录后宜做一次常规烟测，不沿用本轮239.41秒作性能承诺。本次到此结束，不自动再开实验。

后续选项仅供设计审查：底层离散采样/控制同步、按关节执行器建模或合适的effort/velocity接口、正确open_loop_control语义。当前gain1已有有限成功证据，暂不需要大规模控制重构，更不得通过错误的open_loop键修正混入本轮结论。gain10未测试，不推荐。

## 仍有问题的测试与限制

继承全量测试244通过/4既有失败（缺G0锚表等，见audit/inherited-tests.log），READY生命周期相关69用例此前通过，本轮核心源码未改。未伪造缺失证据或扩大测试通过范围。Q表没有训练/替换；没有16组配对，没有语音整合，没有推送或合并PR。

最终分支/提交、原始目录及Q表完整性、残留进程核对见audit/FINAL_STATE.json；所有文件的完整性见MANIFEST_SHA256.txt。
''')
R.joinpath('YURI_FIX_SUMMARY_CN.md').write_text('''# Yuri简短修复摘要

本机已把READY问题分为两层并验证：软件层不再把Action成功/COMPLETED直接等同物理静止，同戳关节消息去重且不降低门槛；控制层确认gz_ros2_control的gain2在500Hz/1ms组合下产生约250Hz的交替速度命令和真实关节振荡。只把实验配置gain降到1，其余控制/几何参数不变。

gain1先通过两臂无工件检查及两件抓放，再通过唯一一次完整8件Rule：四箱各2，148次控制器结果成功，0重试/故障，8次物理READY与匹配槽释放均由原生数据独立复核。gain.5仅空载通过，未当正式方案；gain2失败记录全部保留。

建议审阅并采用“gain1显式配置 + 完整物理READY协议”。原正式工程未覆盖、GitHub/PR未改。已有放置接触段肩误差和夹爪接触深度仍是限制；本次不是硬件安全认证或长期成功率结论。不要删掉故障阻断，也不要将未验证的Q表设成生产默认。
''')
# Exact controller-only patches and compact inventory, with all failed starts visible.
base=W/'src/a1_cell_sim/config/ur5e_cell_controllers.yaml';diff=[]
for tag in ('2','1','0p5'):
 p=base.parent/('ur5e_cell_controllers_stability_gain'+tag+'.yaml');diff.extend(difflib.unified_diff(base.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=base.name,tofile=p.name))
(R/'GAIN_PARAMETER_DIFFS.diff').write_text(''.join(diff))
with (R/'CASE_SUMMARY.csv').open('w',newline='') as f:
 writer=csv.writer(f);writer.writerow(['case','gain','reason','source_commit','start','end','summary_sha256'])
 for c in sorted((R/'runs').iterdir()):
  p=c/'summary.json'
  if not p.exists():continue
  d=json.loads(p.read_text());writer.writerow([c.name,d.get('gain',d.get('control_gain_tag')),d['reason'],d['commit'],d['start'],d['end'],hashlib.sha256(p.read_bytes()).hexdigest()])
print('Wrote required reports, review summary and exact controller patches')
