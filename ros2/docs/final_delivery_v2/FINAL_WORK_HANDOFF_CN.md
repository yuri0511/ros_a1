# Yuri A1：最终工程交接 V2

日期2026-10-17；对象Yuri与下一位接手者。本轮项目收尾，不开启新研究；没有训练、没有16组实验。先读FINAL_SYSTEM_STATUS_CN.md和RULE_Q_VALIDATION_CN.md。

## 1. 环境、保护与版本链

Windows宿主/VMware Workstation26.0.1；Ubuntu24.04.5 LTS客户机，12vCPU、24GB RAM、100GB虚拟盘；ROS2 Jazzy、Gazebo Harmonic8.10.0、MoveIt2.12.4，既有Pixi锁定环境；实际硬件/软件细节以旧final_delivery交接与control_stability版本审计为准。未改宿主、VM设置、锁文件、几何、规划控制缩放；未重新安装依赖。

仓库https://github.com/yuri0511/ros_a1；正式分支ros2-dual-parallel固定72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd。原始已验收工作区/home/yu/ros_a1/ros2仍处083a51568e20089d59f9c742da60637f2f03045b，不作为本轮写入目标。原日常~/run_yuri_ros2.sh未被替换，可能仍启旧gain2，不用于新候选演示。

历史8da性能快照86786e0；72ab整合76dacf3；模型V2 e2a82c9；早期审核81e55c29241c7e8dc089e73a10409e965b439838；READY候选a21ee9b7b0d9631b86dbcca5e15f1b0e556de54e；真实gain1 Rule运行ed76c35839ffd878cb8134e5bdc6de5f6c050364；实验报告提交705d1af26b705aa753723b1ecd3a4200434e7710。ed76→705无ROS运行变化。最终整合本地分支delivery/final-gain1-ready；远端只更新review/yu-gazebo-performance-rl-v2。真实上传SHA及PR状态见GITHUB_FINAL_RECEIPT.md，避免文档虚构自己的自引用SHA。

Draft PR https://github.com/yuri0511/ros_a1/pull/1；审核分支https://github.com/yuri0511/ros_a1/tree/review/yu-gazebo-performance-rl-v2 。不推main/正式分支，不force，不自动合并。GitHub认证者yuri0511，实际修改作者Ping <311167715+Pingggg567@users.noreply.github.com>，AI辅助，未冒署Yuri。

## 2. 本地重要路径

所有路径在/home/yu/yuri_gazebo_performance/下：

| 目录 | 用途 |
|---|---|
| fast_sim_8da998b/ | v1/v2/v3、RTF、1/2ms、1/2/3实例与历史原始数据 |
| integration_72ab6f2/ | 新代码迁移、prefeed=false重训、3组真实Rule/Q配对 |
| model_calibration_v2/ | 完整轮次分组校准、留出验证、50k离线表及DP |
| yuri_review/ | 中文审核ZIP/82.95MiB证据ZIP/旧截断诊断 |
| final_delivery/ | 上次GitHub交付与81e55c2工作树，原样保留 |
| ready_bugfix/ | 状态协议、重复时间戳修复及旧失败证据 |
| control_stability/ | gain单因素、运行ed76、完整Rule8件原始流/native/contact、705报告 |
| final_delivery_v2/ | 本轮工作树、唯一Q诊断、报告、回执、小包与索引 |

最重要Rule证据control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/{launch.log,results.csv,high-rate.jsonl.gz,native-joints.csv,contacts.csv}；原始metadata/summary在上一级。独立复核control_stability/audit/loaded-20261017T031609-gain1-rule-eight.json与gain1-eight-contacts.json。所有旧数据/Q表保留，不运行会覆盖其派生报告的旧write_reports/finalize脚本。

## 3. 架构与协议

四包a1_cell_sim、a1_interfaces、a1_vision、a1_sorting_core；Gazebo物理/相机→ros_gz_bridge→视觉PartInfo→dispatcher→双executor→MoveIt→两套joint_trajectory_controller→gz_ros2_control位置接口。/clock来自独立世界，/world/sorting_cell/stats为native真实时间统计。

dispatcher单送料槽/实际工件目录，RelaySequencer维护两臂槽与共享取件区。ZONE_CLEAR释放共享区，不把本臂当空闲；recorder按原落箱条件确认PART_PLACED；return_standby严格Action成功后physical_ready以新鲜时间单调单来源观测做50样本/.25sim秒窗口，位置.01rad、速度及差分.3rad/s；ARM_READY才释放对应槽。ARM_FAULT/FAILED/拒绝结果锁存，旧COMPLETED不覆盖故障。重复stamp不产生新样本或假新鲜度；暂停/时钟倒退/重置有界处理。恢复不是重新发COMPLETED或reset世界后沿用旧ROS锁，而是先保存证据、停本轮、人工审查后新世界。

gain2、500Hz、1ms本机可导致约250Hz持续速度振荡；gain1单因素消除所测持续振荡。Action SUCCESS表示控制器目标判据，不保证物理静止，尤其位置接口及速度误差语义不能混淆。正式gain2留作历史，必须显式gain1；原open_loop键与真实open_loop_control差异在参数审计中记录，未混入本修复。

## 4. 构建、测试、启动和停止

```bash
cd /home/yu/yuri_gazebo_performance/final_delivery_v2/worktree/ros2
export YURI_PIXI_PREFIX=/home/yu/ros_a1/ros2/.pixi/envs/default
export CONDA_PREFIX="$YURI_PIXI_PREFIX" PATH="$YURI_PIXI_PREFIX/bin:$PATH"
export CMAKE_PREFIX_PATH="$YURI_PIXI_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
for activation in "$YURI_PIXI_PREFIX"/etc/conda/activate.d/*.sh; do source "$activation"; done
colcon build --symlink-install --base-paths src
source install/setup.bash
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py -q
python research_tools/validate_delivery.py  # 只读Q检查，不训练/不启动GZ
# 需X11 DISPLAY，默认已登录桌面为:0；脚本不改X11权限
bash research_tools/final_system/run_rule.sh
```

脚本默认日志~/yuri_final_runs/唯一时间戳目录，打印准确路径；显式Rule/gain1/fast_sim_1ms/v3/LP2、无GUI/RViz但相机仍渲染。其420秒墙钟超时、Action异常、ARM_FAULT、错误槽释放触发受控暂停与本进程组清理；不全局pkill。正常仅八条PLACED及八匹配READY释放结束。可Ctrl-C或`touch <本轮打印目录>/STOP`。不要在没有保存目录/partition归属时kill其他历史任务。

若需要手工安全暂停，仅在确认该实例环境时：

```bash
ROS_DOMAIN_ID=82 GZ_PARTITION=<metadata中的本轮分区> gz service -s /world/sorting_cell/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: true'
# 随后让所属监督器通过STOP/信号收尾；暂停不等于清理ROS锁。
```

跨机器改YURI_PIXI_PREFIX为本机已有环境；通用世界/config用包资源，不依赖/home/yu绝对.so。新增包装脚本仅静态/监督逻辑检查，未冒称已用它重跑Rule；物理核心/配置与ed76一致，去除只读诊断插件后世界语义相同。示范录像需另行安排，不能把headless日志当现有GUI视频。

## 5. Q-learning与语音

正式原codex_rl/schema1及原默认表不替换；V2独立codex_rl_v2/schema2。V2通用表SHA18c4b343b0c359b2c740d9d6eeabef273be3d93e4e573ee28a32b53216977e2c，规范配置SHAce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b，配置文件SHA574a24115d663c8a73c30f897f6b824bd32a6357803147e4a35e76b6339f6b01。prefeed=false、deployment_ready=false、overlap_validated=false。

Q只根据已观察remaining/臂阶段/颜色/POST历时离散ETA选择，不能读未来事件真值；离线模型WAIT事件队列与ROS触发时机有近似，DP全事件信息只是模型上界。当前50000回合都是模型离线，未再训练。旧真实三配对Rule184.59秒Q187.96秒、各24/24，不能以模型122.32秒宣称物理提速。新物理READY时序并未重新校准，Q本轮仅一次诊断，精确日志/结论/停止原因见RULE_Q_VALIDATION_CN.md。

在独立实验需要指定V2时，用policy_params YAML设置q_package_dir=本工作树codex_rl_v2、q_config_path=config/timing.v2.json、q_table_path=artifacts/v2/q-v2-general.json、q_batch=[2,2,2,2]；相对路径可由q_feed解析，启动检查Q_AUDIT_LOAD实际路径/哈希，不能只看feed_policy=q。此次不自动再开Q轮。真实Q一轮不足以判断性能。

语音仅JSON纯契约/18项测试；没有ASR/LLM/ROS客户端实际整合。见VOICE_INTEGRATION_STATUS_CN.md所列源码/安全stop API缺口；真实part_id、明确目标映射、手动模式/FAULT守门不可绕过。VMware未有可用4090 CUDA，建议推理与仿真解耦，不安装权重。

## 6. 已验收、历史失败与限制

4包构建完成（CMake开发/依赖警告见日志），244测试通过，4旧失败：camera_h_fov断言、relay表键集合差异、两项G0_anchors_v3.json缺失断言。未伪造锚表、未跳过。第一次新工作树测试未source install导致导入收集错误，构建并source后正规测试如上；不是源码回归。

Rule实测8/8/148 Action/8 READY/0重试FAULT；239.41墙钟/143.177sim为高频诊断，不能与旧低干扰做性能比较。原生/ROS稳定窗口并不解决所有接触风险：放置约.137rad肩误差在gain2亦存在；finger-part最大穿深约14mm，finger-bin/conveyor接触保留；没有已记录臂间/主体撞箱类别不等于全局无碰撞证明。

旧2.886rad/s诊断在第一件后受控截断、无完整CSV、Q未启动，不能作为正式失败率；gain2后续持续约250Hz证据及gain1修复需连同保留。物理阈值没放宽，但仅有限场景/一轮Rule，非长期认证。最终Q状态依单独报告，不能覆盖Rule已有效证据或筛选重跑。

## 7. 数据、完整性和下一位任务

本轮audit含ed76→705全差异清单/空运行diff、逐文件哈希、可移植世界对比、Q静态验证、构建/测试日志、历史保护清单核对。EVIDENCE_INDEX.md列本机原始流及SHA256；Git只上传必要小摘要/源码/报告，原82.95MiB证据ZIP不上传。旧ZIP和所有工作树均保留。用`git show <SHA>:<路径>`或独立新工作树恢复源码，禁止reset --hard。通过MANIFEST_SHA256与原始索引核对数据，修复新发现的问题另建分支；不要改旧CSV来配合报告。

Yuri优先审查：整体READY生命周期和fault槽；gain1有限适用性及接触/放置偏差；四项历史测试材料；新Q真实诊断与旧模型时序不一致；源码齐全后才接语音安全服务。正式扩展实验/控制研究/训练需另立任务；本轮完成后不自行启动新仿真或合并Draft。

## 本轮Q诊断最终补充

唯一Q轮8/8正确、8物理READY、149底层轨迹控制器全部成功、8决策全命中/0回退/0WAIT。part_3有一次MoveIt起始状态预测碰撞拒绝（伴空JointState），原有界恢复后重试1次；因此是带异常记录的功能完成，非零异常验收，不能据此声称提速。另记录到10个腕/夹爪基座与工件接触点（最大0.146mm），指尖最大穿深14.909mm。详见RULE_Q_VALIDATION_CN.md及原始日志，后续不自动继续实验。
