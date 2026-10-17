# 72ab6f2 本地整合报告

完整基线72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd；本地分支perf/integration-72ab6f2。所有性能数字来自该提交时才标为本轮实测。

远端fetch与ls-remote确认目标分支仍为72ab6f2，远端默认HEAD/main为125c8d2，未自动换成其他版本。新工作树`/home/yu/yuri_gazebo_performance/integration_72ab6f2/worktree/ros2`。原`/home/yu/ros_a1`保持083a515；原perf/fast-simulation-8da998b分支及未提交修改原样保留，没有提交/重置/推送。旧源文件tar、二进制差异及2,560项文件SHA256清单在audit/；收尾再次核验。

迁移采用8da原始/8da优化/72ab新文件的逐文件三方合并，重叠处保留新版相对Q加载器。迁移v3有界就绪、样本证据等待、MotionClient事件、GUI透传和唯一CSV。没有旧文件整文件覆盖新版本。新增policy_params仅为实验参数覆盖；原feed_policy默认q仍保留，所有rule实验/本地启动器明确传feed_policy:=rule，日志要求“退回谁闲推谁”。原YAML、q_feed.py、codex_rl/、measure/、pixi.lock均与72ab一致，见audit/protected-upstream.json。

物理固定1ms、目标RTF4；保留500Hz控制器、增益、几何、规划参数、相机/视觉、判据和互锁。无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2。独立HOME/ROS_DOMAIN_ID/GZ_PARTITION/ROS_LOG_DIR/CSV与原低干扰探针/停止机制保留。启动器支持1/2实例，但本轮策略对照均为单实例；72ab双实例吞吐未额外测量，不能直接引用8da的24.29批/小时作为新版本实测。没有创建2ms正式实验。共享原锁定Pixi环境、独立build/install；四包构建通过。

回归全量157通过、4失败：3项依赖缺失G0_anchors_v3.json（含相机FOV证据），1项既有relay键集合测试与配置不一致。未伪造文件、改阈值或扩大通过范围；明确排除这4项后157通过。旧版缺Q资料的测试在新提交已提供真实包/表后通过。完整日志和XML保留。

离线两张表各50,000回合（默认292状态、通用1,806状态）；默认2222新Q为122.32模型秒，达到同模型DP最优，rule为126.49模型秒。3组真实低干扰配对双方各24件全对，rule平均184.59秒、Q平均187.96秒，Q慢1.83%，未显示稳定物理提速。另有独立功能精密轮；Q精密轮虽8件放对，但发生一次回待命控制器中止、并记录上臂–箱壁接触，不能称全部动作可靠性已通过。三组低干扰配对无控制器中止。所有4轮Q共32次决策命中新表，未知回退/WAIT均0。本地默认继续rule，标记保持false。

训练与DP结果见TRAINING_AUDIT.md、DP_BASELINE_REPORT.md；真实验收与配对见GAZEBO_POLICY_COMPARISON.md。首次启动器因未创建runs目录在启动任何Gazebo前退出，补建输出目录后重试；该本地工具失误日志保留，不算分拣失败。

最终完整性与进程清理见audit/final-integrity.json（生成后有效）。未修改deployment_ready或overlap_validated，也没有安装自定义Skills或新ROS依赖。

