# 给 Yuri 的技术审核说明

上游固定基线为 ros2-dual-parallel / 72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd。本材料记录独立实验，不是部署批准。原始已验收工作区 /home/yu/ros_a1 未覆盖；72ab、8da、V2 的独立工作树分别保存，本地快照见 current_audit/snapshots.json，无远端推送。

建议先审：①返回待机失败时仍可 COMPLETED/释放执行槽，以及首次 standby 失败后 _at_standby 仍置真；②1ms事件等待实现是否适合合入；③Q训练与真实状态边界的差异。deployment_ready、overlap_validated 均保持 false。

可靠性能方案为1ms、v3事件等待、无GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2；批量吞吐以双实例最佳。2ms抓取严重失败，不采用；三实例出现OUTSIDE，不推荐。性能收益与策略收益必须分开。

72ab真实三组配对：Rule184.59秒、旧Q187.96秒，均24/24正确；重试3比7。旧Q确实命中24/24状态，但未证明真实收益。V2默认Q与Rule模型同耗时，V2通用Q接近旧Q，尚无物理优胜证据。历史日志已暴露return_standby/READY缺口，不能以COMPLETED作为物理待命证明。

阅读顺序：CODE_CHANGES、PERFORMANCE_RESULTS、RL_MODEL_RESULTS、SAFETY_AND_LIMITATIONS、DIAGNOSTIC_PAIR_REPORT、REPRODUCE。大型原始证据另包，主包保留完整源码差异、Q表、配置、元数据和关键日志。历史报告中的“未部署/未提交”指其生成当时；以本次诊断和快照记录为准。
