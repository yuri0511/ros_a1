# 无Gazebo测试与构建

最终测试命令：
```
source /home/yu/yuri_gazebo_performance/ready_bugfix/activate.sh
cd /home/yu/yuri_gazebo_performance/ready_bugfix/worktree/ros2
colcon build --symlink-install --base-paths src
source install/setup.bash
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py -q
```

最终结果：**227通过，4失败**；本次新增70个用例全部通过，原157个可通过用例保持通过。源日志audit/tests-final.log及XML。不将4个既有失败排除后冒充全量通过。

既有失败：
- src.a1_sorting_core.test.test_camera_pose::test_world_image_camera_h_fov_final_135
- src.a1_sorting_core.test.test_legacy_anchors::test_relay_section_identical_between_both_tables
- src.a1_sorting_core.test.test_legacy_anchors::test_wired_anchors_match_G0_anchors_v3_json
- src.a1_sorting_core.test.test_legacy_anchors::test_anchor_table_gates_and_belt_stop_are_pinned

三项依赖缺失G0_anchors_v3.json；一项旧relay键集合契约与原配置不符。未伪造证据、未改掉断言。各阶段first/contract/full/release测试日志原样保留。

覆盖：控制器SUCCEEDED+成功码、SUCCEEDED但非零码、ABORT/CANCEL、规划拒绝/错误码、目标受理拒绝、执行超时及cancel请求、物理未静止、位置越界、非有限/缺失/过期/未来/倒退/重复戳、连续窗口中断、另一臂运动、ZONE_CLEAR与READY分别发生、结果/READY乱序/重复/冲突、保留槽、迟到ZONE_CLEAR阻止释放、命令去重、缺放置确认有界故障、正确/OUTSIDE/未知位置原判据确认，以及仅新实例可恢复的受控停机契约。

语音模拟测试覆盖未知字段/枚举、缺真实件/多件歧义/忙件、不得伪造ID、显式左右映射、自动调度/故障拒绝插单、stop不映射杀进程或分拣请求、status只读提案。本次未测试真实ASR/LLM或ROS语音节点。

单测证明确定代码路径和约束，不证明所有物理运动安全。仅一次真实诊断未通过；最终代码没有八件物理回归证据，详见GAZEBO_VALIDATION.md。
