# 代码变更与行为范围

精确补丁 source/72ab-to-v2.patch 基于72ab，包含新增文件，可用git apply --check。三个历史基线补丁见current_audit；source/local-experiment-snapshots.bundle可恢复本地提交。未混入第三方依赖。

| 文件 | 原因及影响 |
|---|---|
| a1_cell_sim/launch/cell_bringup.launch.py | noGUI参数、fast启动生成、唯一传感器/世界选择；默认保留原路径 |
| a1_sorting_core/launch/sorting_demo.launch.py | 透传GUI/RViz/世界/CSV/fast开关，增加仅调度器policy_params覆盖 |
| a1_sorting_core/motion_client.py | Future/事件完成唤醒替代固定轮询，原超时/取消/控制目标保留 |
| a1_sorting_core/pick_place_executor_node.py | fast_sim_waits默认关闭；足量新鲜视觉证据后减少额外等待，抓取物理验证与重试保留 |
| a1_sorting_core/task_dispatcher_node.py | 有界10项启动就绪检查；事件派发；Q加载/选择审计；安全门控保留 |
| a1_cell_sim/worlds/fast_sim_1ms.sdf | 独立世界，max_step_size=.001，目标RTF4仅限速配置，实际值从stats测量 |
| a1_sorting_core/test/test_fast_sim_waits.py | 等待与门控回归 |
| codex_rl_v2/dual_arm_rl/model.py,qtable.py,train.py | 独立schema2模型、部分可观测ETA、完整阶段成本与经验profile重采样；原codex_rl及q_feed未覆盖 |
| codex_rl_v2/experiment_tools/*.py | 数据审计、训练、DP、适配器验证和报告驱动，含本机来源路径 |

8da分支额外有2ms失败世界及当时YAML改动，仅历史对照，未移植其YAML到72ab。72ab正式feed_policy默认q仍保留；本地实验启动器显式rule。诊断观察/停止补丁独立于上述性能快照，见diagnostic目录，不是正式状态机修复。

## Python文件完整清单

- `ros2/codex_rl_v2/dual_arm_rl/__init__.py`（A）。
- `ros2/codex_rl_v2/dual_arm_rl/evaluate.py`（A）。
- `ros2/codex_rl_v2/dual_arm_rl/model.py`（A）。
- `ros2/codex_rl_v2/dual_arm_rl/qtable.py`（A）。
- `ros2/codex_rl_v2/dual_arm_rl/train.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/adapter_audit.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/assess.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/evaluate_v2.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/extract_traces.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/finalize.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/freeze_model.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/oracle_general.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/train_v2.py`（A）。
- `ros2/codex_rl_v2/experiment_tools/write_reports.py`（A）。
- `ros2/src/a1_cell_sim/launch/cell_bringup.launch.py`（M）。
- `ros2/src/a1_sorting_core/a1_sorting_core/motion_client.py`（M）。
- `ros2/src/a1_sorting_core/a1_sorting_core/pick_place_executor_node.py`（M）。
- `ros2/src/a1_sorting_core/a1_sorting_core/task_dispatcher_node.py`（M）。
- `ros2/src/a1_sorting_core/launch/sorting_demo.launch.py`（M）。
- `ros2/src/a1_sorting_core/test/test_fast_sim_waits.py`（A）。

V2 evaluate.py和__init__.py为上游包复制；独立新增的分析工具逐项见补丁。
