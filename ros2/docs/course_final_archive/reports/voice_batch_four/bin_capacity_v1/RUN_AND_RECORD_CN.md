# 复现、运行边界与回滚

交付目录：/home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1
工作树：/home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree
分支：experiment/four-slot-bins

## 当前可执行命令（不启动运动）

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
cd /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2
source install/setup.bash
python research_tools/four_slot/check_gate.py
python -m pytest -q src/a1_sorting_core/test
PYTHONPATH="$PWD/research_tools/voice_four:$PYTHONPATH" python -m pytest -q research_tools/voice_four/test_shutdown.py
```

`python research_tools/four_slot/check_gate.py --request-demo`将明确拒绝，未启动任何Gazebo或运动进程。当前没有通过验收的“四格正式GUI演示命令”。不要把旧run_four.sh用于新世界或绕过motion_validated。

## 重复离线规划（独立终端，同一ROS域）

先保留已有audit结果，使用新的证据目录/工作副本避免覆盖；工具默认写入本交付audit。

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
source /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/plan_only.launch.py
# 另一相同环境终端：
python /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/plan_audit.py
python /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/sweep.py
python /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/release_audit.py
```

计划服务禁用trajectory_execution；不要启动Gazebo或真实控制器混入域91。结束对启动终端Ctrl+C，仅清理所属进程。

`generate.py --spread-x`可重建当前尺寸与锚，但会写候选配置和audit，应先另存旧证据。原URDF在audit/robot-static.urdf，本机依赖mesh路径保留用于复现，跨机须指向相同版本mesh并重新核对哈希。

## 尚需完成的最小工程闭环

连续验证释放/抬升指位与源时间戳，失败立即阻断；验证实际抓持姿态和落点包络；覆盖真实carry_clear入口；将已存工件及双臂真实状态同步到规划场景并等待ACK；核对逻辑相机Z；完成独立启动overlay与观察器接线。之后依序空闲停止→黄1→绿4→五件。不得仅改JSON许可位。

若只展示旧空闲GUI，原命令为 `bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_idle_gui.sh`，这是旧场景，不能用来展示本候选四格容量已验证。动态新世界将来必须启用GUI、llvmpipe，并保持ROS_DOMAIN_ID/GZ_PARTITION全链一致。

回滚：停止本候选自己启动的进程后，使用原e4dad34工作树及原启动入口即可；无需reset、无需删研究数据。未改正式分支、未推GitHub、未改原8件配置。保留新分支供审阅，不自动继续实验。


状态：独立候选工程，第二关未完整通过，禁止动态抓放。此次没有启动 Gazebo，没有新的实际 PLACED/READY 或接触测量。原工作树 e4dad347d6ef50d5faa4000fbb7888042e815df6 保持不动。
