# 续修复现命令（默认不启动新的物理实验）

```bash
source /home/yu/yuri_gazebo_performance/ready_bugfix/activate.sh
cd /home/yu/yuri_gazebo_performance/ready_bugfix/worktree/ros2
colcon build --symlink-install --base-paths src
source install/setup.bash
python -m pytest src/a1_sorting_core/test/test_physical_ready.py src/a1_sorting_core/test/test_arm_lifecycle.py -q
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py -q
```

相关69通过；全量244通过/4既有失败，不能当全绿。

只读复核（输出均在当前ready_bugfix/audit）：
```bash
cd /home/yu/yuri_gazebo_performance/ready_bugfix
python3 tools/continuation_replay.py
python3 tools/analyze_continuation.py /home/yu/yuri_gazebo_performance/ready_bugfix/runs/20261017T023230-ready-native-one-1ms
python3 tools/contact_summary.py runs/20261017T023230-ready-native-one-1ms/instance-1/contacts.csv audit/continuation-native-contacts.json
```

实际已执行物理命令（仅为留档，不作为自动复现前置）：
```bash
PERF_POLICY=rule PERF_CONTACT=1 PERF_PARTS=1 python3 -u tools/run_case.py ready-dedup-one-1ms 1 true 1 160
PERF_POLICY=rule PERF_CONTACT=1 PERF_PARTS=1 python3 -u tools/run_case.py ready-native-one-1ms 1 true 1 160
```

前者启动兼容性失败，后者返回速度超限安全停止。**不要启动8件/重复物理试验直到控制问题被审查。** 不执行previous_510ef89/tools中的历史编辑/终结脚本来重写现有状态。

运行时停止：CASE/STOP文件触发仅本轮正常清理；ARM_FAULT/FAILED/证据观察器退出会先pause本轮GZ_PARTITION的/world/sorting_cell/control再SIGINT会话。禁止全局pkill；reset世界不会清理ROS槽与fault，不用于热恢复。恢复需保存证据、完整停止、检查无残留，复核后新世界/新节点。
