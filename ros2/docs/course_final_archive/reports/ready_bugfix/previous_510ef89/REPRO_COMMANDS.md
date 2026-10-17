# 本轮命令与保护

源码分支bugfix/physical-arm-ready，base81e55c2。四包构建/全测试见TEST_RESULTS.md，唯一物理命令见GAZEBO_VALIDATION.md。禁止把历史训练入口当复现前置执行。

原证据审计：python3 tools/audit_existing.py；高频分析：python3 tools/analyze_high_rate.py；只读回放：python3 tools/replay_gate.py。这些只读原数据并写ready_bugfix/audit，不改历史目录。

停止本轮：在latest-case.txt指定的CASE创建STOP；ARM_FAULT/FAILED自动写guard-fault触发世界pause及会话清理。不要全局pkill。world reset不重置ROS锁/计数，不用于热恢复。

完整恢复要先保存CASE并确认/proc无本轮进程、无旧ROS域实例，由人工确认新实验后新进程/新世界。当前未授权新的物理重跑，不能自动启动8件或16组。
