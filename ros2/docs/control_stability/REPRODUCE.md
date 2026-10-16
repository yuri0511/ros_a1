# 本机复现与停止

不会自动启动新实验。以下记录本轮已执行的命令；gain2已证明会故障，重跑不能当生产验收。

```bash
cd /home/yu/yuri_gazebo_performance/control_stability
source activate.sh
# 使用现有Pixi编译器/库，无新ROS依赖：
g++ -std=c++17 -shared -fPIC -O2 worktree/ros2/research_tools/control_stability/native_trace.cc -o tools/libjoint_trace_prepost.so $(pkg-config --cflags --libs gz-sim8 gz-plugin2)
cd worktree/ros2
colcon build --symlink-install --base-paths src
source install/setup.bash
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py -q
```

测试244通过/4既有失败，不能当全量全绿。原三个gain文件只差gain；原ur5e_cell_controllers.yaml不改。

本轮实际实验入口（一次一轮，运行间先确认旧轮退出）：

```bash
cd /home/yu/yuri_gazebo_performance/control_stability
python3 -u tools/run_unloaded.py 2.0
python3 -u tools/run_unloaded.py 1.0
python3 -u tools/run_unloaded.py 0.5
CONTROL_COMMAND_PROBE=1 python3 -u tools/run_unloaded.py 2.0
PERF_POLICY=rule PERF_CONTACT=1 PERF_PARTS=2 CONTROL_GAIN_TAG=1 python3 -u tools/run_case.py gain1-loaded-two 1 true 1 240
PERF_POLICY=rule PERF_CONTACT=1 PERF_PARTS=8 CONTROL_GAIN_TAG=1 python3 -u tools/run_case.py gain1-rule-eight 1 true 1 420
```

前两次gain2启动有审计失败/基线故障，所有原始目录在CASE_SUMMARY.csv，日志不覆盖。程序不会自动重跑；故障保护暂停该分区世界、停止新运动/派发，随后只清理本轮会话。

只读分析：

```bash
source activate.sh
python tools/analyze_gains.py
python tools/plot_joint_response.py
python tools/audit_loaded.py /home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight
python tools/contact_summary.py runs/20261017T031609-gain1-rule-eight/instance-1/contacts.csv audit/gain1-eight-contacts.json
```

这些会重写当前control_stability下的派生分析文件，不改原始CSV/gzip，不应在运行期间抢占CPU来做性能比较。

监督停止：在正在运行的CASE目录创建STOP文件；监督器会pause该GZ_PARTITION的/world/sorting_cell/control，再SIGINT本轮会话。ARM_FAULT/FAILED/PART_REJECTED/严重接触/观察器意外退出自动走同一保护路径。禁止全局pkill，不以world reset代替清理ROS锁和fault。恢复仅在保存原始文件、确认无旧节点、人工复核后新世界启动。

本机路径固定/home/yu，迁移机器需要修改工具ROOT/Pixi前缀及只读观测插件SDF路径，不能声称所有脚本在任意机器直接可用。正式可移植的改动为READY协议代码与通过已有controllers_file参数显式选择gain1配置；不应把实验SDF绝对.so路径直接作为正式默认。
