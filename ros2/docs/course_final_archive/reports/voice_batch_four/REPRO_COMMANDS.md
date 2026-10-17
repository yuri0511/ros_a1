# 本轮复现/停止边界

执行源码62987d0，真实命令保存在runs/four-20261017T081850-4d18c0/metadata.json，本轮授权已消费。不要复用命令重跑或改UUID筛选样本。

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
cd /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2
source install/setup.bash
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py research_tools/voice_four/test_shutdown.py -q
# 当前343通过、4项既有失败，退出码1如实保留。
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_four.sh --check
```

不在复现脚本中启动新的Gazebo或云调用。所有仿真已停止。
