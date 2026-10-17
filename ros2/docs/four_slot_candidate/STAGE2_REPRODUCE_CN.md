# 复现与后续条件

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
cd /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2
source install/setup.bash
python -m pytest -q src/a1_sorting_core/test
PYTHONPATH="$PWD/research_tools/voice_four:$PYTHONPATH" python -m pytest -q research_tools/voice_four/test_shutdown.py
```

规划工具只用于MoveIt，无Gazebo/控制器：同一环境export ROS_DOMAIN_ID=91，启动`ros2 launch /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/plan_only.launch.py`，另终端同域执行`python research_tools/four_slot/plan_stage2.py`。再执行`FOUR_SLOT_AUDIT_SUBDIR=stage2 python research_tools/four_slot/sweep.py`、`python research_tools/four_slot/summarize_stage2.py`。合成障碍服务验证为`python research_tools/four_slot/test_live_scene.py`。这些工具会写audit/stage2，重现前先另存本轮证据；不可覆盖原始文件冒充新测量。

原日志审计：`python research_tools/four_slot/historical_feedback_audit.py`，仅读取旧日志，不启动模拟器。

本轮基线归档、原4失败、新测试、停止和构建日志都在audit/stage2。初次场景API不存在对象REMOVE错误、序列化读回失败、测试array/list比较错误均留档；修正后重测的是离线工具，不是筛选Gazebo成功轮。

下一步需要：闭合新释放反馈、完整出箱几何与停机距离；测量物件6D姿态和源时间一致性；覆盖实际两臂状态。只有重新审查通过后才有资格进入后续Gazebo关卡。不能开放正式GUI入口、不能改motion_validated绕过。

原Rule工作树不变；未推送、未调用API；停止本轮仅清理独立域91规划进程。原语音UI不动。SHA与差异见本交付目录STAGE2_LOCAL_COMMIT.txt/STAGE2_SOURCE_DIFF.patch。
