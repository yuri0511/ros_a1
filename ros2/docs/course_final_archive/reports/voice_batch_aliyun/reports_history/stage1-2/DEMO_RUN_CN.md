# 当前操作入口（阶段2）

请由用户在Ubuntu终端运行，不把Key发到聊天：

```bash
python3 /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/import_private_key.py
```

密钥不回显，保存到~/.config/yuri_a1_voice（0700/0600）；可在同终端明确输入“同意最多4次”授权少量可能付费请求，亦可拒绝。此操作不调用云、不启动Gazebo、不确认任何分拣任务。完成后仅回复“密钥已导入，已授权最多4次”或“仅导入未授权”，不要发送密钥、截图或其内容。

后续用户真实讲话录音入口（先安排好录音时机；本次尚未运行）：

```bash
python3 /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/cloud_cli.py record --output /home/yu/yuri_gazebo_performance/voice_batch_aliyun/audio/<唯一轮次> --seconds 8
```

录音工具会展示峰值/RMS，不覆盖旧WAV；不自动归一化、不改麦克风系统配置。后续实测应明确分开ASR/LLM/校验/确认/ROS受理/执行/物理完成。新ROS语音演示入口尚未开发，不使用旧gain2入口代替。暂时无需打开Gazebo画面，录像应等联合验证通过再安排。
