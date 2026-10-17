# 正式OBS录制：尚未就绪

新四件文字计划已正确受理，但唯一物理诊断在首件黄色放置时箱壁接触中止，未4/4；两绿未执行；新四件真实ASR/LLM尚未进行。当前不能录制/剪辑成“四件语音全部成功”。可以如实展示计划、零送料门控、黄色识别及安全停止诊断，字幕标明未完成。

后续需先由Yuri审查黄箱第一格进出/释放几何、原边格接触和停止记录缺口，验证事后停止链路补丁与观察器退出问题。当前不擅改机器人路径或碰撞阈值，不重跑筛选；新实验需重新明确授权。

## 入口与当前限制

```bash
# 安全检查：不启动Gazebo、不调用云API
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_four.sh --check
```

受控入口源码为worktree/ros2/research_tools/voice_four/run_four.py，内部入口为run_four.sh。人工执行模式语法是`bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_four.sh --interactive`，但本轮request_id已经消费，当前不得复用它重跑，也不能改UUID来挑选成功样本。

停止入口：

```bash
python3 /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2/research_tools/voice_four/stop_four.py
```

本轮已经停止，没有运行中仿真。latest-run.json索引真实中止目录；结果/日志/接触/JointState及POST_RUN_AUDIT.json均保留。执行版SHA是62987d0；交付版SHA见LOCAL_COMMIT.txt（含事后仅离线测试的停止补丁）。

## 将来通过验收后的录制画面

Ubuntu录音与ASR原文、LLM JSON、本地校验/人工CONFIRM；Gazebo双臂/四色箱全景；逐件真实PartInfo、CSV与ARM_READY计数。黄色放置+READY后才出现红色送料的时序应可核对。需要GUI时说明新增显示负载，不与headless诊断耗时混比；不启动第二套执行器，不使用旧gain2入口，倍速须标注，屏幕不显示API Key。

新口令仍为“开始分拣，一个黄色、一个红色、两个绿色，先完成黄色。”，当前没有对应新录音或云结果。12sim秒回收按累计台账显示，不能用最后画面块数替代计数。
