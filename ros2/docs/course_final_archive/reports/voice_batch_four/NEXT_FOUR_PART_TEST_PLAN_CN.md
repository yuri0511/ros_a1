# 下一次四件动态验证前的准备（本轮结束，等待审查）

已完成真实空闲GUI、真实本地WAV按钮、来源隔离预览、空闲暂停/ROS停服务测试。尚未完成四件分拣、新四件ASR/LLM端到端、活动轨迹中的停止链路复验。原request_id审批已经消耗，不得删除consumed标记、重用旧批准或篡改五件记录。

1. 用户审查CONTACT_POLICY_REVIEW_CN.md，决定继续严格正接触即停止，还是授权开发有证据支撑的分级监测。尚未批准时保持现有规则；同一路径仍可能在第一件黄色暂停，不能承诺4/4。
2. 处理dispatcher重复shutdown退出异常；活动运动暂停期间接触/JointState尾部证据仍需在新授权的有界诊断中核验。不要把空闲成功外推到持件/双臂工作。
3. 下一次四件运行需要对明确代码SHA、独立新request_id与黄→红→绿→绿计划重新人工批准。黄色必须PART_PLACED+ARM_READY+对应槽释放后才发红色，所有原门控不变。本轮没有创建新运动授权或可重复执行凭证。
4. 将独立GUI客户端接到未来实际执行实例的GZ_PARTITION，不能另起此空闲世界冒充执行画面。run_idle_gui.sh仅供空闲展示，不能和真实任务同时占用显示来伪装连接。未来需要把GUI启动纳入实际任务监督器并核对server/client分区与世界。
5. 新语音调用前另行说明预计1次ASR+1次Qwen及当时费用/额度，取得用户同意后再导入新录音。当前UI不会调用API，旧三绿五件音频不作为四件证据。真实ROS提交客户端仍未接入UI，未来必须加入安全审计与二次确认，而不是直接把模拟按钮改为送料。

本地使用：
```bash
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_voice_ui.sh
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_idle_gui.sh
```
前者真实录音、模拟计划；后者约一分钟空闲观察后自动停止。可同时摆放窗口，OBS录制时必须显示DRY-RUN标记。本轮可录“界面与空闲场景演示”，不能录成已成功四件动态分拣。正式动态演示仍待上述审批与物理验收。不要运行旧run_four.sh进行重试。
