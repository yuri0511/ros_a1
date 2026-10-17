# 四件Gazebo验收：待人工确认，尚未运行

状态：PENDING_HUMAN_EXECUTION_CONFIRMATION。真实启动次数0、送料0、真实新CSV 0、真实新READY 0。341项通过和ROS模拟不能表述为Gazebo 4/4。

计划使用原gain1 profile、1ms、500Hz、v3、无GUI/RViz、X11/llvmpipe、LP_NUM_THREADS=2，feed_policy=rule、voice_batch_mode=true。ROS_DOMAIN_ID=88，GZ_PARTITION/日志/HOME/CSV每轮唯一。原8件正式入口不变。

## 绿1/2已有证据及边界

原_place_pose按照每箱计数%3取格，两个绿色分别对应(0.87,-0.33)和(0.90,-0.28)，carry/settle为原4/5锚，不进入第三格6锚。新世界计数从0开始，路径和几何均未改。原有Rule与Q gain1完整批次已成功分拣各两绿；这是历史验收，不是新语音模式证明。

历史绿1存在接触：control_stability/runs/20261017T031609-gain1-rule-eight/instance-1，part_4实际最低tool0约1.0438m，梯子提前停止后悬停释放；原native接触中左指杆/绿箱负Y壁最多约21微米。绿2原静态/Pilz采样未检出箱壁相交，但不保证未来动态物理绝对无接触。

因此不能把“只用前两格”写成“已排除全部碰撞”。本轮不修改这些路径，也不增加允许穿入深度；新增独立诊断监督器遇到任何robot/bin正深度接触即暂停审查，包含历史浅接触类型，可能在完成4件前停下。该外部暂停不是修改物理碰撞矩阵或原READY阈值。

## 本轮一次诊断的门控

启动前核对已提交且干净的运行源码、完整配置哈希及人工批准的request_id/计划SHA/commit。未授权不启动仿真。批准只消费一次；失败不自动重跑、不改request_id筛选结果。

启动后先记录硬件readiness满足但计划未提交的至少3秒，核对零生成请求/零视觉来料；随后一次性提交人工确认的四件计划，保存意图与响应。结果不明不重发。

记录所有真实送料请求、RGB/逻辑帧计数、视觉PartInfo、状态、JointState（含夹爪）、ControllerState、Action状态/结果日志、Gazebo native stats和contact原始CSV。成功需精确4个唯一part_id、黄1红1绿2全PLACED且目标/实际箱一致、4个原阈值READY及4个原sequencer槽释放，顺序Y,R,G,G。没有这些原始证据就不判完成。

ARM_FAULT/失败/错箱/非法或多送料/控制器异常/无效READY/观察器失联/接触审查事件均停止。受控停止先请求voice_batch/stop阻断新派发，再暂停本GZ_PARTITION世界，SIGINT本轮进程组，必要时限定本组升级清理。不会全局pkill或重置别的实验。

12sim秒回收不会撤销CSV累计台账；两绿之间需原删除成功和之后新鲜消失帧。结束不要求画面保留4块，也不继续期待8件。

本报告在真实轮次完成前不填入成功率、重试、接触次数或耗时数字。原接触CSV的点数不能当作独立碰撞次数。
