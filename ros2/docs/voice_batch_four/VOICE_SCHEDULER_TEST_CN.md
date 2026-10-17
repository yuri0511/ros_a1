# 构建与测试结果

四包colcon build成功；物理运行前pytest为341通过、4项既有失败。物理中止后新增两项停止顺序/超时测试，当前为343通过、4项既有失败。日志分别是audit/{build-final.log,pytest-final-verified.log,pytest-after-stop-fix.log}。

4项既有失败：test_camera_pose::test_world_image_camera_h_fov_final_135；test_legacy_anchors的test_relay_section_identical_between_both_tables、test_wired_anchors_match_G0_anchors_v3_json、test_anchor_table_gates_and_belt_stop_are_pinned。没有伪造缺失G0锚表或扩大通过范围。

四件新增测试覆盖新“先完成黄色”语义、原五件不冒充四件、ready/plan不同到达顺序、未确认和未就绪零送料、严格逐件生命周期、同箱回收ACK/消失帧/真实源仿真时间、四件后不多发、ARM_FAULT/迟到事件不误释放、推板home重验、4件CSV动态完成判据。无Gazebo ROS真实服务通信与只读观察器烟测通过。模拟GZ Transport接触测试确认正深度箱壁接触会产生审查停止；目录明确标为synthetic，不是物理实验。

新增voice模式补强：每次送料须用新鲜逻辑帧确认推板满足原home≤0.14m、两臂无任务槽占用、取件区空闲、无在途料及新鲜RGB；同箱下一件须删除成功、旧件消失，且源sim时间晚于ACK，防止旧帧排队后收到被误放行。原8件模式不受此分支门控影响。

## 真实诊断得到了什么

硬件readiness后3.005014秒内尚无计划受理且零请求；实际计划只受理一次；只请求并视觉识别yellow/part_1，未出现红绿送料。记录的15条控制器返回均成功，未见抓取重试记录；无PART_PLACED、ARM_READY或执行槽释放。真正的4件完整流程仍未通过，不能以离线测试替代。

停止流程发现我新增监督器的问题：原版先等待ROS stop服务，约2.118979秒后dispatcher才HALTED，暂停更晚；观测器提前结束原始记录并以-6退出。已改为先请求本partition世界暂停，再请求ROS锁停，停止仿真后才结束观察器，并增加停止时间线/ACK及cleanup_ok记录。两个离线测试验证暂停请求先于ROS RPC，以及暂停超时仍尝试锁停并记录失败。

这些收尾补丁未进行第二次Gazebo验证，不能宣称已消除观察器-6。Python/Gazebo Transport退出日志只有“terminate called without an active exception”，缺少原生堆栈，根因未确认。

原executor、motion_client、physical_ready、sequencer、recorder、launch、锚表、URDF/SDF、gain1配置、接口、Q表及原8件入口相等证明见audit/PROTECTION.json；原工作树保留。
