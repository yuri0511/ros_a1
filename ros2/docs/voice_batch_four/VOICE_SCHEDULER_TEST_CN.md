# 构建、调度与安全测试

2026-10-17；四包colcon build成功。全量pytest：341 passed、4 failed；四项失败与历史基线一致，未补造G0_anchors_v3.json。日志audit/build-final.log、audit/pytest-final-verified.log。

既有失败：test_camera_pose::test_world_image_camera_h_fov_final_135；test_legacy_anchors中的test_relay_section_identical_between_both_tables、test_wired_anchors_match_G0_anchors_v3_json、test_anchor_table_gates_and_belt_stop_are_pinned。

新增四件测试覆盖：新中文语义与旧五件不混用；ready先到/计划先到；实际dispatcher送料入口的完整四件模拟时序；黄色PART_PLACED但未READY、READY但未释放槽均不发下一件；同箱回收ACK、消失帧及真实源时间；四件后零额外送料；ARM_FAULT/FAILED/PART_REJECTED锁死与迟到事件不释放；推板不在原home位置或无观测不送料；独立事件台账拒绝提前送料/缺READY；原ResultBook CSV格式按4件而非8件判断。

无Gazebo真实ROS服务测试（audit/ros-transport-final.log）：未确认拒绝、原三绿几何门拒绝、新四件结构可受理但无硬件readiness仍零送料、重复请求拒绝、/sort_task旁路拒绝、停止状态锁存。该工具的confirmed=true是隔离测试字段，不是本轮物理执行批准；没有Gazebo/控制器或真实工件。

观察器无Gazebo只读烟测成功；另在隔离GZ_PARTITION发布明确标为synthetic的21微米模拟指杆/箱壁接触，确认监督故障请求产生。该模拟不是Gazebo接触测量，证据只位于audit/synthetic-observer-guard/。

独立启动入口--check通过；缺少人工授权时在创建任何运行目录、进程或消费记录之前拒绝。启动器不导入云客户端，不调用API。完整启动/清理及新状态机的实际物理联调尚待授权后的单次诊断，不以单测代替。

## 审查后补强的语音送料条件

每次送料都要求同一新鲜逻辑帧中的推板home观测满足原≤0.14m边界、RGB/逻辑数据新鲜、无在途料、双臂无任务槽占用、共享取件区无占用，随后才由原生成器接收一条颜色请求。

回收原协议使用12sim秒删除；删除成功ACK仍不能单独放行。新增真实源时间比较，禁止“旧帧排队、在ACK之后才收到”被当成新观测；必须图像源sim_time晚于ACK sim_time且旧件消失。该修改仅作用voice_batch_mode；保持原控制、阈值和几何。

原executor、motion_client、physical_ready、sequencer、recorder、launch、YAML锚表、URDF/SDF、控制器配置、接口、旧Q表以及8件入口的逐文件相等证明见audit/PROTECTION.json。原工作树未修改。
