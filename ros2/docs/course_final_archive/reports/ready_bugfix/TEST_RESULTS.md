# 续修测试与构建

四个ROS包构建成功，日志audit/continuation-build-native.log。

- READY与生命周期相关测试：**69通过**，audit/continuation-related-tests-final.log。
- 全量既有core/vision及现有voice纯函数测试：**244通过，4项既有失败**，audit/continuation-tests-final.log/.xml。没有运行语音模型或RL训练。
- 无Gazebo ROS消息来源接口预检通过，audit/rclpy-source-preflight-final.log。真实API只有source/received timestamp及sequence，不含GID；必须结合唯一发布图端点，不再猜测属性。

4项原有失败保持原样：test_world_image_camera_h_fov_final_135、test_relay_section_identical_between_both_tables、test_wired_anchors_match_G0_anchors_v3_json、test_anchor_table_gates_and_belt_stop_are_pinned。三项依赖缺失G0_anchors_v3.json，一项relay键集合契约不符；未伪造锚表或跳过断言。

覆盖：Action成功但运动、拒绝/中止/超时、返回失败、消息缺失/陈旧/非有限、重复/倒退/归零/暂停、同戳异常载荷、来源变更/多源、另一臂运动、任务去重、ZONE_CLEAR/PART_PLACED/READY分离、乱序确认、故障槽/锁与迟到COMPLETED、新实例受控恢复。保留50戳/.25sim秒/.01rad/.3rad/s等全部门槛。

新增真实fixture测试接受本轮初始standby稳定片段，同时拒绝return后的持续腕3振荡；原始gzip SHA和source commit在fixture内。此测试验证保护能拒绝已知危险证据，不代表修复了底层振荡。

完整保留开发中失败：continuation-unit-first.log中一个浮点测试时刻构造问题（测试receipt比now大约1e-15，后改测试now到实际receipt之后）；rclpy-messageinfo-preflight.log记录首次接口假设错误；第一次Gazebo启动错误原样留档。最终测试没有修改实际速度/位置门槛来通过。

真实Gazebo一件未通过READY，8件未执行。单测69/全量244通过不能表述为物理验收通过。
