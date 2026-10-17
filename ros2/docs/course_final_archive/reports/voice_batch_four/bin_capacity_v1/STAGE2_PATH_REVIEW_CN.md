# 实际入口与完整路径碰撞检查

规划输入：原始URDF、未改动四格SDF、候选四格JSON与gain1配置哈希见audit/stage2/INPUT_SHA256.txt。规划仅MoveIt/Pilz，allow_trajectory_execution=false。原始请求、轨迹、FCL有效性结果均在plans.json。

从历史gain1 Rule8完整轮的launch.log提取8个carry_clear结束关节状态（4位小数，量化不确定度约±0.00005 rad）。每个所属机械臂的4个起点分别规划到每格carry，随后carry→settle_top→settle_bottom→LIN rise→return。每格8段、共128段；128成功。不是从standby伪装实际入口。

持件阶段为tool0下204 mm处50 mm附着碰撞盒，touch_links仅夹爪/tool0预期持件接触；没有扩大世界碰撞允许矩阵。附着姿态是审查假设，非新抓取实测。其他3格静态盒一直存在，rise/return加入当前释放件。工作台、墙、归位推板、相机柱、双臂网格参与FCL；另一臂仅standby。

稠密复核127,763个采样点；最大间隔2 ms，五次关节插值。手指精确OBB、持件XY±30 mm、落件XY±8 mm包络内0相交。注意FCL机械臂网格是规划器检查，独立SAT仅覆盖手指/持件与墙和工件，不包含对全臂网格的连续保守膨胀证明。

完整逐段结果：PATH_CLEARANCE.csv。逐格各阶段结论：SLOT_STAGE_STATUS.csv。每格释放和物理退出都标记BLOCKED，因为没有新世界反馈，而非把规划里的固定finger_q当成实际手指运动。

最小见证：红箱slot_4、rise段t=0.372 s，arm2_gripper_finger_left_link与bin_red/slot_4当前释放件包络，SAT分离投影约0.000961172885 m。MINIMUM_WITNESS.json包含两盒变换/尺寸、关节值和指位范围，可逐项复算。不是箱壁0.961 mm，也不是Gazebo测得间隙。

当前实现中还需以真实物理状态确认完全出箱，才允许全开。Action成功不能替代该确认。没有新黄色接触/四件容量/五件任务结果。
