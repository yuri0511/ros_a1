# 四格轨迹碰撞审查

## 方法与实际结果

原URDF+真实Pilz /plan_kinematic_path，allow_trajectory_execution=false，没有控制器和Gazebo。独立ROS_DOMAIN_ID=91。真实FCL检查静态墙、地板、台基、输送带、推板归位态、挡板、相机立柱和机器人网格；每个目标格规划时其他3格各放一个50 mm虚拟碰撞盒，另一臂设为standby。

最终240 mm候选16格×5段，共80/80返回SUCCESS，所有检查的起终状态有效。rise改为LIN，其他PTP。2 ms最大采样间隔按关节轨迹位置/速度/加速度五次插值进行SAT复核，共72,948样本，所审包络0相交；最小分离指标0.961172885 mm。

此前240 mm全PTP方案退出阶段会擦到当前已释放工件包络，原始plans及dense-sweep完整保存在candidate-spread-240-ptp，不删除失败。220 mm历史中间快照存在部分采样，只能作过程资料，不能用作完整通过证据。

## 不能据此宣称整条真实作业安全

1. approach起点是standby，而实际程序送料抓取后经carry_clear再转箱，尚未覆盖该实际起点。
2. FCL其他臂只用standby，不是任意双臂状态。
3. released cube的±8 mm和held cube的±30 mm是边界假设，没有新场景实测及姿态旋转界。
4. rise指位[8,14] mm条件没有贯穿0.4秒落稳等待及抬升全过程的反馈保证。释放瞬间门控不足。
5. 静态存件碰撞盒尚未接到运行期物件观测与场景ACK；工件移动后旧碰撞盒不可靠。
6. 稠密有限采样不是连续碰撞证明；未以测得跟踪误差膨胀全机器人网格。

结论：条件性离线规划通过，第二关整体未通过；不得改motion_validated跳过。

证据：plans.json记录完整请求、实际轨迹、FCL结果；planning-scene.json记录实际碰撞场景；dense-sweep.json、release-envelope-sensitivity.json、robot-static.urdf保存原始离线计算依据。无Gazebo动态接触结论。


状态：独立候选工程，第二关未完整通过，禁止动态抓放。此次没有启动 Gazebo，没有新的实际 PLACED/READY 或接触测量。原工作树 e4dad347d6ef50d5faa4000fbb7888042e815df6 保持不动。
