# 实际规划轨迹审查

本轮没有Gazebo执行。ROS_DOMAIN_ID=86，GZ_PARTITION=green-third-plan-only-20261017，独立日志。只启动MoveIt规划服务，allow_trajectory_execution=false；工具只调用GetMotionPlan/ApplyPlanningScene，不创建FollowJointTrajectory客户端，不启动dispatcher、executor、送料或Gazebo。规划环境已正常SIGINT退出。

原URDF、双臂SRDF、Pilz规划管线、0.2速度/加速度缩放、0.01rad目标约束保持原值。为消除空起始状态的不确定性，显式提供完整两臂/夹爪JointState。另一臂为原standby；这不等于穷举另一臂所有运动。

|位置|原场景7段规划|加入原四箱箱壁后的规划|FCL检查|
|---|---|---|---|
|绿1|7/7成功|末级下探及退出未通过|指杆/绿箱壁相交|
|绿2中格|7/7成功|7/7成功|当前2065配置集合内该格无相交|
|绿3|7/7成功|多段下探及退出未通过|最内指位也相交，最大约1.389mm|

每格检查standby→carry、carry→top、三段梯子、bottom→carry、carry→standby。standby→carry是附加可达性测试，不冒充真实抓取后的carry_clear起点。全体21段、五种对称指位共2065配置；FCL检查两臂原collision网格、arm2与原四箱墙体、名义携带工件与箱壁。总222条相交对记录（绿1 99、绿3 123），是配置/几何对计数，不能当作碰撞事件次数。

中格另按规划输出的位置/速度/加速度重建JTC五次样条，≤2ms采样6590次；对每根手指[-.003,.04]整个平移范围构造精确并集盒，SAT未检出箱壁相交，最小正分离轴间隙约6.258mm。使用历史近似附着旋转及局部XY {-30,0,30}mm九种偏心做携带工件敏感性采样，也未检出箱壁相交。这是命令轨迹离散审查，不是连续时间安全证明，不证明实际跟踪误差和未来真实附着姿态一定安全。

## 未完成范围

用户决定暂不修改后，不继续旧件/已放置工件全组合检查，不开展真实新轨迹/接触检查，不对两臂同时任意运动作安全认证。静态FCL中另一臂只有standby，所有携带工件都是明确标注的模型。绿1已有接触不能被本表掩盖。

复现（只读，无API费用）：激活项目activate.sh并source旧voice工作树ros2/install/setup.bash；设置上述独立domain/partition；ros2 launch green_third_fix/tools/plan_only.launch.py；另终端运行plan_candidates.py（保存原场景输出），随后--walls（保存加箱壁输出）。check_plans.py生成原场景FCL输入；原tools/mesh_box_check查询；center_sweep.py完成中格样条采样。路径均为本机绝对根目录下文件。原工具/新工具不启动运动执行。

原始输入输出、运行日志与哈希存于green_third_fix/audit及MANIFEST_SHA256.txt。
