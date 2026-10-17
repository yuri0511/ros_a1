# 绿箱第三格：静态风险阻断

## 结论

**未通过物理放行，未启动五件Gazebo。** 原carry/settle锚存在、FK与限位合格，仍不足以证明进出箱安全。原URDF指杆和原SDF箱壁的独立FCL与盒体SAT检查，在绿3的候选关节插值采样发现指杆/箱壁相交，即使双指已到最内机械限位-0.003m。

这属于静态候选几何风险，不是本轮Gazebo已发生碰撞，也不是整个工作空间不存在任何可行路径的证明。Pilz实际规划轨迹和真实受力状态尚未采集，不能把插值采样冒充真实Action轨迹。由于规格要求风险未排除时先停止，本轮不擅改锚点、格位、URDF/SDF、夹爪限位、规划器或安全阈值。

## 原有格位与FK

绿3中心(0.87,-0.23,0.745)，对应arm2 carry_anchor6、carry_settle_top6/bottom6。使用原双臂xacro生成URDF，独立矩阵FK：XY误差24.08/14.48/21.53微米，各关节在原URDF限位。tool0 Z分别约1.07999756/0.98997381/0.95400420m，tool0不是工件中心，TCP偏移仍沿用原值。

50mm立方在第三格名义轴对齐时，到箱内壁XY余量5mm/10mm；但夹爪指杆12×12×60mm的外包络更大，因此工件在箱内不证明手指能无碰撞进出。相邻第2/3格Y间距50mm，工件名义边缘相切，无正间隙；现有12sim秒回收可能失败，新增模式必须确认旧件确实删除，不能只等待固定秒数。

## 只读FCL与独立SAT

用已安装fcl/assimp编译离线查询工具，不安装任何新依赖、不连接ROS/Gazebo/执行器。读原URDF collision（含原mesh与box）、原bin_green四壁box，原arm1 standby；绿1/2/3各carry→settle_top→settle_bottom关节插值，17采样×3指位(-.003/0/.04)=51状态/格，共153候选状态。查询arm2–箱壁、名义工件–箱壁、双臂几何；不是完整路径规划或动态接触仿真。

查询结果38条碰撞对：绿1为14、绿2为0、绿3为24，均为指杆—箱壁；没有本采样中的臂间碰撞对。绿1已有浅接触历史，此结果不用于重新否定原8/8事实；它说明边格手指余量本来就小。

绿3最内指位-.003仍有11个采样相交；选定case114，arm2关节[-1.2221,-1.8531,1.42355,-1.1412,-1.5708,-1.2221]，arm2_gripper_finger_right_link与green_wall_y_pos_collision，FCL穿入约0.000715471m。用同一两个原box做独立15轴分离轴检查，所有有效轴均无分离，最小重叠0.000715471m。该证据不依赖UR网格保守膨胀或新增碰撞容差。

q=0候选绿3相交最大约3.715mm；完全张开q=.04在选定路径高度下未产生本次查询碰撞，不意味着可以带工件全开下降，也不允许靠改变夹爪动作规避验收。原真实受力/小开度可能不等于这些静态值，正因不能确认，不能假定接触会自动让路后继续。

原scene_publisher_node.py中的_SCENE=[]，静态箱壁没有进入现有MoveIt世界场景；因此只做现有MoveIt自碰撞检查会漏掉这个风险。没有擅自给正式场景加入/删掉碰撞体或扩大允许碰撞矩阵。

## 门控与所需决定

voice_batch_safety.ensure_geometry_review目前在CLI和ROS服务端同时拒绝green>=3，无启动参数绕过；请求的5件计划没有被偷偷降成两绿来获得成功。回收门控能消除相邻旧件占位风险，不能消除指杆—箱壁风险。

需要Yuri优先审阅绿3原进出箱/夹爪包络与真实轨迹，决定下一步独立路径验证/必要修复；不得简单改安全门常量或削弱READY/碰撞阈值。任何新物理短诊断需先说明目的、使用独立世界和安全停止保护。当前没有第三格真实验证、没有五件CSV/5READY。

证据：audit/{green-third-static.json,mesh-cases.json,mesh-queries.txt,mesh-results.csv,mesh-summary.json,green3-independent-sat.json}；生成URDF在audit/robot-static.urdf。FK/查询/SAT源码在tools/及工作树research_tools/aliyun_voice/audit_tools。原G0_anchors_v3.json缺失仍保留，未伪造。
