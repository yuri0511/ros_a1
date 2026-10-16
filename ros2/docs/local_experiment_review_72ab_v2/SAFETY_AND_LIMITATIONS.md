# 安全与尚未解决的问题

deployment_ready=false，overlap_validated=false，不作安全认证。

历史72ab Q精密轮part6 arm1 return_standby规划拒绝：MoveIt预测arm2_gripper_base与gripper_finger_left_link碰撞，随后仍发布COMPLETED/释放执行槽；part8初始standby发生控制器goal_time abort，代码警告后仍置_at_standby=True。预测碰撞不等于发生实际接触。需Yuri审阅状态边界并修复，不能用逻辑READY或COMPLETED冒充物理待命。

原MotionClient _joints_settled使用位置误差0.55rad、速度0.3rad/s、新鲜度0.5秒的已有宽容判据（用于下垂补偿）。本次仅记录该判据和连续关节偏差，不扩大阈值；通过该判据不证明严格标定构型。

原有接触记录有真实mesh传感器及正样本，但0观测双臂接触不等于全状态无碰撞。8da v3有夹爪基座-工件.616mm、观测14ms；72ab精密Q有upperarm-黄色箱壁约.197mm、观测4.012秒。接触点数不是碰撞事件次数；时间戳分辨率与采样间隔需分开。

原结果记录器12仿真秒回收已放工件，因此最后世界中不保留全部8件；成功标准沿用原XY判定，并补充native pose/contact诊断，不能宣称长期落稳。推板日志到home阈值/发停命令不等于零真实速度。新诊断的pose速度为有限差分估计，除非有native joint velocity证据，不标为真实关节速度。

四个既有测试失败：三个缺G0_anchors_v3.json证据、一个旧relay键集合断言。157其余通过；未补造文件/扩大通过范围。几何/碰撞/规划/控制器参数、原始工作区和历史Q表不变。
