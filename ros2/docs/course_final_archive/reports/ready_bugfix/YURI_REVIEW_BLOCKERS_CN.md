# 给Yuri的续修审阅摘要（未通过物理验收）

本地bugfix/physical-arm-ready修正了READY观察器对同戳数据的处理：同戳不计新样本、不刷新年龄，保留所有载荷的最坏位置/速度；仍要求轨迹结束后连续50戳/.25sim秒、位置.01rad和速度.3rad/s。PART_PLACED、ARM_READY、ARM_FAULT继续分离，旧COMPLETED不释放臂槽。所有原始几何、规划/控制参数、1ms/v3/无GUI方案保持。

真实一件Rule：初始待机窗口已通过；蓝件正确进蓝箱；return_standby Action成功后腕3持续±.489rad/s振荡，窗口正确拒绝，ARM_FAULT停新派发，未释放臂槽。没有8件通过结果，也没有Q/16组实验。之前2.886rad/s未精确复现，但同类Action成功/物理非静止得到高频数据证实。

请优先审查：gz_ros2_control位置通道gain2/500Hz采样保持的离散稳定性，以及position命令模式JTC成功不保证物理静止的契约。控制器reference/output已在目标，feedback有非零速度、error.velocity为0；不要通过放宽READY门槛绕过。另有夹爪-箱壁和夹爪-工件接触证据原样留存。完整原始数据和分析见ROOT_CAUSE_ANALYSIS.md、GAZEBO_VALIDATION.md及audit/continuation-return-oscillation.json。

源码与阻断证据可以审阅，当前修复候选不应作为“8件恢复/生产就绪”合入。需先确定底层控制修订方案，再单独安全验证。未向GitHub推送或合并现有Draft PR。
