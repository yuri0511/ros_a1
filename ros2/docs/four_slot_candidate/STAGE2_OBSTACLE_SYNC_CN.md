# 已放工件障碍同步

已接入候选规划前检查。每个已占格位必须有新鲜完整世界位姿；使用camera.pose×model.pose做6D变换，不再假设固定camera高度。盒尺寸保持50 mm，旋转保留。ApplyPlanningScene成功后GetPlanningScene读回核验；服务均有界，失败不进入规划。服务完成后再验证源时间新鲜度，避免等待ACK把过时观测当新数据。

当前已释放件在rise规划前以RELEASED_UNCONFIRMED障碍加入，持久化格位仍RESERVED；最终结果确认后才OCCUPIED。缺帧不能删除障碍，也不能解除格位；旧障碍留在场景，后续请求阻断。回收删除/重用仍须原SlotBank证明，不自动清空。

真实MoveIt服务 + 明确标记合成观测的结果：四个箱各1→2→3→4，16次同步成功；输入XYZ包含1 mm偏移、包含0.1 rad绕Z旋转，回读保留尺寸与姿态。4个第五件全部拒绝；4个缺帧案例全部无ACK并保留已存障碍。见live-scene-results.json。不是Gazebo实际四件。

首次服务测试发现MoveIt将primitive_poses转换为CollisionObject.pose+局部单位pose，严格原字段比较误报，已保留失败日志与initial-readback.json。修复为组合两级pose后比较；仅允许64个IEEE-754 epsilon的运算舍入（单位尺度约1.4e-14），不是毫米碰撞容差，不改变几何安全阈值。

未验证：新世界相机pose与native真值、姿态误差界、运行期物件移动及服务等待期间的实际安全性、其他臂非standby状态。不存在已获正式动态许可的结论。
