# 物理READY协议与同戳稳定窗口（续修最终版）

保持原PART_PLACED、ARM_READY、ARM_FAULT分离。PART_PLACED只由原result_recorder使用原坐标/箱界判据确认；PLACEMENT_FINISHED不是成功凭证；COMPLETED不释放实际dispatcher执行槽。槽释放需匹配part/arm的正确放置确认与物理ARM_READY同时成立。FAILED/ARM_FAULT锁存、停止新来料/派发并保留故障臂槽，迟到COMPLETED/READY不能清除。ZONE_CLEAR仅释放全局取件区，不表示臂可复用。

## 来源、时间和稳定性

- 通过本机rclpy消息信息或唯一图端点确认DDS GID；本机回调不暴露GID，故每次采样要求图上唯一端点。多/无端点、来源变更拒绝，不合并多个发布者。
- 唯一joint name匹配本臂六关节；另一臂正常运动不使本臂速度判据失败。
- 同来源同header归一个桶：样本数不增加、时间跨度不增加、首次receipt monotonic不刷新。相同时间戳的多个载荷保留位置min/max和最大绝对速度；任一超限载荷都能打断稳定性，不能只取首帧或末帧隐藏异常。
- 跨递增戳用两桶位置极值计算保守差分；同戳内不除零、不捏造dt。所有发布速度也必须满足门槛。header始终来自原消息，墙钟只管新鲜度和有界超时。
- 保留位置.01rad、发布/差分速度.3rad/s、新鲜度.5秒、future容忍.01sim秒、间隙.05sim秒、最少50个递增戳及.25sim秒连续窗口，均未放宽/缩短。
- 同戳正常去重；clock不推进超过原新鲜度预算判pause，阻止READY并要求恢复后新窗口；header倒退或clock倒退锁故障，归零重置单独标记reset，均不热清故障。
- 仅在standby/return的MoveIt与本臂轨迹Action成功、error_code0、无未解决/活跃控制Goal后begin窗口。运动途中非零速度不直接触发READY故障，不用轨迹进行时样本充当最终稳定证据。
- 物理观察最多8墙钟秒，结果ACK最多4墙钟秒，ACK后重验新鲜状态。控制Action/规划沿用有界等待。最多2000桶，线程锁保护。

ARM_READY详情现记录控制器结果、无活跃Goal标志、来源GID、窗口长度/样本数/最大速度和位置误差；dispatcher受理释放时记录ARM_SLOT_RELEASE及两臂槽、取件区状态。监督器核对这些字段，观察器退出或故障时暂停本轮世界并清理本轮会话。

## 真实故障时的状态

本轮ZONE_CLEAR于sim23.002正常释放取件区；正确放置于sim30.173确认；return动作成功后仍超速，sim42.263发ARM_FAULT，dispatcher记录ARM_FAULT_HALT。没有ARM_READY、COMPLETED或ARM_SLOT_RELEASE，无后续新派发；故障臂槽仍绑定part_1，取件区此前已安全释放，不能混称“取件区与臂槽一起释放”。该槽结论由匹配FAULT受理代码和日志确认，没有声称采到进程私有内存快照。

故障不能被普通终态覆盖；只允许保存证据、暂停/正常终止本轮、确认无残留、复核后新世界/新状态机的受控恢复。未实现/验证不明姿态热恢复、硬件急停或自动改变控制目标。两臂槽/全局取件互斥的并发组合通过单测，本轮真实只执行arm1一件，不声称双臂回归通过。
