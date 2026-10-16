# 物理READY与结果协议

## 独立事件

| 事件/通道 | 生产者与判据 | 是否释放执行槽 |
|---|---|---|
| PLACEMENT_FINISHED /task/status | executor完成原放置阶段，仅请求结果判定 | 否 |
| PART_PLACED /task/placement_result | 原result_recorder用原逻辑相机坐标和原箱体内口判为PLACED，已记录CSV | 单独不释放 |
| PART_REJECTED /task/placement_result | 原判据为OUTSIDE/MISPLACED/NOT_FOUND/FAILED，真实类别保留 | 触发故障/保留槽 |
| FAILED /task/status | 执行器任务失败，可先记录失败结果 | 触发故障/保留槽 |
| ARM_READY /task/status | 同一臂控制器成功、物理窗口通过；放置任务还须收到结果确认 | 与匹配PART_PLACED同时具备才释放 |
| ARM_FAULT /task/status | 返回失败、状态不明、时间倒退/超时、缺确认等 | 保留槽并全局停止新派发 |
| ZONE_CLEAR /task/status | 保留原carry_clear物理离区逻辑；verified return也可作为兜底 | 仅放取件区，不释放臂槽 |
| COMPLETED /task/status | 仅在成功回待机后保留旧遥测兼容 | 运行dispatcher忽略其槽释放含义 |

TaskStatus字段布局不变，只补状态字符串契约。需要同时更新executor、dispatcher、recorder；禁止新旧版本节点混用。result_recorder仍以真实坐标判箱，CSV/去重/箱界不改；放置判定时点提前到放置完成阶段，避免return故障抹去真实结果。这一时点变化尚未完整8件物理回归。

## READY条件

- 初始standby和return_standby不以settled兜底掩盖Action失败；要求MoveIt SUCCESS及本臂FollowJointTrajectory SUCCEEDED+error_code=0。
- 本臂没有活跃/未解决的控制Goal；其他臂运动不作为本臂失败。控制超时/拒绝不会被当成功。
- 按原始唯一name匹配六个关节，不用数组位置猜臂，不用别名混取。缺失、非有限值、重名拒绝。
- 位置每关节<=.01rad，取自既有MoveIt JointConstraint目标容差，严于仅用于补偿构型的旧.55rad兜底。未改变规划目标或控制参数。
- 发布速度及新戳位置差分速度均<=原.3rad/s；未提高阈值。本次初始standby有限样本可低于此阈值，但尚不足以认证所有返回姿态。
- receipt monotonic age和header sim age<=原.5秒；未来header超过.01sim秒拒绝。
- 至少50个新的递增戳，连续跨度>=.25sim秒；相邻间隔>.05sim秒中断窗口。重复戳清空窗口、不累计、不刷新；倒退永久锁故障。此窗口是保守候选，尚未通过本机物理验证，不是已充分校准的生产参数。
- 完成Action之后才开始窗口，最多8墙钟秒；结果确认最多4墙钟秒；确认后立即重新核验新鲜物理证据，不能用等待前的旧READY快照。返回Action单次等待保持原60秒边界，不做4次返回重试链；已有共享规划槽等待仍有45秒界。

## 槽与消息顺序

RelaySequencer默认保留旧纯逻辑兼容模式；实际dispatcher显式require_arm_ready=True。旧终态不能释放运行槽。匹配part/arm的正确放置结果和ARM_READY可跨发布者乱序到达，缓存两类证据后才完成；重复消息幂等。冲突结果锁故障；故障臂的迟到READY/ZONE_CLEAR无权释放槽/锁。保留每臂一槽、全局取件锁、queued/done/in-flight去重。

executor增加命令锁，使_busy/待处理队列切换原子化，重复命令不启动第二任务。ARM_FAULT使dispatcher在同一锁下置_halted，取消来料/派发/接力定时器，停止推板新推进；发布命令前再次在同一锁检查，避免timer竞态。已在另一臂执行的任务不被粗暴杀死；本次诊断监督器另行pause世界并正常清理。

## 故障与受控恢复

永久任务FAILED、PART_REJECTED同样停止新派发并保留槽；未知状态不再自动尝试新的恢复运动，原有任务内部有限抓取重试保留。没有自动清除故障或“原地继续下一件”。支持路径：保存原始数据→停止新派发→诊断监督器暂停仿真→SIGINT本轮会话→有界TERM/KILL清理残留→确认无旧会话→人工复核后以新世界/新状态机启动。单元测试覆盖故障旧实例不能恢复、新实例才能重新排队；没有宣称已经验证物理故障后的重启恢复。

不实现真实硬件急停；ROS普通Topic也不提供安全认证。消息来源和唯一运动命令所有权仍依赖受控ROS_DOMAIN_ID/网络环境，不对恶意伪造ARM_READY作安全保证。非split执行路径缺直接控制器成功证据时会fail closed，当前适用验证对象为原双臂pipeline split配置。
