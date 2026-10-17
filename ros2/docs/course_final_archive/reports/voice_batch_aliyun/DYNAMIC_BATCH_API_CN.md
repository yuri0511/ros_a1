# 动态批次接口与时序

## 云候选与本地计划

必需且仅允许schema_version=1、action=sort_batch、counts完整blue/yellow/red/green整数0..3、priority_first、proposed_order；总数1..8，首色库存>0，顺序与库存逐色完全等价、第一项匹配first。拒绝重复JSON键、NaN、布尔冒充整数、负数/小数、未知字段/颜色、伪造关节/控制字段、含糊/否定/不能核对的原话。request_id不允许云输出；真人客户端本地uuid4。

当前真实候选：{blue:0,yellow:1,red:1,green:3}，yellow→red→green→green→green，总5，SHA256=481ae03763ddd82c59a38a46a0ac39d34aad6f50ff1dccf6901b0d78bcc797cb。语义合法不等于物理许可，当前执行安全门因绿3未clear拒绝它。

## ROS服务

/voice_batch/submit，a1_interfaces/srv/SubmitBatchPlan：session_id、request_id、user_confirmed、plan_sha256、uint8四色数量、priority_first、string[] proposed_order；响应accepted/message。服务端再次校验计划和几何许可。session为新进程生成nonce，拒绝旧会话；每个世界只受理一个计划。相同request_id重发返回拒绝/duplicate_request，无第二次送料；不同ID亦不能执行中插批。

/voice_batch/status，std_msgs/String JSON，可靠transient-local最新状态；含session、阶段、request_id、plan/hash、issued、按真实part_id登记的placed/ready/released与fault。没有伪造PartInfo id。

/voice_batch/recycle，std_msgs/String：part_id/deleted=true/sim_time；只有原recorder的DeleteEntity成功回调发出。随后还需更晚的新鲜逻辑相机帧无旧件，才允许同箱下一件。服务受理不是物理完成，也不靠12sim秒定时到期猜回收成功。

/voice_batch/stop，std_srvs/Trigger：锁存HALTED、禁止新派发/送料并停推板。它不是硬件急停认证；若当前动作仍执行，需要有界外层监督器暂停该分区世界并正常清理，不能直接把语音stop映射全局kill。当前未联调真实运动中的stop。

voice_batch_mode默认false，新模式要求显式gain1、1ms运行世界、500Hz/v3、双臂pipeline、feed_policy=rule。旧SortTask服务新模式一律拒绝，默认Rule/Q模式保持旧接口。Q表不参与动态1+1+3批次。

## 生命周期

WAITING_FOR_PLAN（0请求）→PLAN_PENDING_READINESS（已真人确认但未ready仍0请求）→RUNNING（逐件保守串行）→COMPLETED或HALTED。readiness先到/plan先到都恰好开始一次；所有原首件、ZONE_CLEAR、终态和Timer路径统一调用相同reservation门控。

黄色必须真实RGB分类正确、recorder确认PART_PLACED、物理READY通过并由原sequencer释放槽后，才可预留红色。后续同样一件完整结束才处理下一件；同箱另加回收证据。每件记录真实PartInfo和原recorder CSV，并以plan.total个不同part_id全部placed/ready/released + CSV独立复核共同作最终验收，不能依赖原expected_parts=[]下的“本轮结束”日志。

ARM_FAULT/PART_REJECTED/FAILED、视觉颜色不符、额外实体、缺失/错误READY、超时、异常回收ACK均锁存HALTED；迟到COMPLETED不放槽，不无限等待。30秒生成/识别等待、150秒任务等待、45秒回收等待、60秒readiness等待均有独立墙钟约束。实际联合监督与接触实时停止需在绿3修复后再最终联调，不能称当前已经物理验收。

## 已做真实ROS测试，无Gazebo

在独立domain84/partition运行真实服务端/客户端：未确认拒绝，主计划几何门拒绝，两绿的四件候选仅用于测试pending-readiness受理（没有用户物理执行授权、没有硬件ready），重复ID及旧start_batch被拒绝，operator_stop锁存HALTED。全过程/parts/request_spawn消息数0；没有Gazebo、工件或运动，不能当五件实际验收。
