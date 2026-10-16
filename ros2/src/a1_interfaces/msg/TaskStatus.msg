# 任务状态:执行节点发布,调度器与记录节点订阅
std_msgs/Header header

# 任务编号,一次搬运一个任务,如 task_0001
string task_id

# 对应工件编号
string part_id

# 状态: PART_PLACED=placement outcome only; ARM_READY=verified reusable arm;
# ARM_FAULT=latched, no reuse; ZONE_CLEAR=pick-zone release only.
# COMPLETED is legacy telemetry, never a runtime slot-release signal.
# IDLE / PICKING / CARRYING / PLACING / COMPLETED / FAILED
string state

# 附加说明:正常时为空,失败时写原因(如 grasp_timeout / attach_failed)
string detail

# 执行臂 arm1/arm2,空串视为 arm1(兼容旧消息)
string arm
