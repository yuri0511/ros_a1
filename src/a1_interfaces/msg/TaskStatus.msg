# 任务状态:执行节点发布,调度器与记录节点订阅
std_msgs/Header header

# 任务编号,一次搬运一个任务,如 task_0001
string task_id

# 对应工件编号
string part_id

# 状态机状态:IDLE / PICKING / CARRYING / PLACING / COMPLETED / FAILED
string state

# 附加说明:正常时为空,失败时写原因(如 grasp_timeout / attach_failed)
string detail

# 执行臂 arm1/arm2,空串视为 arm1(兼容旧消息)
string arm

# 与来料相同的批次编号
string batch_id

# 这一件的第几次尝试,从 1 开始。0 表示旧消息没填。
uint32 attempt

# 从这次尝试开始到本条状态的秒数。没测量时为 0。
float64 elapsed_sec
