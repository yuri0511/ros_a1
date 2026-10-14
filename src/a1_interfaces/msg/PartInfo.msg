# 工件信息:视觉节点发布,调度器与执行器消费
std_msgs/Header header

# 工件唯一编号,与 Gazebo 实体名一致,如 part_1
string part_id

# 类别(由视觉颜色识别得出,如 red/green),决定投放的料区
string category

# 工件在世界坐标系下的位姿(简化实现:取自 Gazebo 真值,README 有说明)
geometry_msgs/Pose pose

# 这一轮的编号,如 batch_0001。空串表示旧消息或尚未分批。
string batch_id

# 生成时的给定类别。视觉/模型发布来料时留空,避免把答案交给分类器。
# 记录器只用它做验收,执行器不读这个字段。
string ground_truth_category
