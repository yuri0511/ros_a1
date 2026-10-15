# 工件信息:视觉节点发布,调度器与执行器消费
std_msgs/Header header

# 工件唯一编号,与 Gazebo 实体名一致,如 part_1
string part_id

# 类别(由视觉颜色识别得出,如 red/green),决定投放的料区
string category

# 工件在世界坐标系下的位姿(简化实现:取自 Gazebo 真值,README 有说明)
geometry_msgs/Pose pose

# 放置目标箱覆盖(2026-10-06 语言指挥模式):空串 = 无覆盖,执行器按
# 既有 类别->料区 映射投放(抓取段一律不受此字段影响);非空时执行器
# 仅把放置目标改为该箱(仍须在其本臂 bin_names 内,否则判 FAILED)
string target_bin_override
