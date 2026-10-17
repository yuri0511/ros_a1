# 源码修改与边界

独立分支bugfix/physical-arm-ready，基线81e55c2（现有Draft PR代码），正式72ab和原083a未变。未向任何GitHub分支推送或修改PR。

| 文件（ros2/src下） | 修改 |
|---|---|
| a1_interfaces/msg/TaskStatus.msg | 仅补充字符串事件契约，字段不变 |
| core/physical_ready.py | 新增纯逻辑、线程安全、有界历史的同名关节稳定证据检查 |
| core/motion_client.py | 记录本臂Action状态/结果码/未解决目标；standby/return禁失败兜底，核对MoveIt结果码；更正旧GoalStatus注释；原其他运动目标/控制参数不变 |
| core/pick_place_executor_node.py | 初始待机失败不再吞掉；永久失败后不盲发恢复运动；返回必须经物理窗口；区分放置阶段和物理READY；结果确认超时与FAULT；命令去重/原子队列切换 |
| core/relay_sequencer.py | 结果/READY双证据槽释放、乱序/重复处理、故障锁存、迟到事件保护；保留每臂槽/取件互斥 |
| core/task_dispatcher_node.py | 启用新协议，COMPLETED不释放槽，FAULT停止全部新派发，最终publish与停机标志共锁 |
| core/result_recorder_node.py | 按原坐标判据先记录真实结果，再发布PART_PLACED/PART_REJECTED；不因返回失败改写真实放置结果 |
| core/test/test_physical_ready.py | 阈值、窗口、时间、同名/异臂数据、事件分离与恢复测试 |
| core/test/test_arm_lifecycle.py | Action/规划拒绝/超时、executor、recorder真值确认、乱序/冲突/去重测试 |

表中core是a1_sorting_core/a1_sorting_core，test是a1_sorting_core/test。另新增ros2/research_tools/voice_contract.py及test_voice_contract.py，仅JSON→请求提案纯函数；不创建ROS节点、不加载音频/LLM、不真正发服务调用。

未改URDF/Xacro、SDF、碰撞体、箱界、控制器YAML、增益、频率、规划/速度/加速度参数、相机、夹爪、Q表、Pixi锁。READY的位置证据用既有.01rad目标容差，速度仍.3，没有放宽。新结果/READY协议会改变派发时点和结果记录时点，不承诺原性能数字仍成立。

a03bbdc为唯一真实短时轮运行版本；其后修订重复戳处理、独立结果确认、消息乱序/竞态及更严格READY位置判据，均只做单元测试/离线回放，未再次Gazebo。源码diff分别保存运行版和最终版，不能用最终单测冒充现场验收。
