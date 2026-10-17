# 源码差异与版本

独立bugfix/physical-arm-ready，续修基线510ef89d958af29634abb24af3a91876d8a96376。原正式工程、其他实验工作树、GitHub分支与Draft PR不修改。

本次新增修改：

| 文件 | 作用 |
|---|---|
| ros2/src/a1_sorting_core/a1_sorting_core/physical_ready.py | 同来源同戳桶去重、保守载荷极值、递增戳差分；区分pause/backward/reset；确认发布来源；门槛不变 |
| 同目录/pick_place_executor_node.py | 获取本机真实rclpy来源接口；来源不明fail closed；ARM_READY附完整窗口/Action证据 |
| 同目录/task_dispatcher_node.py | 仅增加ARM_SLOT_RELEASE结构化证据日志，不新增释放路径 |
| ros2/src/a1_sorting_core/test/test_physical_ready.py | 同戳不计数/不刷年龄、隐藏异常、pause恢复、时钟倒退、来源歧义、运动后begin、真实数据回放 |
| ros2/src/a1_sorting_core/test/fixtures/ready_native_20261017.json | 从本轮原始gzip逐值摘取401帧，本臂关节名及Action来源/原文件SHA；不是生成的仿真数据 |

此前motion_client、relay_sequencer、result_recorder、TaskStatus与生命周期测试修复均保留，详见previous_510ef89/CODE_CHANGES.md。全量差异基于81e55c2，续修差异基于510ef89；均保存到根目录diff。

源码提交：e27247141f6b63cbb19e6c9686cbaaa6a487f954为首次去重版（有来源接口启动错误）；f8e484ccc057638918762786956a21c1d63ca490为实际一件取放版本。最终报告/真实回放测试提交SHA见audit/CONTINUATION_FINAL_STATE.json，不将文档提交冒充现场源码。

工具：tools/observe.py增加来源、PART_PLACED和READY证据采集，有界200万记录/600墙钟秒；run_case.py记录实际源码commit/状态、证据观察器退出即停；新增timestamp审计、只读分析、fixture摘录与报告脚本。旧工具/报告完整保存在previous_510ef89，不建议执行其中历史编辑脚本；复现测试以REPRO_COMMANDS.md为准。

没有改YAML/Xacro/SDF、控制增益/500Hz频率、规划/轨迹/夹爪参数、相机、碰撞体、箱界、Q表、Pixi锁或1ms步长。没有训练/语音整合/推送。协议更严格可能停止旧程序原本继续的任务；不能宣称原吞吐量或8件功能已恢复。
