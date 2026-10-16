# 语音接口交付边界

本机Documents、附件与相关目录未找到可核验的docs2.zip或voice_bridge/pick_place_ur5e完整源码。现有方向信息来自组员文档描述，不能据此声称SenseVoice/Qwen已接入。此次仅交付research_tools/voice_contract.py和18项模拟JSON契约测试，纯函数无ROS客户端、无音频/模型加载、无运动副作用。

推荐依赖链：独立语音推理主机的SenseVoice→Qwen意图→/voice/command std_msgs/String JSON→新增适配节点→Yuri原/sort_task；原双臂dispatcher、READY/FAULT、MoveIt、controller不被单臂执行器替换。VMware客户机未配置可用RTX4090 CUDA直通，推理与Gazebo解耦；不在本轮安装模型权重或改Windows。

| 输入 | 当前契约行为 | 实际整合待办 |
|---|---|---|
| sort / none / none | 提议start_batch | 仅手动语音模式且scheduler非fault，真正服务端仍校验 |
| sort / 已支持颜色 / none | 从实际可用目录唯一匹配part_id，提议dispatch_single | 订阅PartInfo及状态，维护新鲜度/去重、请求超时和回执 |
| sort / color / left或right | 未明确配置物理箱映射则拒绝 | 禁止把left/right猜为蓝/红箱，映射须人工定义并通过安全合法性审核 |
| stop | 标记未实现，要求协作停止API | 原SortTask没有stop；需定义停止送料/新派发、当前动作受控处置/确认，不杀进程 |
| status | 只读API待接入 | 返回实际状态，不伪造已完成 |

当前语音枚举源为red/blue/green，核心另支持yellow；扩展yellow需对方意图schema协商，不能暗中推测。色名不生成虚构part_id；可用工件必须实测目录中存在且唯一、不busy。自动Rule/Q模式拒绝手动语音sort。JSON限4KiB、禁止未知字段、错误枚举/歧义请求均拒绝。最终dispatch前服务端再次执行取件区和ARM_READY/FAULT门控。

需要对方提供：完整ROS2源码/package.xml/setup或CMake/launch；voice_bridge源码；/voice/command完整schema、示例和版本；ASR/LLM依赖及部署/权重获取许可；模型提示词与解析器；音频输入/设备配置；网络端口/时延/失败重试；stop/status原实现与安全语义；可复现运行说明及原始测试。只提供视频或报告不足以做真实代码整合。
