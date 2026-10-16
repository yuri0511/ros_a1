# 语音系统接口准备（尚未集成）

本会话未实际取得docs2.zip或对方源码；仅依据用户提供的架构。已在attachments/Documents中查找docs2.zip未发现，不声称已阅读其内容。无真实麦克风、FunASR、Qwen或单臂控制器接入，没有安装模型权重。

## 模块边界

对方单UR5e的voice_bridge、SenseVoice-Small、Qwen可作为输入/意图来源；pick_place_ur5e.py不能覆盖Yuri双臂executor、MoveIt、控制器或调度器。只保留输入解析，通过新voice_to_sort_task_adapter调用Yuri已有/sort_task。Yuri实际物料表、去重、双臂路由、取件互锁和物理READY仍是唯一执行权限。

建议数据流：宿主机或独立推理机采集音频→ASR→本地LLM→固定JSON schema→VM适配器验证/模式仲裁/真实物料选择→/sort_task。新模块只持有ROS服务客户端和只读状态订阅，不持有运动ActionClient，不发布/parts/arrival、不制造part_id，不直接控制关节或夹爪。

## 输入与映射契约v1提案

```
{"action":"sort","color":"red","target":"none"}
{"action":"stop","color":"none","target":"none"}
{"action":"status","color":"none","target":"none"}
```

严格允许action sort/stop/status，color red/blue/green/none，target left/right/none；拒绝额外字段、未知枚举、超4KiB、非JSON对象。对方尚无yellow契约，Yuri第四色须双方明确扩展后启用，不能默默当另一种颜色。

| 意图 | 拟调用/处理 | 拒绝条件 |
|---|---|---|
| sort + none/none | start_batch，仅当前已到物料，不代表自动生成8件 | 未处于人工语音模式、故障、未满足原调度条件 |
| sort + 指定颜色 + none | 从真实当前可用物料查询得到唯一ID后dispatch_single | 0件、多件歧义、忙件、已处理件/陈旧记录 |
| sort + left/right | 必须先有人工确认的显式target_map，再dispatch_to真实件 | 无映射、箱名非法、无唯一真实件 |
| stop | 未来受控停机/暂停新派发API；当前/sort_task没有此命令 | API尚未实现时返回unsupported，绝不谎称已停 |
| status | 未来只读状态快照；无运动副作用 | 数据过期或无快照时明确unknown |

left/right不默认映射blue/red，也不按物理左右猜。对方应提供语义定义；配置至少包含目标箱、坐标参考系、使用场景和确认者。语音颜色不能构造part_red等ID。现有/parts/arrival可作缓存输入，但需要同时跟踪结果、槽/故障、批次epoch和TTL；更推荐由调度器新增只读available_parts/status接口，避免使用私有_catalog或陈旧完成件。该只读接口目前未实现，不虚构已经存在。

调度模式需要明确automatic_rule/automatic_q/manual_voice；默认不允许语音插单绕过自动批次。切换只能在安全边界经调度器受理，最终/sort_task仍检查busy_parts、真实目录、去重和READY。并发请求应有request_id、批次epoch、超时、可撤销的队列受理回执；LLM自由文本不能直接拼接ROS或shell命令。当前v1纯函数未实现跨进程幂等协议，须与对方源码对齐后扩展schema。

## stop的必要设计，尚未编码到ROS

stop优先于sort：停止新来料/新派发，保留持件和取件锁；对已在运动的本臂执行有界受控hold/cancel策略，确认Action终态和物理状态。不能直接pkill、SIGKILL运动节点，不能松夹爪掉落工件，不能把取消结果自动当READY。若控制器或状态不可确认，锁ARM_FAULT，等待人工复核/受控重启。正常暂停与急停必须区分，普通ROS/LLM指令不是硬件急停认证。

建议新增/sorting_control（最终名称待约定）的独立服务，支持pause_new_dispatch、request_safe_stop、read_status，返回accepted vs completed的区别和每臂状态；不要复用原start_batch/dispatch_to偷偷承载stop。该API未实现前，本次纯函数对stop仅返回safe_stop_api_required、implemented=false，绝不触发任何进程或ROS动作。

## 环境与推理解耦

本机实测仅VMware SVGA II可见，没有/dev/nvidia*，nvidia-smi不可用；VM无法直接把Windows RTX4090当CUDA设备使用。未更改VMware/Windows，也未下载模型。不能据宿主机型号推断Guest CUDA可用。

优选在Windows宿主机或另一台已具备CUDA的机器运行ASR/LLM，VM只收小型验证过的意图JSON。可用host-only网络上的受限HTTP/WebSocket桥，不必让推理机加入ROS DDS网络；地址/端口/认证均由人工部署时明确，只白名单对端、不暴露公网、有限队列和并发。断连/超时不触发分拣，重连不自动重放旧sort。状态回报与停止确认必须双向。若只能CPU推理，作为离线延迟评估，不与Gazebo同VM争抢CPU后声称性能未受影响。

不要为这项准备改Windows设置、安装CUDA/驱动或下载SenseVoice/Qwen。模型版本、量化、license、硬件和延迟由交付方提供后再验证。

## 对方必须补交

1. voice_bridge与ASR/LLM源码、package.xml/setup.py/CMake、launch/config、依赖锁及ROS Jazzy平台说明。
2. /voice/command完整schema、成功/失败示例、字段/none语义、yellow扩展、left/right坐标/业务定义、时间戳/请求ID/幂等/错误处理。
3. FunASR/SenseVoice-Small和Qwen确切版本、模型/量化/权重SHA及合法获取方法、license、推理硬件与延迟；无需发送账号或Token。
4. 麦克风采样率、音频设备/权限、唤醒/取消/置信度与人确认机制、错误识别样例。
5. 当前sort/stop/status实现及单臂pick_place_ur5e.py仅供审查其耦合点，不能移植为双臂运动层。
6. 可离线复现的JSON测试、断连/重复/LLM异常测试及source commit。

本次交付voice_contract.py为独立纯函数提案，test_voice_contract.py用模拟JSON/物料目录验证约束。implemented=false的状态/stop不宣称已经集成；源码到齐、安全READY问题通过物理验收后，另行建立ROS适配节点与真实语音端到端测试。
