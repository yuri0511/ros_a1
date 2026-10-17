# Yuri A1 — 阿里云语音动态批次分拣：Codex 实施任务

> **直接交给 Ubuntu 虚拟机里的 Codex 执行。** 本文件是执行规格，不代表任务已经开发、测试或验收成功。用户已明确同意开发和必要的有限 Gazebo 验证；遇到授权、凭据或安全门槛，先等待人工确认。

## 0. 项目背景与任务目标

- GitHub：`https://github.com/yuri0511/ros_a1`
- 上游正式基线：`ros2-dual-parallel`，`72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd`，**不得改写**。
- 用户现有审核分支：`review/yu-gazebo-performance-rl-v2`，本次检查时 HEAD `f6a7afc8ed7451e161023c6039b5f2494ea55ab6`，Draft PR `https://github.com/yuri0511/ros_a1/pull/1`。开始前重新 fetch 和核实，不能覆盖他人后续提交。
- 已验收工作树：`/home/yu/yuri_gazebo_performance/final_delivery_v2/worktree/ros2`；**原样保留**，不要直接在这个目录进行高风险实验。
- 以前验证成功的执行基线：`gain=1.0` 控制器 profile、1ms 物理步长、500Hz 控制器、完整 `PART_PLACED / ARM_READY / ARM_FAULT` 物理 READY 协议、v3 快速事件等待、Gazebo headless + X11 llvmpipe；Rule 8/8，Q 一轮 8/8（伴一次 MoveIt 规划拒绝/恢复），不夸大为长期无异常。
- 共享目录：Windows `D:\Yuri_A1_Shared`；Ubuntu `/mnt/hgfs/Yuri_A1_Shared`。组员 `docs2.zip` 在 `incoming/`，仅作为接口和文档参考，其内无已核验完整 ROS 语音源码。
- 现有 `research_tools/voice_contract.py` 是早期 JSON 模拟契约，并非真实语音客户端。必须检查它及其测试，扩展不得破坏历史成果。

**唯一主演示：** 用户在 Ubuntu 虚拟机麦克风说：

“开始分拣。三个绿色，一个红色，一个黄的。先分拣黄色。”

实现如下可验证闭环：

1. Ubuntu 录音 → 阿里云百炼 `qwen3-asr-flash` 中文语音识别 → `qwen-flash` 非思考模式抽取并规划任务。
2. 界面/终端先展示中文识别结果和安全审查后的计划，例如 `blue=0, yellow=1, red=1, green=3; first=yellow; total=5`；用户明确确认后才提交。
3. 内部 ROS2 调度器以**新独立 voice_batch 模式**接收计划，禁止接令前自动产生第一件，按安全的逐件送料机制执行；首个被生成、识别和成功完成的工件必须是黄色，之后在剩余需求中执行红色和三个绿色（具体余下顺序由经验证的计划决定）。
4. 真实相机对工件颜色进行 RGB/HSV 识别，逻辑相机提供仿真位置；两台 UR5e 延用原 MoveIt、预标定锚点、取件区锁、送料槽、每臂执行槽、物理 READY/FAULT 和结果记录器；阿里云模型**不能直接控制机械臂关节、推板或 Gazebo 实体创建**。
5. 成功时恰好 5 个不同 `part_id`，`yellow=1, red=1, green=3, blue=0`，且每件 `PLACED` 目标/实际箱相同、`ARM_READY`/槽释放匹配，无多余来料；给出 CSV 与事件证据。碰撞、规划拒绝、状态故障必须留证，不得择优重跑或声称零异常。

## 1. 初始审查与工作区隔离

- 首先检查 `git status`、worktree、最新远端 ref 和所有已有报告/启动器，确保保留未经提交的文件、原始数据和 Q 表。
- 从审核版本在新工作树创建本地分支，例如 `feature/aliyun-voice-batch-20261017`；新工作目录：`~/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2`，日志与报告写同级专用目录。禁止 `reset --hard`、force push、覆盖原版本、自动合并 PR。
- 核实四包 `a1_cell_sim / a1_interfaces / a1_vision / a1_sorting_core` 与现有运行配置、Pixi 锁定环境；不要混用 ROS1 根目录 `src/`，不升级现有 ROS/MoveIt/Gazebo/Pixi。
- 先审查 `task_dispatcher_node.py` 中 `_start_ready_feed` 自动首件、`_request_spawn_and_start_belt`、`_on_zone_clear`、`_on_status` 及定时器；`auto_start=false` **不能单独阻止按需送料**。还要审查 `spawn_parts.py`、`result_recorder_node.py`、`pick_place_executor_node.py`、`sorting_demo.launch.py`、`sorting_params_pipeline.yaml`、`SortTask.srv` 和所有 READY/FAULT 事件。
- **绿色第三格审查**：上游 `sorting_params_pipeline.yaml` 已为各箱预留三个格位：`x=[-0.03,0,-0.03]`、`y=[-0.05,0,+0.05]`；执行器 `_place_pose()` 用 `_place_counts % 3` 选取位置，并有绿箱第 3 格对应的 carry anchors。此前 gain1+READY 的正式批次主要是每色2件，第三个绿色在新物理配置下尚未实测。**静态检查第三格碰撞、IK、相邻件接触、放置格序号、回收策略**，不得自动修改锚点、几何、安全阈值或规划器来掩盖缺口。建议 voice 模式单色最多 3 件，超过直接拒绝；不能因为格位可循环就允许第 4 件继续叠放。
- 结果记录器当前 `expected_parts=[]` 时 `_batch_finished()` 不会发结束判断；需设计一套针对动态 5 件的可靠完成判定，不能硬编码 8 件，也不能仅凭第一条“本轮结束”日志判断完成。`recycle_delay` 会清除成功工件，最终未必在画面中同时存在全部 5 件，以累计台账而不是末帧总块数为准。

## 2. 阿里云模型与本机音频：先脱离 ROS 验证

- 先对照用户给的百炼官方文档 `https://docs.bailian.console.aliyun.com/zh/model-studio/non-realtime-speech-recognition-user-guide` 核对 `qwen3-asr-flash` 的北京地域地址、Base64/WAV 上传形式、请求结构和返回字段；`qwen-flash` 的精确当前模型 ID、非思考模式和 JSON 结构化能力也须据官方文档核实，禁止凭猜测硬写接口。
- **同一个百炼 API Key** 用于 ASR 与文字指令理解；不调用 DeepSeek，不安装几 GB 的本地 ASR/LLM 模型。Ubuntu 麦克风此前可录人声，但电平比 Windows 低；先用 existing ALSA/PipeWire 工具录 5–10 秒的 16kHz 单声道音频，必要时只做受控的本地软件归一化，不随意改虚拟硬件或系统音频配置。
- Key 原先保存在 Windows 桌面 `qwen_api.txt`，**不要**放进 Git、共享文件夹、日志或参数 JSON。请通过 Ubuntu 本地交互式不回显输入（如 `getpass`）让用户粘贴 Key，导入当前 Ubuntu 用户专有的私有配置或环境（目录权限0700，文件0600），只报告“已加载/调用成功”，不显示密钥。人工介入时暂停等用户。
- 先在纯文本路径测试 Qwen 指令解析，再小规模真实 ASR，优先这句原话及“黄的”口语表达；不要将无意义的识别偏差强行纠正为成功。初次 API 调用前向用户提示预计次数及收费可能，限于必要的少量请求，必要时请用户确认。
- 统一任务结构示例（局部建议，可按已存在接口选择等价严格 schema）：

```json
{
  "schema_version": 1,
  "action": "sort_batch",
  "counts": {"blue": 0, "yellow": 1, "red": 1, "green": 3},
  "priority_first": "yellow",
  "proposed_order": ["yellow", "red", "green", "green", "green"]
}
```

- `request_id` 由可信本地代码生成，不能由大模型任意决定；输出必须通过本地严格 JSON schema、字段白名单、颜色集合、非负整数、总数≤8、单箱≤3、首色匹配、候选顺序精确等价于库存数量及唯一命令类型检查；不符合则拒绝、说明原因并请求重新表达。**大模型生成的是候选任务计划，不是已批准的执行命令**。
- 清楚区别 `ASR 已识别`、`LLM 已解析`、`计划已校验`、`用户已确认`、`ROS 已受理`、`已实际执行`、`物理任务成功`，不可只看到文本就称 Gazebo 成功。

## 3. ROS2 动态批次接口：优先最小修改

- 不要把数量 JSON 偷塞进 `/sort_task` 的 `part_id` 或 `target_bin`。可以在 `a1_interfaces` 新增独立的 `SubmitBatchPlan.srv`（或等价**类型明确**的新服务），包括 `request_id`、四色数量、`priority_first`、严格校验的颜色执行顺序及 `accepted/message`。原 `SortTask.srv` 保持兼容。
- 独立 `voice_batch_mode` 参数默认 `false`。为 `true` 时：
  - 启动后检查 Gazebo/控制器/相机 readiness，但等待用户计划和确认之前**绝不发送第一条 `/parts/request_spawn`**；不能只把 `auto_start` 设为 false。
  - 原有 `start_batch` 只入队已存在工件、不是创建整批新工件，不可直接用它冒充语音任务入口。
  - 原初始自动送料、`ZONE_CLEAR` 后送料、终态后送料、挂起 Timer/重试等**全部统一受计划门控**；状态可定义为 `WAITING_FOR_PLAN → PLAN_PENDING_READINESS → RUNNING → COMPLETED / HALTED`。计划受理最多一次，request_id 幂等；执行中新增/重复请求拒绝；崩溃/重启要避免复用旧请求和幽灵首件。
  - 第一件必须是黄色，并按强优先语义：黄色 `PART_PLACED` 且物理 `ARM_READY` 前，不允许生成下一件，确保“先分拣黄色”不仅是先发请求。后续四件可以按经校验的顺序在旧互锁允许的条件下逐件推进，不允许一次抛多个到取件区。
  - 执行顺序由大模型提出、ROS 校验并受实际臂空闲/互锁条件约束；如果某候选序列不安全则等待或拒绝计划，不允许 LLM 绕开锁直接发布工件。语音批次强制原 Rule/受约束序列，**不使用仅在固定 `2+2+2+2` 等状态训练的旧 Q 表去调动态 `1+1+3`**；原 Q/Rule 自动模式完全不受影响。
  - 现有“RGB 识别颜色，逻辑相机提供位置”的路线保持，不能直接把生成器知道的颜色冒充相机识别成功。识别分类与计划不同则进入明确异常处理，不强行分拣。
  - `ARM_FAULT`、READY超时、规划拒绝/回退结果、视觉异常、工件未生成、重复确认、超过箱容量均应严格阻断或走原有界恢复路径。`PART_PLACED` 不等于 `ARM_READY`；`ZONE_CLEAR` 只释放共用取件区。
- 单独新增可信批次状态回执/结果摘要：`plan_accepted`、`first_color`、逐件请求/识别/入箱/READY、最终每箱计数、异常、真实 CSV/ROS 事件；主链路以真实台账和 5 次物理 READY 共同决定批次完成。默认 8 件演示脚本和原 `q` / `rule` 功能不能改默认行为。
- 保留与另一组员的 `action/color/target` 语音思路兼容的解释层；这是**新增**批次扩展，`docs2.zip` 不是完整 voice_bridge 源码，不得声称原 SenseVoice/Qwen 本地实现已迁移。

## 4. 单元测试与安全回归

先完全脱离 Gazebo 编写和运行：

1. 原中文语音、数字/口语“黄的”解析为精确 `green=3,red=1,yellow=1,blue=0,total=5,first=yellow`；另一种自然语言表达和非法语义拒绝。
2. 模型输出数量不一致、负数、小数、未知颜色、超量、无 first、错误顺序、未知字段、伪造控制字段均拒绝；永不直接执行任意控制动作。
3. 未确认计划时零送料；ready先到、plan后到和plan先到、ready后到，都恰好开始一次，不能丢第一件或双发。
4. 黄色首件视觉与成功/READY闸；剩余颜色实际送料数恰好四件，不多不少。
5. 识别错误、上游API失败、双请求同request_id、断连重发、状态超时、异常关停、ARM_FAULT、重复状态事件、失败后迟到COMPLETED均不得再发料或误释放槽。
6. 第三格绿色对应有效格位和预设锚点；第4件同色拒绝，不能用模3环绕导致碰撞。
7. 真实 result_recorder 5 件 CSV 预期每箱计数及类别，四个箱分布为绿3/红1/黄1/蓝0，所有 target_bin=final_bin，恰好5个不同 part_id，拒绝多件输出。
8. 原本每色2件的 gain1+READY Rule/Q 和 244通过/4既有失败的已知基线：新增功能不得造成新回归，所有历史4失败单列而不伪造G0证据。

先做四包 `colcon build --symlink-install --base-paths src` 和针对性pytest，环境复用 `/home/yu/ros_a1/ros2/.pixi/envs/default`，不要自行升级锁定依赖。历史测试须确保正确 source install/setup.bash 后执行。

## 5. 有界的实际 Gazebo 验证（仅当前步骤通过才执行）

1. 静态检查 green 第三格姿态、碰撞边界与回收/放置策略；如第三格有高风险或无可靠路径，**停止物理验证，先报告，不改几何/阈值冒险**。
2. 用独立世界/独立 ROS_DOMAIN_ID、GZ_PARTITION、HOME、日志/CSV 目录，显式 gain1、1ms、500Hz、v3、`feed_policy:=rule`、原真实视觉与完整物理 READY/FAULT。头次测试保持低干扰与可追溯日志。保留安全监督器超时/停止/进程组清理。
3. 启动后先确认没有生成任何工件；经用户确认计划才执行。首次语音可先由用户真实讲话输入，随后检查 OCR/ASR 输出与大模型结构化JSON，再由人工确认执行。
4. 最多做 1 次完整五件主验收；如需要第三格短诊断，先说明目的并隔离，失败不筛选重跑。先黄色成功且 READY，再依计划逐件；采集 `VOICE_PLAN`, `SPAWN_REQUEST`, `PartInfo`, `PART_PLACED`, `ARM_READY`, `ARM_FAULT`, 全部控制器结果和5行CSV。
5. 验收目标：恰好5个工件；首件 `yellow` 且先完成；绿3/红1/黄1；无多余请求；全部目标箱与实际箱相同、5次物理READY与本臂槽释放匹配；标明是否有抓取重试、控制器异常、接触与规划拒绝，**发生异常保存真实记录**。
6. 出现ARM_FAULT、意外来料、前一黄色尚未完成就多送料、控制器状态不一致、三绿放置异常、严重接触或CSV错判时受控停止；不修改已验收安全阈值、不为漂亮结果反复重跑。
7. 除非关键安全和静态验证需要，不额外重跑已经存在完整证据的旧 8 件 Rule。需要对兼容性出具结论时清楚区分“静态/单元测试通过”和“额外真 Gazebo 实测”。

## 6. 交付和报告

在 `~/yuri_gazebo_performance/voice_batch_aliyun/` 至少生成：

- `ARCHITECTURE_AND_DIFF_CN.md`（与Yuri/组员的关系、为何采用独立语音批次模式）
- `ASR_LLM_MODEL_AUDIT_CN.md`（模型ID、地域、实际调用、开销、错误，不记录密钥）
- `DYNAMIC_BATCH_API_CN.md`（消息、服务、JSON、ROS响应、正常/异常时序）
- `GREEN_THIRD_SLOT_SAFETY_CN.md`（第三格证据、可行性和限制）
- `UNIT_AND_BUILD_RESULTS_CN.md`（构建、全量和新增测试）
- `GAZEBO_VOICE_BATCH_VALIDATION_CN.md`（真实5件结果或不能继续的原因，附原始日志/CSV）
- `DEMO_RUN_CN.md`（独立的语音演示启动命令、如何录音/确认/安全停止/查输出、不要使用旧gain2入口）
- 本地独立git commit及SHA、源码diff、所有源文件与配置哈希，记录路径。可写一个录屏友好的CLI显示识别文字、JSON计划及实时五件进度；不要把云模型接口与ROS执行混成虚假动画。

优先保证可运行、可复现，不要求开发华丽GUI；Windows可后续用OBS录 Ubuntu虚拟机。绝对不把API Key写入共享目录、Git、PR、stdout、错误日志、截图或报告。

**最终汇报只回答：** (1) 中文语音是否真识别，(2) LLM是否真输出5件黄色优先计划，(3) 未说话是否零来料，(4) 三个绿色第三格是否通过真实验证，(5) Gazebo是否按计划5/5正确并完成5 READY，(6) 仍有哪些碰撞/重试/异常，(7) 原Rule/Q/READY是否保持兼容，(8) 如何给用户录像和后续PPT。**先不推GitHub、不合并PR、不启新RL训练、不安装自定义Skills。**
