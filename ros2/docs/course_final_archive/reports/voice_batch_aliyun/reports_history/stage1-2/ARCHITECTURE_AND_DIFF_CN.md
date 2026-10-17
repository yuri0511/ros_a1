# 阿里云语音动态批次：实施进度（阶段1–2）

本次基于f6a7afc8ed7451e161023c6039b5f2494ea55ab6，新分支feature/aliyun-voice-batch-20261017，新工作树voice_batch_aliyun/worktree。已确认远端ref未更新，原final_delivery_v2工作树干净；正式72ab不动，无推送/合并。

已完成独立的云候选计划严格校验、中文显式数量/优先语义交叉核对、北京百炼客户端、不回显私钥导入工具及有界4次付费调用门控、录音/文本/ASR CLI。46项离线测试通过，全部云HTTP均为注入的模拟返回，没有真实付费调用，也不把模拟当识别成功。

发现auto_start=false仍会由_start_ready_feed触发首件；ZONE_CLEAR、终态及Timer也可请求送料。因此后续voice_batch_mode默认false的实现必须整体门控，不能只改auto_start。当前尚未修改dispatcher/launch/SortTask/TaskStatus/控制器/运动代码。

后续阶段3待阶段2人工导入密钥、授权及少量真实文本/语音验证后继续：新增独立SubmitBatchPlan服务、可信request_id/单次受理/确认、逐件强串行（首黄PLACED+物理READY后再发下一件）、真实PartInfo对账、动态5行CSV/5READY共同完成、同箱旧件回收确认、fault/超时有界阻断。不能使用旧Q调动态需求。

组员docs2.zip现已可读取，但仍是报告/图片/视频素材，不是已验证完整voice_bridge代码。未迁移SenseVoice或单臂控制器。旧voice_contract与18项模拟测试原样保留，新模块不覆盖它。
