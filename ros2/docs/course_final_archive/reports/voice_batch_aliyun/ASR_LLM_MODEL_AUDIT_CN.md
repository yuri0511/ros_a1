# 真实ASR/LLM及密钥审计

## 官方接口核对

2026-10-17核对阿里云官方：
- https://help.aliyun.com/zh/model-studio/qwen-asr-api-reference
- https://help.aliyun.com/zh/model-studio/non-realtime-speech-recognition-user-guide
- https://help.aliyun.com/zh/model-studio/qwen-api-via-openai-chat-completions
- https://help.aliyun.com/en/model-studio/qwen-structured-output

规格给出的docs.bailian地址读取失败，改用同产品官方help.aliyun。官方推荐北京workspace专属域名，也说明dashscope.aliyuncs.com现有域名仍可使用。本机真实使用北京https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions；白名单不含新加坡/其他模型提供方，拒绝重定向转发凭据。

qwen3-asr-flash：16k单声道PCM16 WAV的data:audio/wav;base64输入，stream=false，asr_options.language=zh/enable_itn=false，读取choices[0].message.content。qwen-flash：enable_thinking=false、response_format=json_object、temperature=0、max_tokens=400。结构化JSON仍由本地业务schema及原话语义严格审查。

## 本轮真实调用

1. qwen-flash纯文本原句：336 tokens（prompt246/output90），生成正确5件计划。
2. qwen3-asr-flash真实Ubuntu麦克风8秒：usage229 tokens、seconds=8，原样输出“开始分拣，三个绿色，一个红色，一个黄色的，先分拣黄色。”
3. qwen-flash解析上述真实识别文本：328 tokens（prompt246/output82），输出blue0/yellow1/red1/green3，顺序yellow/red/green/green/green。

共3次，未使用4次授权中的最后一次，无API自动重试；金额未取得账单，不编造或声称免费。LLM两次合计664 tokens；ASR按自身计费规则及8秒记录另列，不能把两类usage混作统一费用证明。具体request_id/usage见cloud/text-primary/llm-text.json、audio/voice-primary/{asr.json,llm-parse-asr.json}。

第一次本地交叉核验拒绝了ASR中的“黄色的”，因为初版只认识“黄色/黄的”。增加语法同义形式“色的”后，用已保存真实响应重新校验；没有改写音频/识别文本/候选数量、没有增加API请求、没有强制顺序。初始validated=false及拒绝原因原样保留，新复核记录audit/real-speech-validated.json。

## 录音

arecord default、8秒、128000采样、16k/mono/S16_LE；原WAV SHA256=4ee995cf59df5898db89eb27edb60e91df81e9fc199c7ebbfbf38391be2ffa02。峰值32768、RMS约2056.42，未归一化或覆盖原录音；不能把峰值满量程说成理想无削波录音。语音最终真实识别与候选匹配，但并非鲁棒语音识别准确率统计。

## 凭据

人工Ubuntu本地getpass、不回显；不能关闭回显则拒绝。~/.config/yuri_a1_voice目录0700、Key/budget/usage文件0600，同一个Key用于两模型。AI未查看/展示凭据内容、前缀、长度或哈希；仅本机客户端在认证时按权限读取，没有复制到Git/共享目录/日志。人工明确YES授权最多4次（24小时有效）；status仅返回已加载/预算/消耗次数。初次交互失误后简化为自动北京地址，不影响模型和安全边界。

本轮没有DeepSeek、本地模型权重安装或真实机器人控制调用。API授权不等于Gazebo任务确认，主计划尚未被用户确认执行。
