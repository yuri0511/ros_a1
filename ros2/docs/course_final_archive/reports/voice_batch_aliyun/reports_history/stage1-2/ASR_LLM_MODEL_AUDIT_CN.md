# 百炼接口与凭据审计（尚未真实调用）

官方文档核对时间2026-10-17：
- https://help.aliyun.com/zh/model-studio/qwen-asr-api-reference
- https://help.aliyun.com/zh/model-studio/non-realtime-speech-recognition-user-guide
- https://help.aliyun.com/zh/model-studio/qwen-api-via-openai-chat-completions
- https://help.aliyun.com/en/model-studio/qwen-structured-output

规格所给docs.bailian地址读取失败，改读同产品官方help.aliyun页面。当前官方推荐北京业务空间域名https://{WorkspaceId}.cn-beijing.maas.aliyuncs.com/compatible-mode/v1/chat/completions，同时明确现有dashscope.aliyuncs.com仍可正常使用；默认使用该北京旧域名，可由人工填入官方北京workspace地址。域名白名单、拒绝重定向，不能把Bearer凭据转发到其他主机。

ASR模型严格qwen3-asr-flash，OpenAI兼容REST，content.input_audio.data为data:audio/wav;base64,...，stream=false、asr_options.language=zh、enable_itn=false；返回choices[0].message.content，非空/完整stop才接受。PCM16、16k、单声道、客户端限制1–15秒/2MiB，录音CLI限制5–10秒。音频只保存在本机专属运行目录，原文件不归一化覆盖。

LLM严格qwen-flash，enable_thinking=false，response_format=json_object，提示包含JSON，temperature=0、max_tokens=400。JSON mode不保证业务schema，因此本地关闭式白名单、数量/库存/优先顺序/原话语义交叉核验仍必需。没有DeepSeek调用、没有下载本地模型。

同一个私有Key用于两模型。人工getpass输入必须关闭回显，否则拒绝。目录~/.config/yuri_a1_voice=0700，api_key/budget/usage=0600，不写源码、共享目录、命令参数或日志。AI不读取或打印该文件内容。错误仅记录HTTP状态及受限的usage/请求ID，不记录Authorization、服务器原始错误或音频Base64。

初次拟申请最多4次付费请求：最多2次文本解析+1次<=10秒ASR+1次识别文本解析，无自动重试，每次失败亦消耗预算，24小时有效。金额尚无账单证据，不承诺免费或编造价格。当前真实请求数=0，花费未发生本工具调用。等用户本机导入及明确授权后继续。
