# Ubuntu 语音交互窗口

采用现有Pixi PyQt5，无新依赖安装。源码 research_tools/voice_ui/app.py，原8件入口及抓放代码未变。默认模式固定为“模拟数据 / DRY-RUN · 云调用关闭 · ROS任务提交关闭”，没有ROS提交客户端或真实执行切换开关。

界面包含真实麦克风开始/结束、最长30秒时长与RMS音量、保存WAV后本地播放；识别文字/来源/连接状态；四色数量、总数及黄色放置+ARM_READY优先；二次确认默认选否；机器人状态无来源时全部显示未知，不虚构READY、完成或失败。窗口可滚动，适合Ubuntu小分辨率与Gazebo并排摆放。

```bash
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_voice_ui.sh
```

操作：点“开始录音”，说口令，点“结束录音”。本轮仅保存本地WAV，不识别、不付费、不送料。点“播放本地录音”检查内容。近乎静音会明确报警。录音保存于 voice_batch_four/ui_audio/<随机ID>/microphone.wav，16kHz/单声道/S16，目录0700、WAV0600，附SHA256/RMS/秒数；不写共享目录。

QtTest真实鼠标事件已操作录音/停止按钮，默认ALSA设备实际输出WAV；测试环境无人说话，RMS近零，因此只证明设备捕获与文件链路，不能宣称已识别新四件口令。所有试录各自保留，不覆盖原8秒语音。实际人工口令清晰度待用户本地播放核对。

“加载四件文字测试数据”：独立黄1红1绿2蓝0，来源新文字fixture，不标成Qwen新输出。“查看历史真实三绿五件”：读取原asr.json及llm-parse-asr.json，保留原始措辞及green=3，只供查阅，不能确认；历史LLM原候选是JSON字符串，已修正显示端解析兼容，未改历史文件。

“确认模拟任务”弹出二次确认，否不改变状态，是只设置模拟已提交，禁止重复确认；不会自动推进虚假的执行中/已完成。真实执行中、已完成、执行失败目前均未观测。查看状态明确提示无ROS连接。

未来接口 cloud_port.py 的 PreviewPipeline 分离 transcribe_wav → parse_text → 本地validate_candidate/audit_utterance → 未确认预览。默认DisabledCloud永远拒绝请求，UI不加载密钥。未来经新付费授权后才能注入现有BailianClient适配器并增加工作线程、超时和用量展示；仍不得自动确认或提交。当前没有把该端口接上按钮或网络，不能宣称实时ASR/LLM UI集成完成。

现有云端仅qwen3-asr-flash/qwen-flash，继续使用原私有Key路径；本轮未读取密钥、没有调用API。未来ROS接入必须绑定会话/request_id/计划哈希、硬件READY、合法计数、原互锁和用户确认，并保留未知提交结果不得重发机制。语音预览本身不获得运动权限。

7项相关测试通过（录音UI、来源隔离、二次确认、历史拒绝、默认云拒绝、模拟云预览和停止异常路径；pytest实际函数数7），日志 audit/ui-stop-tests-final.log。早期JSON字符串显示失败原日志保留，修复后重测通过。Python compileall通过；本轮无ROS消息/包编译变更，未重新构建整套机器人或重跑历史343项。旧4项已知测试缺陷没有被宣称修复。
