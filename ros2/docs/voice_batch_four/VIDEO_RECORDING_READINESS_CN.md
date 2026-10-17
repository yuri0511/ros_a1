# OBS录制准备状态

目前可展示：四件新文字任务、严格JSON/中文审计、用户确认前零送料的模拟证据、构建/单测和待授权状态。当前不能把这些画面表述为新四件Gazebo或真实语音成功。

只有一次文字物理4/4及必要的真实语音端到端验收完成后，才适合录正式成功演示。绿色旧浅接触可能使本轮诊断暂停；失败轮按原样保留，不择优重跑。

## 独立命令

只检查（无仿真、无API）：

```bash
bash /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2/research_tools/voice_four/run_four.sh --check
```

人工在Ubuntu终端确认后进行一次受控诊断（会启动仿真并运动，当前尚未获准执行）：

```bash
bash /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2/research_tools/voice_four/run_four.sh --interactive
```

入口展示新四件原文、计划、SHA和配置；必须真人TTY输入CONFIRM。请求ID只能消费一次，不能作为反复演示的自动重跑脚本。Agent也可在聊天获得本轮明确批准后创建绑定同一SHA和commit的单次批准记录；没有记录会提前拒绝。

停止（不杀其他进程）：

```bash
python3 /home/yu/yuri_gazebo_performance/voice_batch_four/worktree/ros2/research_tools/voice_four/stop_four.py
```

默认结果在voice_batch_four/runs/唯一目录；latest-run.json只保存本轮路径。保留summary.json、metadata.json、approved-task.json、submission-receipt.json、launch.log、events.jsonl、raw-ros.jsonl.gz、stats.jsonl、contacts.csv、results.csv及停止日志。当前没有runs目录表示尚未运行。

## 后续画面

建议Ubuntu终端显示新录音/ASR原文、Qwen JSON、本地校验和人工CONFIRM；Gazebo双臂/四色箱全景；批次状态及CSV/READY计数。当前验证配置是headless；如另开GUI录屏应明确显示负载变化，不用该耗时与旧低干扰性能数字比较。不要启动第二套执行器或改用旧gain2脚本。倍率加速须标注，屏幕不显示密钥。

黄色优先画面应显示黄色放置确认及ARM_READY先于红色送料；两绿使用原第一/第二格。回收后按累计CSV展示4件，不能据最终画面块数误报。未来麦克风口令应说“开始分拣，一个黄色、一个红色、两个绿色，先完成黄色。”
