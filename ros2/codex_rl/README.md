# codex_rl

这是 `dual-arm-rl` 分支提交 `90c0386722308cafd1ac3b1da8775ce4473245f8` 里的三样东西，原样复制，没有改内容：

- `dual_arm_rl/`：调度模型、Q 表读写
- `config/timing.gazebo-v1.json`
- `artifacts/gazebo-v1/q_table.json`

两份 JSON 的 `overlap_validated` 都是 `false`。加载时要求它们一致，所以不要只改其中一个。`q_table.json` 里的 `deployment_ready` 也是 `false`。

演示默认读这里。切换方式见 `ros2/README_RUN.md` 第 6 节。
