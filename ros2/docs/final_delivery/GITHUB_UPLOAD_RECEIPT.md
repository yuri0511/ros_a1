# GitHub上传回执

- 官方CLI登录账号：yuri0511，viewerPermission=ADMIN；Yuri本人网页登录完成。
- 分支：https://github.com/yuri0511/ros_a1/tree/review/yu-gazebo-performance-rl-v2。
- 首个代码交付提交：f032aa23febcac7810085d131d2c9420f4a9c00e。
- Draft PR：https://github.com/yuri0511/ros_a1/pull/1，base ros2-dual-parallel，head review/yu-gazebo-performance-rl-v2，不合并。
- 实际提交作者：Ping <311167715+Pingggg567@users.noreply.github.com>。
- 本文档随后的交接提交会使HEAD前进；此处记录代码来源SHA。最终HEAD另由本地回执/API核验，文件自身不预写无法自洽的自身提交哈希。

上传：性能Python/launch可选参数、独立1ms世界、V2模型/训练/DP、V1/V2小型配置/Q/评估、校准来源、必要测试、中文报告与原日志带SHA/行号摘录。代码提交审计131文件约2.16MB；文档更新另计。

未上传：两个ZIP（主8.44MiB/大型82.95MiB）、完整原生接触CSV/ROS日志、Pixi、build/install/log/cache、认证凭据。它们留Ubuntu，见EVIDENCE_INDEX.md。

本次四包构建成功；157通过/4既有失败，165需求只读模型校验通过。新Gazebo0，新训练0，16配对未执行。最新截断Rule诊断不计正式失败率，Q轮未运行，READY缺口需Yuri优先审核。原始工程和旧数据核验完好；无残留仿真进程。
