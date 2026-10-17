# 独立本地审阅分支

基线72ab6f2；本地review/local-72ab-v2。只含代码、Q模型产物与审阅文档，不含运行日志/ZIP/Pixi/build/install。未发布Fork/PR，未推送。diagnostic-only安全停轮补丁单独打包，不作为正式状态机修复提交。

本分支全差异：yuri_review/72ab-to-review-code.patch。实测原始数据与哈希见独立审核/证据ZIP。某些历史复现脚本包含本机绝对路径，迁移须调整。

Q通用表路径为ros2/codex_rl_v2/artifacts/v2/q-v2-general.json。按相同schema2包与config加载；deployment_ready仍false，Rule/Q最小诊断因Rule READY静止不一致提前中止，Q未运行。
