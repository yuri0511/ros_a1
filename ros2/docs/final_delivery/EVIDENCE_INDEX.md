# 原始证据索引

主审核ZIP与大型证据ZIP均留本地，不提交到Git。源数据、旧Q、失败轮不删除不改写。

| 文件 | 本地路径 | 字节数 | SHA256 |
|---|---|---:|---|
| Yuri_Review_72ab_V2.zip | /home/yu/yuri_gazebo_performance/yuri_review/Yuri_Review_72ab_V2.zip | 8847669 | 51e63ca1f3c59d4fd6ee34ab80369f805878f8a73c42013dc9a657fd68e74f50 |
| Yuri_Evidence_72ab_V2.zip | /home/yu/yuri_gazebo_performance/yuri_review/Yuri_Evidence_72ab_V2.zip | 86983202 | 4083d6b71810b9ef69e842a354778d667c26baa92d4fc1296d11e88e0164ea96 |

校验：在yuri_review执行sha256sum -c ARCHIVES_SHA256.txt及unzip -t。主包MANIFEST_SHA256.txt覆盖全部主包文件；EVIDENCE_FILES_SHA256.txt对应独立证据包各原始条目。ZIP私下分享前仍应按受众复查本机路径等隐私；本次强凭据模式扫描未发现Token/私钥，未上传凭据。

| 来源 | 重要文件 |
|---|---|
| fast_sim_8da998b | REPORT.md、EXPERIMENT_MATRIX.csv、runs/*、audit/*；1/2ms与1/2/3实例 |
| integration_72ab6f2 | INTEGRATION_REPORT、TRAINING_AUDIT、DP_BASELINE_REPORT、GAZEBO_POLICY_COMPARISON、DATA_COMPATIBILITY；runs/*原始八轮 |
| model_calibration_v2 | REAL_TRACE_ANALYSIS、MODEL_MISMATCH_REPORT、CALIBRATION_DATA.csv、MODEL_V2_REPORT、TRAINING_V2_REPORT、DP_V2_REPORT、NEXT_GAZEBO_TEST_PLAN；data/raw/training |
| yuri_review/diagnostic | runs/20261016T171634-diag-rule/instance-1 原始log/guard/JointState/pose/stats/contact/CSV/ROS日志；audit诊断JSON |
| final_delivery | 本次安装/构建/单测/隐私/完整性/身份与上传回执，仅交付无新实验 |

本PR内的小型证据位于ros2/docs/final_delivery/evidence_summaries：guard-fault.json、diagnostic.jsonl、contacts-task-summary.json、仅表头的results.csv、diagnostic-result.json、preflight/guard/runtime-source证明、原日志带源SHA和行号的逐字摘录、diagnostic-only补丁及最终测试摘要。未上传全部原始CSV/ROS日志。

V1小型配置/Q/评估在ros2/research_artifacts/v1；V2校准和评估在ros2/research_artifacts/v2，schema2配置/Q/训练元数据在ros2/codex_rl_v2/config及artifacts/v2。原Yuri表在ros2/codex_rl，保持不变。

需要进一步证据时按来源SHA索引在Ubuntu或独立证据ZIP检索，不用重复训练/补跑替代缺失证据。最新截断Rule诊断与未启动Q不计正式失败率。
