# 历史复现工具（默认不执行）

8da/72ab目录为本机已用过的实验驱动原文，包含硬编码/home/yu路径，按历史上下文保存。它们可能写报告、训练Q或启动Gazebo，不能当作安全的通用CLI自动执行；新机器必须改输出ROOT并先做路径审计，禁止覆盖历史实验。V2对应工具在codex_rl_v2/experiment_tools。当前阶段仅交付，不运行任何这些实验入口。

首次完整复现应先审阅docs/final_delivery/FINAL_WORK_HANDOFF_CN.md。diagnostic-only补丁在docs/final_delivery/evidence_summaries/，仅供停轮机制审查，未作为正式状态机修复启用。正式默认feed_policy和原Q表保持72ab，实验显式启用独立参数。
