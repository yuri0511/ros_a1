# Yuri A1：最终成果归档（停止研发）

按用户2026-10-17最终指令：停止课程材料制作、停止四格/语音动态研发，不再实验；上传已有成果供审核。

## 最终状态

- 正式演示基线仍为gain1 + 物理READY + Rule原8件版本。历史一次8/8、四箱各2、148控制器Action成功、8次READY及匹配任务槽释放、0重试/ARM_FAULT。这是一次仿真回归，非长期或实体机器人安全认证。
- Q-learning做过真实Gazebo对照，未证明优于Rule；50000回合为离线模型训练。V2通用Q后续一次功能轮完成8/8，但记录一次MoveIt规划拒绝后重试；不能混作旧配对性能结果。
- 阿里云qwen3-asr-flash ASR和qwen-flash任务解析真实成功。原始“色的”语义曾被本地校验拒绝，修正同义词后用原记录通过；没有伪造重识别。
- 五件语音驱动Gazebo没有完成。四件轮黄色浅接触中止，后续未选择性重跑。
- 四格研发停止于`071536a34f8fe5fe5407cdce930f82173ffe444c`。已加入MoveIt真实持件同步候选及ABC门控，但最终全路径48段仅42段通过：黄色下降终点的全尺寸附着工件与箱底有约2.997mm模型相交。A/B/C均未启动，0/3授权消耗；没有新增PLACED/READY/成功CSV。
- 未放宽安全阈值，未删除箱壁或缩小工件；motion_validated仍false。四格候选不替代已验收基线。
- 本次课程PPT、PDF、6分钟讲稿、正式视频**尚未完成**。用户终止制作前只整理了部分证据、真实截图及源码包；不得当作完整课程成品。

## 归档结构

`BRANCH_ARCHIVE.json`：14个本地成果版本的精确SHA与远端归档分支。它们位于`archive/20261017/`命名空间，独立保留语音、UI、四格及历次模型/性能代码。审核分支的运行源码仍是原gain1+READY八件代码，此次只新增归档文档。

`reports/`：各阶段已有报告的去重副本，保留其历史语境。早期报告中“语音未集成”等表述只适用于当时阶段，当前状态以上述总结为准。各版本时长和结果不得交叉比较。

`EVIDENCE_MANIFEST.json`：逐文件路径、字节数、SHA256、重复引用与附件归属。原始实验数据作为独立GitHub附件归档，不把数百MB数据塞进Git历史。源码在分支中，构建依赖不打包。

密钥、凭据、私人麦克风音频、依赖和缓存不公开。原始本地工作区、日志、旧Q表与历史提交全部保留。没有推送正式分支，没有合并Draft PR。

## 原8件启动（仅命令说明，本次不运行）

```bash
YURI_PIXI_PREFIX=/home/yu/ros_a1/ros2/.pixi/envs/default bash /home/yu/yuri_gazebo_performance/final_delivery_v2/worktree/ros2/research_tools/final_system/run_rule.sh
```

该入口显式gain1、Rule、1ms、500Hz、v3、软件渲染，按独立目录保存结果。Ctrl-C或在该运行目录创建STOP文件进行受控停止。不得以旧gain2日常入口冒充本基线。

同世界GUI客户端已保存在四格归档分支的`ros2/research_tools/four_slot/baseline_gui.py`：从运行metadata读取对应GZ_PARTITION，仅执行`gz sim -g`。本次没有重新验证GUI视频或启动新仿真。

## 版本及来源

上游：https://github.com/yuri0511/ros_a1 ，基线`72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd`。

历史Rule真实运行：`ed76c35839ffd878cb8134e5bdc6de5f6c050364`。既有交付基线：`f6a7afc8ed7451e161023c6039b5f2494ea55ab6`。

实际修改者保持原Git作者，不因认证账号yuri0511而改署名。五名组员完整姓名及个人分工未确认，不能虚构。

本次上传结束后停止。任何后续运行、研发或正式材料制作需新的用户指令。
