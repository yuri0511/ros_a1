# GitHub 最终上传回执

核验时间：2026-10-17T03:50:58.041440+08:00。

- GitHub认证账号：yuri0511；viewerPermission=ADMIN。凭据由gh本地机制使用，未请求/读取/记录密码、设备验证码或Token。首次Git未配置凭据helper导致推送失败，改用本次命令级gh auth git-credential后正常成功；未修改全局凭据设置。
- 代码实际作者：Ping <311167715+Pingggg567@users.noreply.github.com>（AI辅助）。
- 正式base：ros2-dual-parallel，72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd，未修改。
- 审核分支：https://github.com/yuri0511/ros_a1/tree/review/yu-gazebo-performance-rl-v2 。
- 已实际推送的整合/报告提交：f48b398a25640cf2d12e5e237a1035f2a570d574。
- 物理Q诊断执行提交：39e6d64f9856b0f92f76ff934f1d9011eaa23ed6；物理Rule执行ed76c35839ffd878cb8134e5bdc6de5f6c050364；705仅增加报告/离线分析，运行核心一致。
- Draft PR：https://github.com/yuri0511/ros_a1/pull/1 。只更新指定review分支，无force，不推main/正式分支/dual-arm-rl，不合并。
- 本回执将以文档提交继续更新同审核分支；它不伪造包含自身的提交哈希。最终远端HEAD由本地同名回执最后一段和GitHub API给出。

已上传：整体READY协议/测试、gain1独立配置、明确Rule便携监督入口、纯JSON语音契约、中文课程/交接材料、小CSV/关键日志与哈希索引；前次审核的性能/V2源码和表继续保留。

未上传：本机全部原始high-rate/native/contact大流、旧82.95MiB证据ZIP、Pixi依赖、build/install/log、.so、视频。没有制作不存在的语音集成或GUI录像。原始项目和control_stability历史清单校验通过，旧Q/gain2不变。

真实结果：Rule复用既有8/8零重试；唯一Q新诊断8/8、8次物理READY、149底层Action成功，但一次MoveIt预测碰撞拒绝并恢复重试，另有小段腕/夹爪基座接触工件。Q不是零异常验收，不证明提速。详见RULE_Q_VALIDATION_CN.md。

## 最终远端核验

- 最终审核分支HEAD：`f6a7afc8ed7451e161023c6039b5f2494ea55ab6`。
- PR#1：OPEN、isDraft=true，base仍72ab6f2，未合并。
- 本机工作树干净；原始项目校验及control_stability历史清单均通过；无残留仿真/MoveIt/控制器/诊断采样进程。
- Git内同名回执记录首次上传代码/报告SHA，之后仅增加回执和交接引用；本地此段提供最终文档提交SHA，避免文件自引用哈希问题。
