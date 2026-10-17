# Yuri ROS 2 双机械臂 Gazebo 性能专项实测报告

测试日期：2026-10-16（Asia/Shanghai）。所有性能数字均来自这台 Ubuntu 虚拟机；不把目标 RTF 当作实测倍率，不把多个 RTF 相加当作吞吐量。

## 结论

- 已完成完整 8 件验证的单实例最高稳态差分 RTF：**0.970**，配置 `C-formal-target4`，目标 4，llvmpipe 线程 default。其全可用 stats 区间 RTF 为 0.940；5 秒窗口 P10/P90 为 0.751/1.147。
- 单实例最快完整完成时间（含已有 GUI 验收）：**244.0 秒**，来自 `A 原始 GUI 基准`。本轮无 GUI 单实例最快为 **248.2 秒**（`C-formal-target4`）。RTF 最高的方案不一定是完整任务最快的方案。
- 已测可靠配置中最佳并行数量：**2 个**；无 GUI、目标 RTF=4、`LP_NUM_THREADS=2`、Fast DDS、X11/llvmpipe，相机保留。
- 该配置 1 轮共完成 **2 个独立 8 件批次**，累计测量墙钟 380.0 秒，合并实测吞吐量 **18.95 批次/小时**；相比 A 的 14.75，提升 **28.5%（1.28 倍）**。
- 该配置单轮最高观察值为 18.95 批次/小时；把每轮退出清理也计入时，合并为 **18.85 批次/小时**。不包含人工分析/方案切换间隔；没有测试长期循环复位后连续运行一小时。
- 完整验证通过不代表已证明长期零故障。下面列出每轮重试、最终正确率、控制错误及观测范围。所有原始产物保留，未开发/训练强化学习。

按用户要求提前收尾：无节流完整批次已经完成；刚启动的单实例 LP2 正式对照在 18.1 秒时正常停止（0 件）。收尾时发现三实例停止后残留了一个 Gazebo Server（PID 33829，E 的 instance-1），常规 SIGINT/TERM 未退出，已确认其私有 HOME/partition 后精确 KILL。由于没有独立记录其后续 CPU 活动，不能假定它没有干扰：之后目标 2x 的 234.3 秒、无节流的 239.0 秒虽均 8/8 正确，但从干净单实例性能比较中剔除；保留为观察值。参见 data-quality.json / residual-cleanup.json。未再做双实例重复验收，也未执行新增的故障注入检查。结论以已完成数据为限，双实例吞吐量只有一轮完整实测。

## 环境、边界与原工程保护

Ubuntu 24.04.5 LTS；VMware Workstation 26.0.1；12 vCPU、24 GB RAM；Gazebo Harmonic 8.10.0；ROS 2 Jazzy；MoveIt 2.12.4；Pixi 0.82.0。实测渲染为 Mesa 25.2.8 / llvmpipe LLVM 20.1.2。完整版本记录见 `versions.txt`。

原工程 `/home/yu/ros_a1/ros2` 保持验收 Commit `083a51568e20089d59f9c742da60637f2f03045b`。实验位于 `/home/yu/yuri_gazebo_performance/workspace/ros2`，独立本地 clone、独立 build/install/log，复用原有 Pixi 依赖前缀，不重装/升级 ROS，不改变锁文件，不提交或推送。

开始前检测到上一轮 Gazebo/RViz/ROS 仍在运行，已按旧报告机制创建对应运行目录的 `STOP` 文件正常停止；证据为 `baseline-A/exit.json`。旧结果文件和历史日志均保留。最终原始文件复核见 `original-integrity-final.json`。

唯一跟踪源码差异：实验版 `sorting_demo.launch.py` 透传 `gazebo_gui`、`launch_rviz`，并给结果记录器传独立 CSV。世界变体只改 `<real_time_factor>` 为 1/2/4/0；`max_step_size=0.001` 完全不变（`world-audit.json`）。未改规划参数、轨迹、控制器、关节/夹爪、碰撞几何、互锁、RGB/逻辑相机及分拣标准。差异见 `launch-changes.diff`。

## 测量方法与口径

每个正式运行从第一个启动命令的墙钟时刻计时，到最后一个实例第八条结果记录为止。某实例先完成后仍保留其进程，直到整组完成，所以吞吐量包含慢实例拖尾。每个批次必须 8 件全部 PLACED、目标箱等于最终箱、四箱各 2 件才计为成功。

`episodes_per_hour = 3600 × 成功完整批次数 / 整组墙钟秒数`。`comparison.csv` 同时记录含清理的 lifecycle 值。已有 A 只提供启动至完成口径，因此与 A 的提升按相同完成口径计算，未虚构 A 的退出耗时。

每个实例直接订阅 Gazebo Transport `/world/sorting_cell/stats`，原始 `sim_time/real_time/iterations/step_size/real_time_factor` 保存为 `stats.jsonl`。主 RTF 用启动 30 秒后至该实例完成的 `Δsim_time/Δreal_time`；另给全 stats 区间值与连续 5 秒差分窗口 P10/P90。所有正常完整实验的迭代计数与 1 ms 仿真时间推进一致。

资源每 2 秒采样整机/12 个核心 CPU、进程 CPU/RSS、内存、Swap、OOM。CPU 总百分比的 100% 表示全 12 vCPU，进程百分比的 100% 表示一个 vCPU。CPU 稳态统计窗口为启动 30 秒后至整组完成。Gazebo server 内含控制器，因此进程 CPU 不能单独解释为纯物理成本。

前期持续订阅 1000 Hz /clock 和约 500 Hz joints 的探针消耗约半个 vCPU，后已保存为 `probe-high-frequency-pilot.py`，所有正式对比统一换成低干扰探针（约一个核心的 5–6%）。正式运行仅在第 40 秒附近额外采样 10 秒 ROS 时钟/关节/RGB，用于延迟核对，仍存在短暂观测开销。前期 pilot 单列，不与正式数据混算。

已有 A 是先前一次成功 GUI 验收，8/8、0 重试、244.040 秒；其 RTF 0.709/0.776 为两个局部采样均值，不能与本轮连续差分 RTF 视作完全相同测量。配置与顺序测试、重试波动、没有随机化的局限均保留。

## 正式完整任务对比

| 配置 | 目标 / 实例 / LP线程 | 正确完整批次 | 总墙钟 s | 成功批次/h | 含清理批次/h | 各实例稳态 RTF | CPU均值/P95 | 最低可用内存 GiB |
|---|---|---:|---:|---:|---:|---|---|---:|
| A 原 GUI（已有） | 1 / 1 / default | 1/1 | 244.0 | 14.75 | 未测 | 局部 0.709 / 0.776 | 未按本轮口径采样 | 未测 |
| B-formal-headless-1x | 1 / 1 / default | 1/1 | 250.6 | 14.37 | 14.18 | 0.815 | 57.6 / 64.8% | 18.89 |
| C-formal-target4 | 4 / 1 / default | 1/1 | 248.2 | 14.50 | 14.40 | 0.970 | 64.6 / 67.7% | 18.90 |
| D-formal-dual | 4 / 2 / 2 | 2/2 | 380.0 | 18.95 | 18.85 | 0.679, 0.682 | 78.7 / 81.2% | 17.81 |
| E-formal-triple | 4 / 3 / 2 | 0/3 | 381.8 | 0.00 | 0.00 | 0.435, 0.440, 0.436 | 89.0 / 91.1% | 16.65 |
| C-formal-target2 † | 2 / 1 / default | 1/1 | 234.3 | 15.37 | 15.26 | 0.919 | 64.4 / 66.9% | 18.36 |
| C-formal-unthrottled † | 0 / 1 / default | 1/1 | 239.0 | 15.06 | 14.81 | 0.925 | 65.3 / 68.0% | 18.33 |
| C-formal-target4-LP2 † | 4 / 1 / 2 | 0/1 | 18.1 | 0.00 | 0.00 | — | — / —% | 18.20 |

†：存在残留进程干扰疑点，不参与最快可靠配置选择。因保护机制中止/未完成的行只描述观测窗口，不能作为有效吞吐量配置。`comparison.csv`、`instances.csv` 提供未舍入数据。

| 配置/实例 | 8件结果 | 墙钟 s | 全stats RTF | 稳态5s P10/P90 | 自动重试 | 视觉错误 | 控制器执行错误 | 互锁/串线异常 |
|---|---|---:|---:|---|---:|---:|---:|---:|
| B-formal-headless-1x / 1 | 8/8正确 | 250.6 | 0.808 | 0.706 / 0.871 | 1 | 0 | 0 | 0 |
| C-formal-target4 / 1 | 8/8正确 | 248.2 | 0.940 | 0.751 / 1.147 | 4 | 0 | 0 | 0 |
| D-formal-dual / 1 | 8/8正确 | 380.0 | 0.672 | 0.516 / 0.795 | 7 | 0 | 0 | 0 |
| D-formal-dual / 2 | 8/8正确 | 283.7 | 0.653 | 0.505 / 0.800 | 1 | 0 | 0 | 0 |
| E-formal-triple / 1 | 7件（未完整成功） | — | 0.443 | 0.316 / 0.527 | 1 | 0 | 0 | 0 |
| E-formal-triple / 2 | 7件（未完整成功） | — | 0.441 | 0.299 / 0.534 | 2 | 0 | 0 | 0 |
| E-formal-triple / 3 | 7件（未完整成功） | — | 0.434 | 0.308 / 0.522 | 1 | 0 | 0 | 0 |
| C-formal-target2 / 1 | 8/8正确 | 234.3 | 0.929 | 0.696 / 1.112 | 1 | 0 | 0 | 0 |
| C-formal-unthrottled / 1 | 8/8正确 | 239.0 | 0.898 | 0.715 / 1.138 | 2 | 0 | 0 | 0 |
| C-formal-target4-LP2 / 1 | 0件（未完整成功） | — | 0.697 | — / — | 0 | 0 | 0 | 0 |

“自动重试”是执行器输出的失败尝试计数；一次工件可重试多次。最终 8/8 与首次抓取成功率是不同指标，不能用 8/8 掩盖重试。未完成短测也不应当被当作完整分拣失败样本。控制器统计截断于任务完成，正常 SIGINT 清理产生的 KeyboardInterrupt / rcl_shutdown 重复调用不算任务运行故障。

## 短测、兼容性与测量修正记录

| 运行 | 目标/实例/线程/RMW | 状态 | 观察窗口 s | 稳态 RTF | 说明 |
|---|---|---|---:|---|---|
| B-headless-1x | 1/1/default/rmw_fastrtps_cpp | completed | 256.0 | 0.836 | 高频探针 pilot；完整通过但不混入正式对照 |
| C-target2-screen | 2/1/default/rmw_fastrtps_cpp | timeout | 86.8 | 0.950 | 短时筛选，未计完整吞吐量 |
| C-target4-screen | 4/1/default/rmw_fastrtps_cpp | timeout | 86.8 | 1.119 | 短时筛选，未计完整吞吐量 |
| C-unthrottled-screen | 0/1/default/rmw_fastrtps_cpp | timeout | 86.8 | 0.917 | 短时筛选，未计完整吞吐量 |
| C-unthrottled-LP2-screen | 0/1/2/rmw_fastrtps_cpp | timeout | 86.6 | 0.985 | 短时筛选，未计完整吞吐量 |
| C-cyclone-LP2-screen | 0/1/2/rmw_cyclonedds_cpp | timeout | 86.7 | 0.908 | 短时筛选，未计完整吞吐量 |
| C-cyclone-LP2-EGL-screen | 0/1/2/rmw_cyclonedds_cpp | requested_stop | 66.7 | 1.142 | 拒绝：EGL 实际选择 SVGA3D，RGB 恒色，不能用于性能结论 |
| C-fastdds-large-LP2-screen | 0/1/2/rmw_fastrtps_cpp | timeout | 86.7 | 0.975 | 短时筛选，未计完整吞吐量 |
| C-optimized-full | 4/1/2/rmw_fastrtps_cpp | completed | 248.4 | 0.881 | 高频探针 pilot；完整通过但不混入正式对照 |
| C-fast-single-full | 4/1/default/rmw_fastrtps_cpp | requested_stop | 109.1 | 0.931 | 因探针观测开销修正主动终止，未完成 |
| D-isolation-check | 4/2/2/rmw_fastrtps_cpp | isolation_verified | 57.4 | 0.721, 0.777 | auto_start=false 专用隔离验证，无分拣吞吐量 |

Harmonic 的本机帮助与实现核对后使用 SDF `real_time_factor`。0 代表启动时不节流；没有更改 1 ms 步长，也没有用不支持的 classic `real_time_update_rate` 参数假装生效。Harmonic 的运行时 physics 服务对非正 RTF 有限制，所以 0 使用独立 world 启动。参数语义核对：[Gazebo 8.10.0 实现](https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.10.0/src/SimulationRunner.cc)。目标 2/4 仅提高限速上限；实际推进速度以 stats 为准。

已试 X11/llvmpipe（有效）及 `--headless-rendering` EGL。EGL 在本 VMware 环境中实际使用 SVGA3D，并给出恒定 RGB 像素 (160,170,179)，第一件无法识别，已保存 `rgb-evidence.png`、OGRE 日志和 `compatibility-failure.json`；其看似更高 RTF 无效。没有采用。

共享环境已有 Cyclone DDS，切换短测使 ROS RGB 接收恢复接近 Gazebo 10 Hz/仿真秒，但没有改善 RTF，故未采用。Fast DDS LARGE_DATA 环境短测也未改善图像到达率；只确认请求了该环境选项，不把内部传输是否成功切换当作已证实。未修改/升级中间件版本。

## 主要瓶颈

**A. CPU 物理与控制计算：** 1 ms 步长、真实碰撞和控制均保留。提高目标后实际 RTF 仍远低于 2/4，表明目标限速不再是唯一限制。Gazebo server CPU 包括物理、传感器、控制器和 Transport；不能把全部 server CPU 归因于物理。`physics-render-threads-B.txt` 的线程诊断中主线程约 41% 单核，而 llvmpipe 工作线程合计约 108% 单核，说明单纯将其称为“CPU 物理瓶颈”不准确。

**B. 软件渲染：** 无 GUI 不等于无渲染。RGB 传感器必须保留，OGRE 日志确认正式试验仍用 llvmpipe。去掉 GUI/RViz 节省显示成本，但无法消除相机渲染与桥接成本。限制 LP 工作线程是每进程环境设置，未改相机分辨率/更新率。

**C. 现实时间等待：** `attach_helper.py` 跟随线程以 40 Hz 墙钟周期工作（monotonic / sleep / Event.wait）；调度启动、送料/派发含 threading.Timer，首次送料存在约 20 秒墙钟延迟，launch 也有 8/15 秒 TimerAction。运动结果轮询常用 0.05/0.1 秒墙钟间隔；实际 settle 放件路径有 0.4 秒等待。超时、失败恢复及可选功能也有 sleep，但不是每轮都执行；例如 1.5 秒 drop 等待是条件路径，不能把它当成每次固定成本。所有等待保留。

因此，物理 RTF 上升并不等于整轮按比例缩短。抓取重试与不可直接缩短的墙钟阶段会抵消收益。`wall-clock-waits.txt` 是原源码只读检索，`task-phase-timings.json` 给出取件锁持有与释放到下一次派发的实际墙钟区间，不把该区间全部解释为 sleep。

**D. ROS 通信与执行：** MoveIt 规划日志的计算时间通常约亚毫秒至几毫秒，远小于完整批次，不能视为主要加速空间。控制器动作的墙钟时长受仿真速度与动作本身约束，下面给出完整测量。RGB 端到端存在到达率差异；原生 Gazebo 约 10 Hz/仿真秒，Fast DDS 的 ROS 订阅端较低。独立仅图像探针也观察到差异，不能完全归因于主探针繁忙，也不能仅凭这些数据将损失精确归因于 DDS 或桥接某一个环节。视觉仍按实时图像分类，并与生成颜色日志核对，无预设答案。

| 配置/实例 | 原生RGB / ROS RGB / 逻辑相机 Hz（仿真秒） | 关节 Hz（仿真秒） | 关节年龄P95 s | RGB年龄P95 s | 规划中位/P95 s | 控制动作中位/P95墙钟 s |
|---|---|---:|---:|---:|---|---|
| B-formal-headless-1x / 1 | 10.00 / 3.50 / 10.00 | 500.0 | 0.0010 | 0.0710 | 0.000613 / 0.001360 | 0.745 / 3.554 |
| C-formal-target4 / 1 | 10.00 / 3.51 / 10.00 | 499.1 | 0.0010 | 0.0990 | 0.000610 / 0.002302 | 0.563 / 3.005 |
| D-formal-dual / 1 | 10.00 / 2.29 / 10.00 | 498.4 | 0.0020 | 0.1010 | 0.000694 / 0.002471 | 0.827 / 3.714 |
| D-formal-dual / 2 | 10.00 / 2.39 / 10.01 | 476.1 | 0.0020 | 0.1030 | 0.000666 / 0.002357 | 0.837 / 4.465 |
| E-formal-triple / 1 | 10.00 / 2.22 / 10.00 | 495.4 | 0.0020 | — | 0.000852 / 0.004132 | 1.244 / 6.172 |
| E-formal-triple / 2 | 10.00 / 2.72 / 10.00 | 486.7 | 0.0020 | 0.1030 | 0.000746 / 0.003328 | 1.254 / 6.105 |
| E-formal-triple / 3 | 10.00 / 2.38 / 10.00 | 483.1 | 0.0020 | 0.1060 | 0.000834 / 0.004117 | 1.299 / 6.316 |
| C-formal-target2 / 1 | 10.00 / 3.74 / 10.00 | 499.0 | 0.0010 | 0.0890 | 0.000659 / 0.001636 | 0.606 / 2.922 |
| C-formal-unthrottled / 1 | 10.00 / 3.53 / 10.00 | 498.9 | 0.0010 | 0.0850 | 0.000624 / 0.001667 | 0.577 / 3.439 |
| C-formal-target4-LP2 / 1 | — / — / — | — | — | — | — / — | — / — |

年龄=采样到的 ROS 仿真时钟−消息时间戳，包含传感器周期、桥接和回调调度，时钟观测有离散误差；它不是纯网络单向延迟。关节频率/年龄仅来自约 10 秒窗口，不能外推成整轮绝无延迟峰值。

## 并行隔离与资源上限

所有实例共用同一个依赖前缀和实验 build/install，但启动分别拥有：`ROS_DOMAIN_ID=61/62/63`、唯一 `GZ_PARTITION`、独立 HOME、ROS_LOG_DIR、结果 CSV 和 Gazebo/OGRE 日志目录。ROS discovery 限于 LOCALHOST。相同 `/world/sorting_cell` 名称在不同 Gazebo partition 内，ROS 同名话题/服务在不同 DDS domain 内。分区变量语义参考 [Gazebo Transport 13 环境变量](https://gazebosim.org/api/transport/13/envvars.html)，实际隔离另做了本机服务与时钟验证。没有对原一键脚本删除防重复保护。新增 `run_performance.sh` 自带锁和已有仿真检查。

专用隔离验证 `verified=True`：每个域只看到自己的 ROS/GZ 标记、恰好一个 /clock publisher 和一个 controller_manager，时钟 publisher GID 各不相同。暂停第一实例后其时钟增量 0.000s，第二实例继续 1.432s；第一相机停止、第二相机继续。重置第一实例的仿真时钟归零，第二仍继续；停止第一后第二时钟继续到 34.507s。请求/响应与快照均保存于该轮 `isolation-control.jsonl`、`isolation-proof.json`。

完整双/三实例测试还持续检查 ROS/GZ 标记、clock publisher 数、controller manager 数、原生与 ROS 图像、独立工件 CSV；未观察到跨域标记、重复时钟来源、控制命令串线引起的异常或结果互相追加。没有通过给活动机械臂注入危险错误指令来测试串线。这里的 world reset 证明服务隔离，并不等于已验证运行中重置整个 ROS 分拣状态机；采样新批次推荐完整重新启动。

测试器对各启动 session 发送 SIGINT，只处理本轮实例；不使用全局 pkill。三实例退出时实际发生一个 server 残留，收尾已精准清理。已补上父 launch 退出后仍检查 session 存活进程并强制回收的分支，补丁静态检查通过，按用户收尾要求未再开仿真复测。测试器新增单实例 STOP_INSTANCE / 单实例退出隔离分支，设计为其余继续；本次收尾前未执行该新增接口和故障注入的实测。已实际通过的是上面的暂停、重置及 SIGINT 停止隔离。整机 OOM/内存或持续 Swap/三实例持续 CPU>90% 20 秒触发资源保护时才终止整组。测试期间未启动超过三个完整实例。三实例在 381.8 秒触发该 CPU 保护，未完成批次，不计算有效吞吐量，也不再扩容。

## 运行可靠性与限制

正式已完成的任务轮次（包含有干扰疑点的两轮，正确性仍可核对）合计 6/6 批次满足 8 件全部正确；各轮重试见上表。短测、隔离、主动终止和资源保护中止不混进这个分母。原 GUI 基准没有重试；优化/并行测试中存在自动重试，因此不能承诺首次抓取成功率不受影响，也不能以少量完整批次证明长期可靠性相同。

未进行新增故障注入验证；不能将已实测的停止/重置隔离扩大解释为所有故障恢复路径已经验收。已存在的 MoveIt “Found empty JointState message”、未配置 Octomap 3D sensor plugin 等日志也在原 GUI 基准出现。实际控制器执行故障按上表统计；这些既有日志没有通过修改控制算法去隐藏。所有 shutdown 错误保留在原日志中。没有进行长达小时的稳态压力循环、故障恢复任务重派发、强化学习或 Q 表训练。

## 大量 Gazebo 采样推荐

本机优先采用 **2 个隔离实例、无 GUI/RViz、目标 RTF=4、LP_NUM_THREADS=2、Fast DDS、X11/llvmpipe**，保留当前相机、1 ms 物理、控制器与互锁。目标只作上限，不承诺实际 4x。每轮独立启动、独立 CSV；资源保护继续保留。需要桌面交互或留 CPU 余量时可减少到两个实例。不要采用本机已失败的 EGL 路径。

```bash
# 参数：实例数（1..3） 目标RTF（1/2/4/0）
~/yuri_gazebo_performance/run_performance.sh 2 4
```

默认 LP_NUM_THREADS=2；如复现单实例 default 线程配置，直接按 TEST_COMMANDS/metadata 中的环境执行 tools/run_case.py，不要继承额外的 RMW/EGL/传输选项。脚本复用当前已存在的 Pixi 环境，不下载独立依赖。

```bash
# 正常停止整轮（CASE_DIR 换为日志打印的目录）
touch "$CASE_DIR/STOP"
# 单独停止一个实例；其他实例继续
python3 ~/yuri_gazebo_performance/tools/instance_control.py "$CASE_DIR" 1 stop
# 测试完成后重新汇总
python3 ~/yuri_gazebo_performance/tools/analyze.py
python3 ~/yuri_gazebo_performance/tools/export_results.py
```

本次测得的是固定 Commit、当前 VM 分配和少量对照下的最高有效配置，不是所有线程/渲染/DDS参数组合的全局数学上界。后续大量采样应继续记录批次成功率和重试，将成功批次数除以包括启动、退出、必要复位在内的实际调度墙钟；不要用 RTF 求和估算产出。

## 交付索引

- `REPORT.md`：本报告。`comparison.csv/json`：整轮比较。`instances.csv`：每实例结果。
- `TEST_COMMANDS.md`：从实际 metadata 自动导出的每个测试命令与关键环境；`runs/*/metadata.json` 与 `launch-environment.json` 保存精确启动命令、域、分区、HOME、日志和 CSV 路径。
- `runs/*/instance-*/stats.jsonl`：原始 Gazebo stats；`ros-probe.jsonl`、`latency-window*.json*`：ROS/相机/时钟/关节样本；`launch.log`、`ros/`、`home/.gz/`：ROS、控制器、OGRE/服务日志；`results.csv`：逐件分拣结果。
- `runs/*/system.jsonl`、`processes.csv`、`process-env/`、`per-core-mean.json`：每核/进程/内存/Swap/环境证据。`resource-summary.csv/json` 提供各 PID 稳态 CPU 与 RSS 汇总。
- `baseline-A/`：已验收 GUI 基准的只读副本；`world-audit.json`、`launch-changes.diff`、`original-tracked-files.sha256`、`original-integrity-final.json`：参数与原工程完整性。
- `workspace/ros2`：独立实验工程；`activate.sh`、`run_performance.sh`、`tools/`：可复现启动、隔离、采样、统计脚本。旧实验产物未删除。

生成时间：2026-10-16T13:15:20+08:00
