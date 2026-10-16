# 最终 Rule 演示入口（gain1＋物理 READY）

在仓库 ros2 目录，用既有 Jazzy Pixi 环境构建，无需重新安装：

```bash
# 若复用本机已有环境：
export YURI_PIXI_PREFIX=/home/yu/ros_a1/ros2/.pixi/envs/default
# 在其他机器，设为该机器已配置的 Pixi prefix，或使用本工作树 .pixi/envs/default。
export CONDA_PREFIX="$YURI_PIXI_PREFIX" PATH="$YURI_PIXI_PREFIX/bin:$PATH"
export CMAKE_PREFIX_PATH="$YURI_PIXI_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
for activation in "$YURI_PIXI_PREFIX"/etc/conda/activate.d/*.sh; do source "$activation"; done
colcon build --symlink-install --base-paths src
bash research_tools/final_system/run_rule.sh
```

推荐本机完整激活/构建命令见 docs/final_delivery_v2/FINAL_WORK_HANDOFF_CN.md。脚本本身激活现有 prefix 中全部 activate.d/*.sh，不安装或更改依赖。明确选择 gain1、Rule、1ms、500Hz、v3、无 GUI/RViz、X11 llvmpipe、LP_NUM_THREADS=2。RGB/逻辑相机保留。原 ur5e_cell_controllers.yaml（gain2）和原 Q 表不变。

每轮创建唯一日志/CSV/HOME/ROS_LOG_DIR/GZ_PARTITION；ROS_DOMAIN_ID=82，拒绝已有 Gazebo。默认 420 秒墙钟上限，仅收到八次 ARM_READY 对应槽释放及八条 PLACED 后结束。Ctrl-C，或在打印的运行目录创建 STOP 文件，均只停止本轮进程组；异常先尝试暂停该分区世界，再 SIGINT，超时有界清理。不允许通过脚本覆盖增益、READY 阈值或策略。

READY 修复会改变返回失败后的旧行为：放置结果保留，但未确认物理到位/静止时锁存 ARM_FAULT，不释放执行槽。旧代码消费者必须与 TaskStatus 契约和 dispatcher/sequencer/executor/recorder 整体升级。

此便携入口不包含本机绝对路径的只读 native 观测插件。已验收八件运行的 ROS 运行源码与这里一致；八件运行使用额外诊断世界进行独立复核，因此入口包装和去仪器化世界没有被冒称为同一条已执行命令。原 SDF/几何/物理参数未改变；诊断世界只是附加只读观测插件。

当前证据为一次 Rule 8/8 回归，不能当作长期安全认证。不要用本次高频诊断耗时比较历史低干扰性能。
