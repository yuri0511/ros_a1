# 双臂分拣

Gazebo 里两台 UR5e 给蓝、黄、红、绿方块分拣。一次只推一件。北侧 arm1 负责蓝和黄，南侧 arm2 负责红和绿。取件区同一时间只允许一台进入。臂回到待命后，要先满足物理 READY 才算空闲；异常走 ARM_FAULT，不会把还没停稳的臂当成可以接下一件。

8 件演示用 gain1 控制器：`ros2/src/a1_cell_sim/config/ur5e_cell_controllers_stability_gain1.yaml`，位置比例增益 1.0，控制频率 500 Hz。

## 编译

在 Ubuntu 24.04 上进入 `ros2/`。不需要用 apt 装 ROS。

```bash
curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=v0.82.0 bash
pixi install --locked
pixi run bash -c 'colcon build --symlink-install'
```

不要给这条命令加 `set -u`。`build/`、`install/`、`log/`、`.pixi/` 不要提交。

## 用 gain1 和规则跑 8 件

仍在 `ros2/`。没有 NVIDIA 显卡时先执行下面四行。

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY
pixi run bash -c 'source install/setup.bash && ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false controllers_file:=ur5e_cell_controllers_stability_gain1.yaml feed_policy:=rule demo_parts:=8'
```

`feed_policy:=rule` 是谁闲推谁。启动日志里会有 `退回谁闲推谁`。Gazebo 和 RViz 默认不打开窗口。结束时看最后一行 `本轮分拣结束`，四种颜色各 2、失败 0 才算这轮放对。

## 切换到 Q 表

把上面的 `feed_policy:=rule` 改成 `feed_policy:=q`。不用改参数文件。相对路径会解析到：

- `ros2/codex_rl_v2/config/timing.v2.json`
- `ros2/codex_rl_v2/artifacts/v2/q-v2-general.json`

加载成功时日志是 `已加载 Q 表`。两份 JSON 里的 `overlap_validated` 都是 false，不要只改其中一个。`deployment_ready` 仍是 false。固定批次各色 2 件的另一张表是 `q-v2-demo.json`。

## 主要结果

离线模型里，Q 比规则快 3.3%。Gazebo 里 3 组配对：规则平均 184.6 秒，Q 平均 188.0 秒，两边都是 24/24 放对。模型上的这点优势，没有在这三组仿真里变成更快。
