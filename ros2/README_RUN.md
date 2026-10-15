# 在一台新的 Ubuntu 24.04 上跑 8 件「谁闲推谁」

这份说明对应仓库分支 `ros2-dual-parallel` 里的 `ros2/` 目录。按下面的顺序做，可以从一台还没装 ROS 的电脑，启动双臂 Gazebo 演示。

演示的行为是：一轮 8 件，一次只推一件。哪台臂空闲，就推这台臂负责的颜色。北侧 arm1 负责蓝、黄，南侧 arm2 负责红、绿。一台正在取件区时，另一台不能进入；另一台可以在回家或放箱子的路上同时动。规划仍走同一个 MoveIt，执行走各自的关节控制器。Q 表没有接进这个演示。

这个分支是从 `main` 分出来的。仓库根目录的 `src/` 是 ROS 1 Noetic 单臂代码。Ubuntu 24.04 上只编译、只启动 `ros2/` 这一份。

## 电脑要满足的条件

- Ubuntu 24.04，64 位（x86_64），并且能打开图形桌面。Gazebo 和 RViz 都要窗口。请在桌面里开终端。没有画面的远程连接启动不了这个窗口。
- 磁盘至少留出 10 GB。这一套 Pixi 环境在已经跑通过的那台电脑上大约 6.6 GB，编译和下载还要额外空间。
- 内存 8 GB 可以跑完。软件渲染时会比较吃内存，演示过程中尽量不要再开别的大程序。
- 不需要管理员权限，也不要用 apt 安装 ROS。ROS 2 Jazzy 由下面的 Pixi 装进当前用户目录。

没有 NVIDIA 显卡时，按第 5 节的软件渲染命令启动。这是已经跑通的启动方式。

## 1. 安装 Pixi 0.82.0

在终端执行：

```bash
curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=v0.82.0 bash
source ~/.bashrc
pixi --version
```

最后一行应显示 `pixi 0.82.0`。安装脚本会把 `~/.pixi/bin` 写进 `~/.bashrc`。如果新开的终端里找不到 `pixi`，再执行一次 `source ~/.bashrc`。

## 2. 克隆这个分支

已经配置好 GitHub SSH 密钥时：

```bash
git clone -b ros2-dual-parallel --single-branch git@github.com:yuri0511/ros_a1.git
cd ros_a1/ros2
```

用 HTTPS 时：

```bash
git clone -b ros2-dual-parallel --single-branch https://github.com/yuri0511/ros_a1.git
cd ros_a1/ros2
```

后面的命令都在 `ros_a1/ros2` 这个目录里执行。这里有 `pixi.toml`、`pixi.lock` 和四个功能包：`a1_interfaces`、`a1_cell_sim`、`a1_sorting_core`、`a1_vision`。

## 3. 安装 Jazzy 和仿真依赖

```bash
pixi install --locked
```

这一步按 `pixi.lock` 里的版本下载 RoboStack Jazzy、MoveIt、UR 描述、Gazebo 和控制器。第一次会下载好几 GB，需要等待。环境装在当前目录的 `.pixi/` 里。`.pixi/`、`build/`、`install/`、`log/` 都不要提交。

`pixi.toml` 里把 `setuptools` 限制在 80 以下。80 及以上的 setuptools 去掉了这套 Python 包要用的可编辑安装。不要删掉这个限制。

`--locked` 会在锁文件和 `pixi.toml` 不一致时停下来。出现这种情况就停，不要改成不带 `--locked` 的安装，否则版本会和这次跑通的环境不一样。

## 4. 编译

```bash
pixi run bash -c 'colcon build --symlink-install'
```

编译成功后，当前目录下会出现 `install/`。这里的命令不要加 `set -u`。`install/setup.bash` 会读取可能还没赋值的 `COLCON_TRACE`，开着 `set -u` 时会在启动前直接退出。

四个包都显示 `Finished` 就可以进入下一步。

## 5. 启动 8 件演示

仍在 `ros_a1/ros2` 目录：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY
pixi run bash -c 'source install/setup.bash && ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false'
```

这几行环境变量让 Gazebo 用软件渲染。`gpu_offload:=false` 关掉启动文件里默认的 NVIDIA 独显切换。没有 NVIDIA 驱动时必须这样启动。

默认就是双臂、流水线、8 件，不用再加别的参数。`sorting_params_pipeline.yaml` 里的 `demand_parts: 3` 会被启动参数 `demo_parts`（默认 8）盖掉。

启动之后：

- 会弹出 Gazebo 和 RViz。开头几分钟窗口可能一动不动，这是软件渲染还在加载。不要关 Gazebo 窗口。关掉窗口会把物理引擎一起停掉，这轮就废了。
- 方块一次出现一个，从传送带送到取件位置。空闲的臂决定下一件颜色：两台都空闲时，先给 arm1 蓝色；之后优先给刚才没喂到的那台。同一台臂里，件数少的颜色优先，蓝先于黄，红先于绿。
- 一台去取时，另一台可以回家或去放，但不会同时伸进取件区。取件锁在竖直抬高 0.10 米之后才放开，不是在刚夹起来的时候放开。
- 在已经跑通的 8 GB 虚拟机上，从第一件开始取到第八件放完，墙钟大约 17 分钟和 25 分钟各有一次。新电脑的快慢会不同。请等到第八件放完。

结束时，启动这轮的终端里会有 `result_recorder` 的日志，形状类似：

```text
== 本轮分拣结束: bin_blue=2 bin_yellow=2 bin_red=2 bin_green=2 失败=0 放错=0 ==
```

记录器可能从第 3 件起就打印「本轮分拣结束」。以 8 件都放完之后的最后一行为准。四种颜色各 2、失败 0、放错 0，就是这轮放对了。

停仿真：在启动它的那个终端按 Ctrl+C，然后等进程退出。不要靠关掉 Gazebo 窗口来停。

## 这份代码里已经包含的改动

相对组长原来的串行演示，`ros2/src` 里有这些运行差异：

- 推板回位等推板真正回到起点附近（世界坐标约 0.14 米以内），再用仿真时间 12 秒做兜底。流水线路径不再用墙钟 2.6 秒当作推板已经回家。
- 两台正在使用的关节轨迹控制器 `goal_time` 是 2.0 秒仿真时间。轨迹计划走完后再等这么久，还没进容差就结束，避免执行槽一直被占住。默认控制器文件是 `ur5e_cell_controllers.yaml`。`ur5e_cell_controllers_gain10.yaml` 不是这轮演示用的文件。
- 夹起之后先在取件点正上方竖直再抬 0.10 米，然后转向对应箱格。
- 每台臂各有一个执行槽。取件区另外有一把锁：派发时锁上，执行器发出 `ZONE_CLEAR`（正常原因是这次竖直抬高）后才允许另一台进入。
- 来料规则是「谁闲推谁」，一次一件。

手臂和夹爪上有只读接触传感器，碰撞体的尺寸和位置没有改。演示命令不依赖接触记录也能跑。

`ros2/DUAL_ARM_RL_PLAN.md` 是更早的方案草稿。当前启动命令不会一次放三件到带子上，也不会读 Q 表。

语音节点不在这条启动命令里。它如果要用密钥，只从环境变量 `VOICE_LLM_API_KEY` 读取。不要把密钥写进仓库。
