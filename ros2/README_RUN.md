# 在一台新的 Ubuntu 24.04 上跑 8 件双臂演示

这份说明对应仓库分支 `ros2-dual-parallel` 里的 `ros2/` 目录。按下面的顺序做，可以从一台还没装 ROS 的电脑，启动双臂 Gazebo 演示。

演示的行为是：一轮 8 件，一次只推一件。北侧 arm1 负责蓝、黄，南侧 arm2 负责红、绿。一台正在取件区时，另一台不能进入；另一台可以在回家或放箱子的路上同时动。规划仍走同一个 MoveIt，执行走各自的关节控制器。

下一件推什么颜色有两种算法，用同一条启动命令，靠参数文件切换。仓库里现在写的是这台已经跑过的电脑上的 Q 表路径。别的电脑上没有那两个 JSON 时，启动会退回「谁闲推谁」。怎么切换见第 6 节。Q 表本身不在这个分支里。

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
- 方块一次出现一个，从传送带送到取件位置。下一件的颜色由第 6 节的算法决定。路径是空的、或者 Q 表没加载上时，用「谁闲推谁」：两台都空闲时，先给 arm1 蓝色；之后优先给刚才没喂到的那台。同一台臂里，件数少的颜色优先，蓝先于黄，红先于绿。
- 一台去取时，另一台可以回家或去放，但不会同时伸进取件区。取件锁在竖直抬高 0.10 米之后才放开，不是在刚夹起来的时候放开。
- 在已经跑通的 8 GB 虚拟机上，从第一件开始取到第八件放完，墙钟大约 17 分钟和 25 分钟各有一次。新电脑的快慢会不同。请等到第八件放完。

结束时，启动这轮的终端里会有 `result_recorder` 的日志，形状类似：

```text
== 本轮分拣结束: bin_blue=2 bin_yellow=2 bin_red=2 bin_green=2 失败=0 放错=0 ==
```

记录器可能从第 3 件起就打印「本轮分拣结束」。以 8 件都放完之后的最后一行为准。四种颜色各 2、失败 0、放错 0，就是这轮放对了。

停仿真：在启动它的那个终端按 Ctrl+C，然后等进程退出。不要靠关掉 Gazebo 窗口来停。

## 6. 切换「谁闲推谁」和 Q 表

改的是 `src/a1_sorting_core/config/sorting_params_pipeline.yaml` 里 `task_dispatcher` 的四个参数。这个文件只在流水线模式加载。上面的启动命令默认就是流水线。改完不用重新编译，下一次启动会读到新内容。

四个参数是：

- `q_package_dir`：放着 `dual_arm_rl` 这个 Python 包的目录。
- `q_config_path`：计时配置 JSON，例如 `timing.gazebo-v1.json`。
- `q_table_path`：训好的 `q_table.json`。
- `q_batch`：这一轮蓝、黄、红、绿还要各推几件。演示 8 件时写成 `[2, 2, 2, 2]`。

### 用规则：谁闲推谁

把 `q_config_path` 和 `q_table_path` 都改成空字符串 `""`。两个里面有一个是空的，调度器就不加载表，下一件颜色按「谁闲推谁」。

```yaml
    q_package_dir: ""
    q_config_path: ""
    q_table_path: ""
    q_batch: [2, 2, 2, 2]
```

已经跑通的 8 件接触验证用的就是这个规则。顺序是蓝、红、黄、绿，再来一轮。

### 用 Q 表

三个路径都要填上，而且配置 JSON 和 Q 表里的 `overlap_validated` 必须一样。现在这两份文件都是 `false`。不要只改其中一个。表加载时会核对这个字段和配置内容；对不上就拒绝加载，日志里会出现 `Q 表没有接上,仍用谁闲推谁`，然后仍按规则来。

这个分支不包含那两份 JSON。它们留在训练那一侧。`deployment_ready` 仍是 `false`。

在已经保存过表的这台电脑上，仓库里的 yaml 写的是：

```yaml
    q_package_dir: /home/yuri/lead_ws/ai-collaboration/codex_rl
    q_config_path: /home/yuri/lead_ws/ai-collaboration/codex_rl/config/timing.gazebo-v1.json
    q_table_path: /home/yuri/lead_ws/ai-collaboration/codex_rl/artifacts/gazebo-v1/q_table.json
    q_batch: [2, 2, 2, 2]
```

换一台电脑时，把这三个路径改成你自己的文件。路径不存在时，启动日志会写 `Q 表没有接上,仍用谁闲推谁`，演示仍会按规则跑完，不会因为缺表而停掉。

接上之后，皮带上仍一次一件，取件区的锁仍在，不会给正忙的那台臂提前送下一件。两台都空闲、四种颜色还各剩 2 件时，这张表现在的选择是红色、交给 arm2。这是读表算出来的，还没有在 Gazebo 里跑过。表里没有的状态，改用表代码里的规则：先给空闲的臂，同一台臂再按颜色序号。这和「谁闲推谁」按件数少的优先不完全一样。

加载成功时，日志里会有一行 `Q 表已接上`，并打印状态个数和配置文件里的 `overlap_validated`。

`test/test_q_feed.py` 直接读上面这台电脑上的两个 JSON。没有那两个文件时，这 4 个测试会失败。其余测试不读它们。

## 这份代码里已经包含的改动

相对组长原来的串行演示，`ros2/src` 里有这些运行差异：

- 推板回位等推板真正回到起点附近（世界坐标约 0.14 米以内），再用仿真时间 12 秒做兜底。流水线路径不再用墙钟 2.6 秒当作推板已经回家。
- 两台正在使用的关节轨迹控制器 `goal_time` 是 2.0 秒仿真时间。轨迹计划走完后再等这么久，还没进容差就结束，避免执行槽一直被占住。默认控制器文件是 `ur5e_cell_controllers.yaml`。`ur5e_cell_controllers_gain10.yaml` 不是这轮演示用的文件。
- 夹起之后先在取件点正上方竖直再抬 0.10 米，然后转向对应箱格。
- 每台臂各有一个执行槽。取件区另外有一把锁：派发时锁上，执行器发出 `ZONE_CLEAR`（正常原因是这次竖直抬高）后才允许另一台进入。
- 来料一次一件。`q_config_path` 和 `q_table_path` 都非空且能加载时问 Q 表，否则用「谁闲推谁」。忙着的臂不预送。见第 6 节。

手臂和夹爪上有只读接触传感器，碰撞体的尺寸和位置没有改。演示命令不依赖接触记录也能跑。可选的记录脚本在 `ros2/measure/record_contacts.py`，要在仿真启动前运行。两轮 8 件接触验证和网格对照的摘要在 `ros2/measure/CONTACT_ROUNDS.md`。大的接触 csv 只提交了 gzip，都小于 10 MB。更新后的交接和审阅在 `ros2/docs/HANDOFF.md`、`ros2/docs/REVIEW.md`。更早的组长交接 `ros2/docs/HANDOFF-20261008.md` 仍留在原处。

`ros2/DUAL_ARM_RL_PLAN.md` 是更早的方案草稿。当前启动命令不会一次放三件到带子上。

语音节点不在这条启动命令里。它如果要用密钥，只从环境变量 `VOICE_LLM_API_KEY` 读取。不要把密钥写进仓库。
