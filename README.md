# A1 工件分类搬运 —— 组长 ROS2 版移植到 ROS1 Noetic(Panda + MoveIt 仿真)

**任务:** 3 个彩色工件,按类别分别搬进两个指定料区(暖色 red/yellow 放 **bin_A**,冷色 green/blue 放 **bin_B**)。连续搬完 3 件,并显示计数结果。
**运行环境:** 你现在的 Noetic + `panda_moveit_config demo.launch`(fake controllers)。**不需要 Gazebo,也不需要 UR5e。**

```
ros_a1/
├── README.md
└── src/                                  ← 把这三个包拷进 ~/catkin_ws/src/
    ├── a1_interfaces/                    自定义接口(和组长同名同字段)
    │   ├── msg/PartInfo.msg  msg/TaskStatus.msg      ← 组长原文件,没改
    │   └── srv/SortTask.srv                          ← 组长原文件,没改
    │       srv/SpawnParts.srv                        ← 新增(代替 Gazebo 生成/删除工件)
    ├── a1_sorting_core/                  分拣核心
    │   ├── src/a1_sorting_core/
    │   │   ├── state_machine.py   bins.py   relay_sequencer.py   ← 组长纯逻辑,原样搬过来
    │   │   ├── panda_geometry.py                                 ← 新增:Panda 抓取几何
    │   │   └── scene_utils.py                                    ← 新增:规划场景工具
    │   ├── scripts/
    │   │   ├── task_dispatcher_node.py        调度
    │   │   ├── pick_place_executor_node.py    取放执行(状态机 + MoveIt)
    │   │   ├── scene_publisher_node.py        静态场景(桌子)
    │   │   ├── spawn_parts.py                 生成工件 + 来料登记
    │   │   ├── result_recorder_node.py        结果判定 + 计数 + CSV
    │   │   └── run_trials.py                  重复测试
    │   ├── config/sorting_params.yaml
    │   ├── launch/sorting_demo.launch
    │   └── test/  test_state_machine.py test_bins.py test_relay_sequencer.py(组长原测试) test_panda_geometry.py(新增)
    └── a1_vision/                        可选拓展:颜色识别
        ├── src/a1_vision/color_classifier.py          ← 组长原文件(HSV 分类)
        ├── scripts/vision_classifier_node.py          ← 改写成"虚拟相机"
        └── test/test_color_classifier.py              ← 组长原测试
```

---

## 1. 组长的 A1 是怎么做的

组长的工程是 ROS2 Jazzy + Gazebo Harmonic,而且做得很重。主要有这些部分:

- **机器人:** 两台 UR5e 立在桌边,末端是自制的双指夹爪,用 `ros2_control` 控制。
- **来料:** 工件是 5cm 的彩色立方体(red / green / blue / yellow),由 `spawn_parts.py` 在 Gazebo 里生成,放在传送带上。pipeline 模式下还有推板把工件推到带尾。
- **视觉(`a1_vision`):** 俯视 RGB 相机做 HSV 颜色分类,逻辑相机给出位置,识别结果发到 `/parts/arrival`(PartInfo)。
- **调度(`task_dispatcher_node`):**
  - 来料登记后进入队列,一次只执行一件,上一件结束再派下一件;
  - 通过 `/task/command` 派发任务,`/sort_task` 服务提供 `start_batch` / `dispatch_single` 两个入口;
  - 另外还有双臂接力、并联分色、推板控制。
- **执行(`pick_place_executor_node`):**
  - 状态机 `IDLE → PICKING → CARRYING → PLACING → COMPLETED / FAILED`,每次状态变化发布到 `/task/status`(TaskStatus);
  - 自己写了 MoveGroup action 客户端,用 Pilz PTP/LIN 运动;
  - "抓住"是用 Gazebo 附着实现的;
  - 失败会重试(attempts=2),可以用 `inject_failure_parts` 注入失败,遇到未知类别直接失败。
- **记录(`result_recorder_node`):**
  - 用逻辑相机的真值位置判断工件最终落在哪个料区:PLACED(放对)/ MISPLACED(放错)/ OUTSIDE(不在任何料区)/ FAILED(失败);
  - 结果写 CSV,发布 `/result/counts` 计数和 `/result/markers` RViz 标记。
- **测试:** `run_trials.py` 自动跑 N 轮,统计成功率。共有 88 个 pytest 单测。

## 2. 保留了什么、改了什么

**保留(可以直接说"沿用组长设计"):**
- 包划分:`a1_interfaces` / `a1_sorting_core` / `a1_vision`,节点名也一样:`task_dispatcher`、`pick_place_executor`、`scene_publisher`、`result_recorder`、`spawn_parts`、`run_trials`、`vision_classifier`。
- 接口:**PartInfo、TaskStatus、SortTask 三个文件一字不改**。
- 话题和服务名:`/parts/arrival`、`/task/command`、`/task/status`、`/sort_task`、`/result/counts`、`/result/markers` 全部相同。
- 状态机:`IDLE→PICKING→CARRYING→PLACING→COMPLETED/FAILED`,`state_machine.py` 原样使用。
- 串行调度:`relay_sequencer.py` 原样使用。
- 料区判定和 CSV 台账:`bins.py` 原样使用。
- HSV 颜色分类:`color_classifier.py` 原样使用。
- 颜色和料区分配:red/yellow → bin_A,green/blue → bin_B(组长 legacy 默认)。工件尺寸 5cm、颜色 RGB 值、默认来料 `red,green,red` 都和组长一样。
- 失败处理:attempts=2 重试、`inject_failure_parts` 注入失败、`unknown_category` 不重试。
- 终态在机械臂回到待机位之后才上报;COMPLETED 的 detail 写目标料区名。
- 组长的单测原样保留,在 Python 3.8(Noetic 用的版本)下全部通过。

**改了(答辩时要能说清楚原因):**

| 项 | 组长 ROS2 | 这里 ROS1 | 原因 |
|---|---|---|---|
| 机器人 | 双臂 UR5e + 自制夹爪 | 单臂 Panda + Panda 手爪 | 你已经装好 panda_moveit_config;UR5e 包没有 ROS1 的 MoveIt 配置 |
| 仿真 | Gazebo Harmonic 物理仿真 | MoveIt 规划场景 + fake controllers | 课程允许只做仿真;不用调物理参数 |
| 运动接口 | 自写 MoveGroup action 客户端 + Pilz | `moveit_commander`:`go()` + `compute_cartesian_path` | ROS1 自带 Python 接口,简单很多 |
| 夹爪 | ros2_control 话题 | MoveIt `hand` 规划组;新增 `/gripper/state` 话题 | 课程要求"夹爪状态" |
| 抓住工件 | Gazebo 附着 | 规划场景 `attach_box` / `remove_attached_object` | 无 Gazebo |
| 来料 | Gazebo 生成 + 传送带 | `spawn_parts` 把立方体放到 3 个来料槽 (0.45, -0.15/0/0.15),直接按颜色发布 `/parts/arrival` | 基础任务是"按给定类别",不要求视觉 |
| 视觉 | 真实相机图像 | 可选;"虚拟相机":按规划场景颜色合成像素,再用组长的 HSV 分类 | demo.launch 没有相机 |
| 结果判定 | 逻辑相机真值 | 查询规划场景里工件的实际位置 | 同样是"外部观测",不信执行器上报 |
| 料区位置 | bin_A (0.35, 0.20) / bin_B (0.50, 0.20) | bin_A (0.35, **+0.35**) / bin_B (0.35, **−0.35**) | 原位置和 Panda 的来料槽重叠 |
| 删除 | 双臂接力 / 并联分色 / 推板 / 按需来料 / 会话楔死检测 / 抓取验证探针 | 无 | 单臂 + 无物理仿真用不到 |
| 新增 | — | `/sort_task` 增加 `reset` 命令;`SpawnParts.srv`;`/result/summary` | 让 `run_trials` 每轮能重新生成同名 part_1..3 |

## 3. 组长文件 → 我的文件 对照表

| 组长 ROS2 文件 | ROS1 文件 | 说明 |
|---|---|---|
| `a1_interfaces/msg/PartInfo.msg` | `a1_interfaces/msg/PartInfo.msg` | 原样 |
| `a1_interfaces/msg/TaskStatus.msg` | `a1_interfaces/msg/TaskStatus.msg` | 原样 |
| `a1_interfaces/srv/SortTask.srv` | `a1_interfaces/srv/SortTask.srv` | 原样(命令多支持 reset) |
| `a1_interfaces/CMakeLists.txt`(rosidl) | `a1_interfaces/CMakeLists.txt`(message_generation) | 构建方式改 |
| — | `a1_interfaces/srv/SpawnParts.srv` | 新增 |
| `a1_sorting_core/state_machine.py` | `a1_sorting_core/src/a1_sorting_core/state_machine.py` | 原样(+ py3.8 兼容一行) |
| `a1_sorting_core/bins.py` | `.../bins.py` | 原样(+ 一行) |
| `a1_sorting_core/relay_sequencer.py` | `.../relay_sequencer.py` | 原样(+ 一行) |
| `a1_sorting_core/task_dispatcher_node.py` | `scripts/task_dispatcher_node.py` | rclpy→rospy;删接力/推板/按需来料 |
| `a1_sorting_core/pick_place_executor_node.py` | `scripts/pick_place_executor_node.py` | 主流程和重试结构一致;运动/夹爪/附着换成 MoveIt |
| `a1_sorting_core/motion_client.py` | (由 `moveit_commander` 代替) | ROS1 不需要自写 action 客户端 |
| `a1_sorting_core/attach_helper.py` | `.../scene_utils.py` + executor 的 `_attach/_detach` | Gazebo 附着 → 规划场景附着 |
| `a1_sorting_core/scene_publisher_node.py` | `scripts/scene_publisher_node.py` | 组长 `_SCENE` 为空,这里放一张桌子 |
| `a1_sorting_core/result_recorder_node.py` | `scripts/result_recorder_node.py` | 逻辑相机 → 规划场景位置;删回收功能 |
| `a1_sorting_core/run_trials.py` | `scripts/run_trials.py` | Gazebo create/remove → `/parts/spawn` |
| `a1_cell_sim/scripts/spawn_parts.py` | `a1_sorting_core/scripts/spawn_parts.py` | batch 模式;Gazebo 实体 → 规划场景立方体 |
| `a1_sorting_core/config/sorting_params.yaml` | `config/sorting_params.yaml` | `ros__parameters` 格式 → ROS1 格式;UR 标定值 → Panda 值 |
| `a1_sorting_core/launch/sorting_demo.launch.py` | `launch/sorting_demo.launch` | Python launch → XML |
| `a1_vision/color_classifier.py` | `a1_vision/src/a1_vision/color_classifier.py` | 原样(+ 一行) |
| `a1_vision/vision_classifier_node.py` | `a1_vision/scripts/vision_classifier_node.py` | 真实相机 → 虚拟相机 |
| `test_state_machine.py` `test_bins.py` `test_relay_sequencer.py` `test_color_classifier.py` | 同名 | 原样,全部通过 |
| `pipeline_logic.py` `camera_pose.py` `grasp_verifier.py`(及对应测试) | 未移植 | 传送带 / 相机 / 物理抓取验证专用 |
| `a1_cell_sim/`(世界、URDF、控制器)和 `a1_ur5e_for_ros1` 包 | 未使用 | 由 panda_moveit_config 代替 |

## 4. 架构(PPT 节点图)

```
 spawn_parts ──/parts/arrival(PartInfo)──► task_dispatcher ──/task/command(PartInfo)──► pick_place_executor ──► move_group
 (或 vision_classifier)        │                 ▲  /sort_task(SortTask)                │  状态机 + MoveIt        (demo.launch)
                               │                 └──────── /task/status(TaskStatus) ◄────┤
                               ▼                                                         └─► /gripper/state
                        result_recorder ◄──── /task/status
                        ├─► /result/counts  [bin_A, bin_B, 失败, 放错]
                        ├─► /result/markers (RViz 料区框 + 计数)
                        └─► ~/.ros/a1_sorting_results.csv
 scene_publisher → 桌子;   run_trials → /sort_task reset + /parts/spawn + 统计
```
对应课程要求的四个 ROS 要点:
- **任务命令:** `/sort_task`、`/task/command`;
- **运动接口:** MoveIt `panda_arm`;
- **夹爪状态:** `/gripper/state`;
- **完成数量反馈:** `/result/counts`、`/result/markers`。

## 5. 编译和运行(Noetic)

```bash
cd ~/catkin_ws/src
unzip ~/Downloads/ros_a1.zip
cp -r ros_a1/src/* .            # 得到 a1_interfaces a1_sorting_core a1_vision
cd ~/catkin_ws && catkin_make    # 或 catkin build
source devel/setup.bash
chmod +x src/a1_sorting_core/scripts/*.py src/a1_vision/scripts/*.py   # 如果权限丢了

roslaunch a1_sorting_core sorting_demo.launch
```
启动后会依次发生:
1. 生成 3 个工件(红、绿、红);
2. 自动登记来料并开始分拣;
3. part_1(红)放进 bin_A,part_2(绿)放进 bin_B,part_3(红)放进 bin_A 的第二个格位。

其他用法:
- 换颜色:`roslaunch a1_sorting_core sorting_demo.launch part_colors:="red,blue,yellow"`
- 手动开始:`auto_start:=false`,然后 `rosservice call /sort_task "command: 'start_batch'
part_id: ''"`
- 视觉拓展:`vision:=true`(来料由视觉节点识别颜色后登记)

### 怎么展示"连续搬 3 件 + 结果"
1. **RViz**(录视频用):
   - 点 Add → By topic,添加 `/result/markers`(MarkerArray),就能看到两个料区的彩色框、"bin_A: 2" 这样的计数,以及顶部的汇总行;
   - 工件本身是规划场景里的彩色方块,MotionPlanning 面板里默认能看到;
   - 加好后 File → Save Config,下次不用重新加。
2. **终端**(另开几个窗口):
   ```bash
   rostopic echo /task/status        # 每件的 PICKING / CARRYING / PLACING / COMPLETED
   rostopic echo /result/counts      # data: [2, 1, 0, 0] = bin_A 2 件、bin_B 1 件、失败 0、放错 0
   rostopic echo /gripper/state      # OPEN / HOLDING:part_1
   rostopic echo /result/summary
   ```
3. **日志**:result_recorder 会打印 `part_1 -> PLACED 类别=red 目标=bin_A 实际=bin_A`,全部完成后打印 `== 本轮分拣结束: bin_A=2 bin_B=1 失败=0 放错=0 ==`。
4. **CSV**:`~/.ros/a1_sorting_results.csv`(每件一行)。

## 6. 测试与分析(评分 15 分)

| # | 测试 | 命令 | 看什么 |
|---|---|---|---|
| T1 | 重复运行 | 先 `roslaunch ... sorting_demo.launch`,再另开终端 `rosrun a1_sorting_core run_trials.py --trials 5` | 5 轮每轮 3/3,用时稳定;结果在 `~/.ros/a1_trials.csv` |
| T2 | 改颜色组合 | `rosrun a1_sorting_core run_trials.py --trials 4 --colors "red,green,red" "blue,yellow,green" "green,green,green" "yellow,red,blue"` | 4 种颜色都分对;同箱 3 件依次放进不同格位 |
| T3 | 失败恢复(拓展) | `roslaunch ... sorting_demo.launch inject_failure_parts:="['part_2']"` | part_2 第一次失败 → 恢复 → 第二次成功,终端有"第 1 次尝试失败,恢复后重试" |
| T4 | 重试用完 | `... inject_failure_parts:="['part_2']" attempts:=1` | part_2 FAILED,调度器跳过它继续搬 part_3;counts 里失败 = 1 |
| T5 | 未知类别 | `rosservice call /parts/spawn "colors: 'red,purple'"` | 会被拒绝(spawn 只接受四种颜色);可以讲:类别不在路由表里时,executor 报 `unknown_category`,不重试 |
| T6 | 视觉(拓展) | `... vision:=true`,再跑 `run_trials.py --trials 3` | 识别结果和生成颜色一致 |
| T7 | 速度 | `velocity_scaling:=0.2` vs `1.0` | 比较每轮用时 |
| 单测 | 纯逻辑 | `cd src/a1_sorting_core && PYTHONPATH=src python3 -m pytest test/`,a1_vision 同理 | 34 + 5 项通过 |

分析时可以讲:
- 每轮用时;
- 失败原因分类(plan_failed / attach_failed / injected_failure);
- 同一料区的格位安排;
- 为什么用"外部观测"判定而不是相信执行器的上报;
- 遇到的问题,比如工件贴着桌面会被判成碰撞,所以放置时抬高 2mm;笛卡尔路径完成不到 90% 时退回普通规划。

## 7. 注意事项 / 没验证的部分

- ⚠️ **没有在真实的 Noetic 环境里跑过**(我这边没有 ROS)。已经做过的检查:
  - Python 3.8 语法编译和 pyflakes 静态检查,全部通过;
  - launch / package.xml / yaml 格式检查;
  - 纯逻辑单测 39 项(组长原测试 + 新增)全部通过。

  需要在实机上确认的有:
  - Panda 抓取高度 `tcp_z_offset: 0.105`;
  - 规划组名 `panda_arm` / `hand`、命名位姿 `ready`(Noetic 版 panda_moveit_config 默认就是这些);
  - 来料槽和料区是否都能规划到。

  如果规划失败,先调 `config/sorting_params.yaml` 里的 `tcp_z_offset`、`pre_grasp_height` 和料区坐标。
- 如果你的 hand 组叫 `panda_hand`,改 yaml 里的 `hand_group`。
- 规划场景里的工件颜色要在 RViz MotionPlanning 的 Scene Geometry 里才能看到;如果看不到颜色,不影响功能。
- 组长的 88 个测试只搬了与 ROS 无关的 4 个文件(39 项全过)。其余测试针对 Gazebo / 传送带 / 双臂,没有移植。
- 虚拟相机的图像是合成的,不是真实相机拍的,答辩时要如实说。真要接相机,就把 `synth_patch` 换成订阅图像话题,再用组长的 `world_to_pixel` 投影。
- 失败恢复时,如果工件在手上,会先解附着,工件会停在原地(规划场景里可能悬空);下一次尝试从当前位置重新抓。
