# 双臂接力流水线 — 接口契约与几何布局(组长定稿,单人编写)

> 本文档是任务1(双臂接力)的唯一契约来源。所有命名/几何/话题/参数以本文为准。
> 文件所有权见文末。队员改动一律不得越界;需要共享区改动时在交付报告提需求。

## 1. 拓扑与布局(世界系,单位 m)

```
              (-x)                        (+x)
   arm1 基座 (0.00, 0.00, 0.602)     arm2 基座 (1.10, 0.00, 1.122)
   立柱:地面落地, z 0~0.40           立柱:台面安装, z 0.72~1.12(桌面高 0.72)
        |                                  |
   来料位 (0.45, -0.15/0/+0.15)      bin_out (0.85, +0.20)
        |                                  ^
        v                                  |
   bin_A (0.62, -0.20)  bin_B (0.62, +0.20) ┘-─ arm2 从料区取件码放到 bin_out
   俯视相机 (0.65, 0.00, 1.55)  ← 两臂工作高度均须避开相机立柱区
```

- **arm1**:现有 UR5e,`tf_prefix` 为空(链接名 `base_link`/`tool0` 等,保持不变,零回归)。
- **arm2**:第二台 UR5e+同款夹爪,`tf_prefix='arm2_'`(链接 `arm2_base_link`…`arm2_tool0`,关节 `arm2_shoulder_pan_joint`…),ros2_control 系统名 `ur_arm2`,夹爪系统名 `arm2_gripper_system`。立柱固连 world,位于工作台另一端(+x 侧),坐在桌面上。
- **bin_out(最终出货区)**:标识板中心 (0.85, +0.20, 0.7225),尺寸 0.24×0.20×0.005,琥珀色(与红/绿料区区分)。放置参数:`bin_out.x=0.85, bin_out.y=0.20, bin_out.z=0.756`,网格偏移与 bin_A/bin_B 相同([-0.05,-0.05,0.05,0.05] / [0.0,-0.05,0.0,+0.05])。
- arm2 可达性核算(竖直向下姿态,水平距离):bin 网格最远格 (0.57,-0.25) 距基座 0.586 ✓;bin_out 网格 (0.80~0.90, 0.15~0.25) 距基座 0.32~0.34 ✓。全部落在 0.25~0.65 舒适域内(避坑值:竖直向下距基座 >0.65 NO_IK 高发)。

## 2. 话题 / 接口契约

| 项 | 契约 |
|---|---|
| arm1 任务命令 | `/task/command` (PartInfo) — 不变 |
| arm2 任务命令 | `/task/command_arm2` (PartInfo) — 新增,消息类型复用 PartInfo |
| 任务状态(共享) | `/task/status` (TaskStatus) — **TaskStatus.msg 新增 `string arm`**,取值 `arm1`/`arm2`;空串视为 arm1(兼容旧消息) |
| 接力时序 | 严格串行:part_i 由 arm1 分拣 COMPLETED → 调度器延迟 relay_delay 后向 `/task/command_arm2` 派发同件 → arm2 终态(COMPLETED/FAILED)→ 才派发 part_{i+1} 给 arm1。arm1 FAILED 则跳过该件接力 |
| arm2 取件位姿来源 | **逻辑相机真值 xy**(俯视安装换算 world_x=cam_x+rel_z, world_y=cam_y+rel_y),z 用参数(料区工件静置中心 0.756)。不依赖 arm1 内部网格计数 |
| arm2 控制器 | `arm2_joint_trajectory_controller`(关节 `arm2_` 前缀 6 轴)、`arm2_gripper_controller`(关节 `arm2_gripper_finger_left/right_joint`,命令话题 `/arm2_gripper_controller/commands`) |
| MoveIt | 单 move_group 管 双组:`ur_manipulator`(arm1,不变)+ `arm2_ur_manipulator`(arm2,链 `arm2_base_link`→`arm2_tool0`)。工具帧 `arm2_tool0`,附着链接 `arm2_gripper_base`,规划帧仍 `world` |
| CSV 台账 | 表头追加两列:`time,part_id,category,target_bin,final_bin,result,arm,phase`;`arm`∈{arm1,arm2},`phase`∈{sort,relay}。旧列含义不变 |
| 判定 | arm1 行:目标=detail(bin_A/bin_B);arm2 行:目标=bin_out。料区 AABB 列表参数化,双臂模式含 bin_out (0.85,0.20,half 0.12/0.10)。去重键 (part_id, arm) |

## 3. 参数(新增到 sorting_params.yaml 的 `/pick_place_relay:` 段,组长写)

执行节点参数化(单臂默认值必须与现行为完全一致):
- `arm_name`(默认 `arm1`)→ 填入 TaskStatus.arm;
- `command_topic`(默认 `/task/command`);
- `pose_from_camera`(默认 false)+ `camera_xy`(默认 [0.65, 0.0]):true 时任务开始从逻辑相机缓存取该件真值 xy,z 用 `feed_z`(arm2 配 0.756);
- `bin_names`(默认 [bin_A, bin_B]):料区循环参数化(含 `place_joints_<name>`、`<name>.x/y/z`),arm2 配 [bin_out];
- 既有参数 tool_frame/group_name/gripper_*/standby_*/attach 等直接以不同值复用。

调度器:`relay_mode`(默认 false)、`relay_command_topic`(默认 /task/command_arm2)、`relay_delay`(默认 1.0)。
记录器:`bin_names` 参数化、`expect_relay`(默认 false,双臂 launch 置 true;end-of-batch 摘要条件 = 每件有 arm2 终态行或任意 FAILED 行)。

## 4. arm2 标定值(联调后定稿,2026-10-01)

- 待机位姿 (0.85, 0.00, 1.30)(tool0,竖直向下),关节角
  `[2.5792, -1.9752, 2.7545, -2.35, -1.5708, 1.0084]`(URDF 链解析 IK 标定,
  与 dual xacro 的 initial_positions 一致)。注意不能为了贴近抓取高度把待机位
  降到 1.1 以下:低位折叠构型会被 MoveIt 判 START_STATE_IN_COLLISION。
- 取件锚定(两槽位,按取件目标 xy 就近选择,锚定后小平移 ≤6cm):
  - bin_A 侧 (0.60, -0.20):`[3.272, -0.8041, 2.0446, -2.8113, -1.5708, 1.7012]`
  - bin_B 侧 (0.60, +0.20):`[2.511, -0.8041, 2.0446, -2.8113, -1.5708, 0.9402]`
- 放置锚定(bin_out 网格中点上方 (0.82, 0.20, 1.10)):
  `[2.1235, -0.7765, 2.5675, -3.3618, -1.5708, 0.5527]`
- 巡航高 carry_safe_z = 1.10(tool0);取件下探用 PTP(pick_descend_linear=false,
  LIN 在 0.71m 远距取件点的路径采样会让 KDL 跳 IK 分支),放置下探保持 LIN;
  速度/加速度缩放 0.25(0.5 会踩 Pilz 5.0 rad/s² 限值边缘)。
- bin_out 网格偏移 x [-0.06,-0.06,0,0] / y [-0.04,+0.04,-0.04,+0.04]:
  各格距基座 ≥0.29m(更近的格 KDL 确定性翻分支),相邻格间距 ≥8cm。
- arm1 料区网格同步改为 y ±0.04(相邻格 8cm 间距,消除释放弹飞)。

## 5. 文件所有权(严格分区)

- **队员A**:`a1_cell_sim/worlds/`、`a1_cell_sim/urdf/`(仅新增与 gripper.urdf.xacro 前缀化;`ur5e_cell.urdf.xacro` 与 `ur5e_cell_dual.urdf.xacro` 为组长文件禁改)、`a1_cell_sim/config/ur5e_cell_controllers.yaml`
- **队员B**:`a1_interfaces/`、`a1_sorting_core/a1_sorting_core/` 下 `state_machine.py`、`task_dispatcher_node.py`、`result_recorder_node.py`、`pick_place_executor_node.py`、`bins.py`、新增纯逻辑模块、`test/`
- **队员C**:`README.md`、`docs/`(本契约文件除外)
- **组长独占**:`a1_cell_sim/launch/`、`a1_sorting_core/launch/`、`config/sorting_params.yaml`、`srdf/`、`attach_helper.py`、`motion_client.py`、`ur5e_cell.urdf.xacro`、`ur5e_cell_dual.urdf.xacro`、`config/moveit_controllers_dual.yaml`、`config/kinematics_dual.yaml`、全部 colcon build / ros2 launch / gz 命令 / git commit

## 6. 并联分色形态条款(布局 v2 重写,2026-10-05/06;v1 版全文转沿革)

> **v1 版(2026-10-04 终审定稿)整体作废转沿革**:其前提"bin_C 判别
> 实验世界变体 sorting_cell_binC.sdf + `world_file:=` 参数 + arm2
> 高肩 z=1.22 投 bin_C"已随布局 v2 收编删除(主线
> `sorting_cell.sdf` = 四色四箱定案态,`world_file:=` 参数不再存在);
> "E122 τ 36.9/37-40 楔死区间"是旧高肩构型标定,v2 两臂同款低肩镜像,
> G0 v3 全表 τmax=20.544 ≪ 30,不再适用。v2 现役条款四条:

1. **一箱一色、四箱不混色(用户钉死,勿动)**:arm1 = 北排
   bin_blue/bin_yellow,arm2 = 南排 bin_red/bin_green;箱心
   (0.30/0.90, ±0.28),坐标容差 ±2cm。调度=单槽串行一件一臂
   (派发生成于上一件完全终态后;并联仅指"一件只归一臂、无二次码放",
   不宣称吞吐;吞吐数字需另做计时轮)。
2. **互达与互避(布局 v2 G0 复核)**:双臂 interlock 最小间隙
   +225.7mm,pick×pick +0.1mm ⇒ 单槽串行是硬约束不是优化项;
   跨臂碰撞对**全部启用**(SRDF 无跨臂禁用对,真实互避),仅臂内
   邻接对按官方 UR 同构禁用。gain 2.0 保持域(G0 τ 表口径):
   全 64 锚 τmax=20.544(loaded),余量 >30 N·m 红线。
3. **带尾取件链 v2 定案口径(G2/G3 定罪,勿回退)**:
   - 两臂 `pick_descend_linear=false`(带尾远距 LIN 采样踩 Pilz
     elbow 6.10 rad/s² 硬限 + KDL 分支翻转双重风险,r1/r1c/r2 三轮实证);
   - 两臂 `vel_scale=acc_scale=0.2`(0.3/0.25 档 verify_probe/lift
     短 LIN 随机耗尽,0.2 为唯一双约束安全档;0.2 的 leg2 爬行前科
     所在腿已被 carry_anchor/settle 全锚 PTP 替代退役);
   - `verify_probe_pan_delta_rad=0.085`:关节空间 pan 弧探针零在线
     IK(带尾深折构型 pose 目标 LIN/PTP 皆可被时间参数化拉成 >60s
     爬行 ⇒ 悬置 goal 占槽 ⇒ -4 洪水 ⇒ move_group 2.12 段错误级联死,
     g3 r1-r1d 四轮定罪;端点位移 40mm 不变,弧线由 verify_follow
     物理判据兜底);
   - `motion_client._drain_result(15s)`:settle 早退还成功前给结果
     回传排空窗(open_loop JTC 长轨迹收尾可晚于物理到位 ~10s+);
   - place 段保持 LIN(手持工件不允许横漂)。
4. **E0 大跨腿楔死定案(勿重引旧假说)**:tabletop 大板碰撞与 table
   基座 z 区间互穿 = gz/ODE 假接触触发器(活体 remove 实验 panEff
   -150→0.1 归零定罪),修复=四块箱底局部碰撞补丁(承托面 z=0.72
   逐字等价);臂关节 dynamics damping=3.0 消欠阻尼甩振放大器;
   协议张指 8N 前置(左指半闭停 y− 内壁面=入箱腿擦壁卡因)。
   **"w3 深卷绕数据 bug + 2π 修复"= 错误结论已作废**(w3=-4.41639
   与 +1.8668 同一物理位形,原锚值下 e0b 全绿)。gz disable_collision
   对 MODEL/LINK 是空操作(v8 要求 type COLLISION)——引用旧"禁碰
   排除"实验前先核对实体类型。

