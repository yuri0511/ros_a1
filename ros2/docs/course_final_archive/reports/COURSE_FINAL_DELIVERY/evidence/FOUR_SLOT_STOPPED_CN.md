# 最后集中攻关结果与课程交付回退

结论：**持件同步候选代码已实现，初始带载规划通过；完整放置轨迹未通过，第二关继续BLOCKED。按止损要求停止四格扩容研发，转课程PRE/OBS及原8/8基线交付。**

## 修复及真实证据范围

- 原先Gazebo跟随与MoveIt载荷碰撞不同步。现于实际夹紧确认后、首次带载提起前，加入完整50mm立方体AttachedCollisionObject；运行接口使用同一时间的实际观测与TF，保留完整相对平移/旋转。Gz跟随不替代MoveIt附着。
- 场景Apply ACK及GetPlanningScene读回确认对象唯一性、尺寸、相对姿态、触碰链；释放后由新鲜真实观测恢复世界障碍。故障时保留仍被夹持的碰撞体，阻断继续运动，不假装释放。
- 首次离开支撑仅允许已验证夹紧、真实接触、正确离开方向下的**该工件与该支撑面**碰撞对；其他矩阵项保持，离开后恢复。独立轨迹复核禁止向下增加穿透。
- 连续带载规划在执行前加入<=1ms采样的实际全尺寸载荷FCL检查。队列、50ms、50唯一样本及原READY门控未放宽；joint_receiver.py与本轮起点逐字节一致。
- A、B黄色单件、C历史五件入口已接线，C保留原真实ASR/LLM记录及哈希，不调用云API。ABC总入口在发出任何动态启动前检查全路径；motion_validated仍false。
- 上述为**未完成动态验收的候选代码**。不能称为真实Gazebo持件同步、释放、故障恢复已经通过。

## 最后静态检查结果与止损原因

复用保存的黄色实际抓取数据，利用原始JointState及相机模型重建抓取相对位姿（离线FK重建，不冒充当时录制的TF）；运行代码另用真实TF查询。

1. 初始提起、验证探针往返、carry_rise、carry_clear共5段真实Pilz规划，3,118个<=1ms状态采样通过。支撑转换在模型回放中检查，不是本轮真实夹持/接触认证。
2. 6格位共48段真实Pilz请求，42段计划及88,140个采样状态通过，6段settle_bottom失败。整链没有通过。
3. 黄色slot1下降终点：`retained_part_1`（附着物体）与`tabletop_floor_yellow`相交 **0.0029970930188440515m，约2.997093mm**。这是保存的实际持件变换与当前名义放置目标不匹配导致的**规划模型碰撞**；不是本轮Gazebo动态碰撞，也不是51.5nm数值支撑接触。
4. 红/绿各终点还报告约51.53–51.55nm支撑相交，仍未放行。没有把黄色3mm穿透混入限定支撑许可，没有删除箱体或缩小工件。
5. `--check`正确返回BLOCKED。因此没有消耗A来制造一个不能继续到B的局部成功，也没有开启新一轮路径/几何研究。

部分早期离线检查脚本漏乘MoveIt规范化后的CollisionObject.pose，误报t=0穿透增长；该**检查器坐标组合错误**已修正后保留重算结果。旧日志保留，不记为机器人真实故障或Gazebo重跑。

## A/B/C授权与实际执行

|阶段|实际启动|结果|
|---|---|---|
|A 红4带载校准|否|静态前置阻断|
|B 真实黄色单件|否|静态前置阻断，A亦未通过|
|C 历史五件|否|静态前置阻断，B亦未通过|

**授权3次，实际消耗0次。** 本轮没有新的PLACED、ARM_READY、CSV、动态重试、接触或控制器成功/失败统计。不得用历史记录补成成功。

本轮只启动无执行能力的MoveIt plan-only服务，未启动Gazebo GUI/服务器。已有真实llvmpipe GUI截图属于之前的空闲世界，不能当成本轮动态画面。最后检查无Gazebo/MoveIt残留。

## 构建、测试与源码

- ROS构建a1_cell_sim/a1_sorting_core成功。
- 持件/相关定向测试56通过；核心模块最终366通过、4项历史失败；验收工具最终31通过。不同测试集合有重叠，不相加为总覆盖数。
- 4项历史失败：camera水平FOV；3项legacy anchor/G0基线资料不一致或缺失，详见原始pytest日志。未伪造证据修为PASS。
- 17个变更Python文件语法解析通过；git diff --check通过。
- 主要变更：held_payload_scene.py；executor、motion_client、attach_helper；只读contact观测/spawn传感器；ABC调度及permit；acceptance_task；两套离线路径复核；相关测试。baseline_gui.py仅创建显示客户端。
- 原始Rule/Q、最终8/8工作树及UI工作树均干净；未推送。

## 原始证据索引

本目录`latest-payload-review.txt`与`latest-all-path-review.txt`指向RESULT.json、完整请求/响应、采样结果与规划日志。另保存build.log、held-tests.log、core-tests-final.log、tool-tests-final.log、preflight-final.json、static-receipt.json、authorization-usage.json、SOURCE_DIFF.patch及MANIFEST_SHA256.txt。早期失败日志保留。

历史真实8/8：`/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/`。历史语音计划：`/home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/real-speech-validated.json`，plan SHA256 `481ae03763ddd82c59a38a46a0ac39d34aad6f50ff1dccf6901b0d78bcc797cb`。

## 正式演示回退及录屏命令（仅准备，未执行）

使用原有gain1+READY八件Rule入口，**不使用四格候选或旧gain2日常脚本**：

```bash
YURI_PIXI_PREFIX=/home/yu/ros_a1/ros2/.pixi/envs/default \
bash /home/yu/yuri_gazebo_performance/final_delivery_v2/worktree/ros2/research_tools/final_system/run_rule.sh
```

该启动器先打印新运行目录。第二终端用其**准确目录**替换下方占位符：

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
python /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2/research_tools/four_slot/baseline_gui.py /home/yu/yuri_final_runs/<上一步打印的实际目录名>
```

客户端从metadata读回同一GZ_PARTITION/ROS_DOMAIN_ID及llvmpipe环境，命令只有`gz sim -g`。不生成任务、不启动第二个世界。客户端接口单元测试通过；本轮未实测新的八件GUI视频。OBS录制实际Gazebo及终端状态/结果，不把历史语音解析画面说成实时端到端语音抓放；倍速需标注。

停止：启动器终端Ctrl-C或`touch <实际运行目录>/STOP`；显示客户端在其终端Ctrl-C。启动器故障先请求当前世界暂停，再有界结束子进程。不要使用全局pkill。

课程报告/PPT/视频方案沿用`/home/yu/yuri_gazebo_performance/final_delivery_v2/`下COURSE_REPORT_CN.md、PPT_OUTLINE_CN.md、VIDEO_DEMO_PLAN_CN.md与FINAL_WORK_HANDOFF_CN.md，补充本报告结果。建议6–8页、2–3分钟：A1任务与ROS架构→真实8/8演示→gain1/READY修复→Rule/Q结果→ASR与LLM证据→边界。

已经证实的是历史一次8/8（148 Action成功、8 READY、0重试/FAULT），不是长期安全认证；旧72ab低干扰配对Q平均187.96s、Rule184.59s，无Q实机仿真提速证据。真实ASR与LLM解析已完成，四格五件语音端到端尚未打通。真实物理机器人安全性未验证。

四格仅可无动作检查：
```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
cd /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2
source install/setup.bash
python research_tools/four_slot/concentrated_acceptance.py --check
```

不得将BLOCKED检查入口当作正式五件运行命令。停止自动研发，等待课程交付审核。
