# 集中验收前置接线：仍BLOCKED

- 没有带载标定、送料抓放或付费API。接收器源码与f64c35c完全一致，没有重复压力测试。
- motion_validated仍false。新增的受限试验许可要求原始完整A阶段证据、精确源码/几何/路径哈希、黄色slot1/arm1/仅1件及有效期；不是全局认证。缺证据就拒绝，不会改JSON标志。
- exit_clearance门没有删除：原入口仍拒绝；受限入口还须真实两指8–14mm贯穿退出、来源时间间隔<=50ms、原新鲜度、目标误差<=0.01rad/速度<=0.3rad/s、50个唯一样本/0.25sim秒稳定及三类完整夹爪碰撞体均高于0.815m箱沿，才允许全开。
- 原候选四格下降是PTP，但原审查为LIN；本轮只把四格专用下降/退出与LIN审查对齐。MotionClient默认仍PTP，Rule/Q原参数和入口不变。
- 已实现run_yellow_single.py、独立task_record/观察器装配；复用真实spawn/RGB/dispatcher/MoveIt/原结果记录器与物理READY审计。单件计划0/1/0/0、无旧五件语音冒称；没有真实部件ID、PLACED或READY造假。实际入口当前拒绝运行。
- 实际ROS服务模拟通过：未确认0送料、确认但无硬件反馈仍0送料、拒绝非黄色计划/重复确认/旧SortTask旁路，stop保持HALTED。仅模拟契约，没有Gazebo，没有物理READY。
- 单元状态机模拟确认单件必须PART_PLACED+ARM_READY+释放槽才COMPLETED，完成后不发第2件；模拟数据未写成真实CSV。
- 两个ROS包构建通过；核心回归357通过+4项相同历史失败（相机FOV、三项旧锚/G0文件）；定向测试84通过。冻结接收器哈希核对、原Rule/Q工作树完整性核对通过。
- 本轮定向真实Pilz/FCL审查16段、25065个<=1ms轨迹采样，黄1/红4 carry_clear→carry→下降→带已落件退出→返回通过，箱沿上方夹爪完整包络通过。规划原始请求/返回保留。最初审查器错误地使用全场景替换，清除了SRDF默认自碰撞矩阵；修正为差分更新并只移除已存在的审查库存。失败日志原样保留，没有放宽任何碰撞对。支撑脱离/落回沿用未改动源码的4021点已有规划证据，下一轮A仍每段独立检查，不冒称已动态通过。
- 两次空闲GUI检查，无动作/送料：首次桌面根截图被拒后正常停止；第二次读回Gazebo自有窗口像素成功，窗口PID环境GZ_PARTITION与同世界server一致，World=sorting_cell。原始截图、RGB图、GUI记录在calibration_runs/A-20261017T120851-e44136/。RGB640x480，非黑，样本std24.89；GUI画面工作站较小，实际录屏前可手动缩放。Xlib读回已隔离子进程，窗口丢失/超时不会杀死监督器。首次失败和第二次成功均保留，无筛选动态成功。

## 最直接、尚未解决的技术阻断

当前执行器抓取验证后调用的是Gazebo GzKinematicAttach（改变仿真跟随），它不是MoveIt附着碰撞物体。四格的RetainedSceneSync仅同步已放置和已释放的物体；初始抓取→带载抬升/探针→carry_clear期间，没有真实所持工件的MoveIt碰撞场景同步，也没有这段完整载荷路径的独立证据。本轮16段审查从历史carry_clear起点开始，不能冒充已覆盖前段。

所以集中--check明确BLOCKED，不会仅凭红4校准成功或motion_validated布尔值启动B。A也尚无完整真实通过证据。本轮按要求在发现该直接阻断后停止，没有继续扩大为运动/支撑模型研究。

## 下一轮命令与次数

检查（无运动）：
```bash
cd /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
source install/setup.bash
python research_tools/four_slot/concentrated_acceptance.py --check
```
将来补齐上述载荷场景和路径证据、取得绑定最新commit的明确授权文件后，统一入口：
```bash
python research_tools/four_slot/concentrated_acceptance.py --approval-file /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/acceptance_preparation/NEXT_DYNAMIC_APPROVAL.json
```
本轮没有创建此批准文件。设计最多2个独立世界：A一次（180秒）→证据审查→B一次（300秒）；失败不重跑，GUI/摄像头未确认不提交任务。目前不能申请宣称已具备全部条件的A+B最终验收。
