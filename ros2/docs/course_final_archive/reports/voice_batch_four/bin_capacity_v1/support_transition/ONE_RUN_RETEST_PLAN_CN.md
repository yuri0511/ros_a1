# 申请：一次红箱第4格有界带载复验

当前状态：等待本轮用户批准，尚未启动 Gazebo。

- 独立世界、ROS_DOMAIN_ID=95、唯一 GZ_PARTITION；gain1、1ms、500Hz、软件渲染；没有批次任务、自动送料、云调用或 GUI 正式执行入口。
- 总墙钟上限180秒，带载子过程上限90秒，单个 Action 上限20秒。
- 一个原尺寸50mm/0.2kg红色校准块，保留既有料箱/夹爪几何和全部硬件/规划约束。
- 顺序：待机反馈检查 → 进入第4格 → −1.5N轻触 → 原−8N锁紧及夹指稳定确认 → 提起32mm → 垂直移出箱口到既有carry位 → 返回上方 → 有界接触下降到本轮原始实测关节姿态 → 支撑确认 → 释放 → 工件回WORLD → 空爪退出及返回。
- 只在受控离开支撑和最终接触放回阶段处理同一个工件/红箱底支撑对；离开后恢复正常矩阵。其他碰撞体从不删除，其他碰撞对不放行。
- 新增只读工件接触传感器；原始接触、native位姿、JointState、控制器状态、Action结果、ACM每次变更/恢复及每段独立轨迹检查全保留。
- 异常接触、穿透增长、夹持丢失、控制器异常、反馈过期或无法退出立即暂停并保存，失败不自动重跑。

批准后才生成一次性批准记录，绑定本地提交SHA。入口以O_EXCL消费该批准，不能重复使用；源码有未提交修改或SHA不符即拒绝启动。

```bash
cd /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/worktree/ros2
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
source install/setup.bash
python research_tools/four_slot/calibrate_idle.py \
  --fingers --arm-probe --empty-slot --release-fixture --lift-fixture \
  --approval-file /home/yu/yuri_gazebo_performance/voice_batch_four/bin_capacity_v1/support_transition/ONE_RUN_APPROVAL.json
```

上述批准文件当前不存在。没有文件时入口已经验证会在启动任何仿真之前拒绝。收到用户本轮明确批准后，由Agent记录真实批准、最终SHA和唯一request_id，再运行一次。

通过标准：完整动作链全部成功；实际工件跟随提起且完全离箱；放回原格位、真实脱离夹爪；正常矩阵恢复且无残留特殊许可；无非目标碰撞、控制器失败或不安全退出。任何一项证据不足则保持BLOCKED。

后续整条送料/视觉/单件分拣仍需接入原执行器和任务状态协议，不能用本次校准块计作已分拣工件。


## 已执行，授权已消费
本提案对应bc0c4cde一次运行已于C-20261017T111639-43f307执行并停止；不得再次使用。结果见ONE_RUN_GAZEBO_RESULT_CN.md。
