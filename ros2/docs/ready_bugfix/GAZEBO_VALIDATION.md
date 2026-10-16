# 续修真实Gazebo验证：返回速度超限，停止在一件

**初始待机的重复戳误停已消除；一件正确入箱，但返回后未达到物理READY，因此未进行8件Rule。** 所有失败和启动异常均保留，不筛选成功重跑。本轮不是正式吞吐量或Rule失败率样本。

## 运行清单

| 运行目录 | 源码 | 结果 |
|---|---|---|
| runs/20261017T013726-ready-short-1ms | a03bbdc（上一阶段） | 初始待机重复戳误停，历史原样保留 |
| runs/20261017T022945-ready-dedup-one-1ms | e272471 | 来源MessageInfo读取错误，首条消息退出，未开始工件任务，空CSV；不能算分拣失败 |
| runs/20261017T023230-ready-native-one-1ms | f8e484c | part_1 blue入bin_blue，返回持续超速，ARM_FAULT，0个ARM_READY/释放；唯一真正取放验证 |

本次参数均1ms、v3事件等待、headless无RViz、X11/llvmpipe、LP_NUM_THREADS=2、单实例、feed_policy:=rule、最多1件。每轮独立HOME/ROS_DOMAIN_ID81/GZ_PARTITION/日志/CSV，重用Pixi依赖，不修改原工作区。准确命令与环境见各metadata.json及launch-environment.json。

## 真实顺序（后一轮）

| 事件 | ROS仿真秒 | 墙钟Unix秒 |
|---|---:|---:|
| 初始standby Action成功 | 约11.20（关节关联，不是Action原生stamp） | 1792175568.624468 |
| 内部稳定窗口通过 | 最后关节stamp11.453，span.252/125戳 | 1792175568.884394 |
| PICKING | 11.464 | 1792175568.886264 |
| ZONE_CLEAR | 23.002 | 1792175581.257645 |
| PLACEMENT_FINISHED | 30.171 | 1792175589.089142 |
| PART_PLACED（原判据） | 30.173 | 1792175589.091244 |
| return_standby Action SUCCEEDED/error0 | 约34.88（关联关节，不是Action原生stamp） | 1792175594.003578 |
| ARM_FAULT | 42.263 | 1792175602.020672 |
| world pause确认data:true | 不以响应墙钟冒充sim stamp | 1792175602.441607 |

Action日志本身使用墙钟ROS日志前缀；表中Action sim只与邻近原始关节关联，未伪造原生时间戳。FAULT到pause确认约.421墙钟秒。完整生命周期55.275墙钟秒含启动/停止，不能作为成功批次耗时。

初始窗口最大发布速度.007014rad/s、差分.014028、位置误差.00003711rad。返回后最后2sim秒腕3约±.489333rad/s，位置差分最大.978667，100%超原.3门槛；位置虽只偏约.000489rad，不能据位置接近宣布静止。8墙钟秒观察到期触发FAULT。未精确复现历史2.886幅值，但同类Action成功/非静止已复现，详见ROOT_CAUSE_ANALYSIS。

共20971条JointState、904对重复戳、单GID、0倒退。所有17次已记录轨迹结果均SUCCEEDED/error0；无已记录抓取重试/Action中止/超时。ARM_FAULT来自独立物理检查，不能篡写成控制器Action失败。结果CSV保留1条PLACED、bin_blue；没有COMPLETED/ARM_READY/ARM_SLOT_RELEASE，dispatcher记录ARM_FAULT_HALT，故障臂槽未错误释放。ZONE_CLEAR此前合法释放取件区，不是释放臂槽。

## 接触与停止

原生contact CSV55504个接触点行（不是55504次独立碰撞）。未记录双臂/机械臂主体-箱体接触；存在夹爪-箱壁2251行、最大深度.000180m，夹爪-工件52903行、最大.011448m，以及夹爪-传送带350行、最大.000092m。箱壁和抓取穿深原样报告，缺同条件对照，不能据此宣布无新增碰撞或安全认证。详见audit/continuation-native-contacts.json。

监督器成功pause后SIGINT本轮会话；summary.residual_session_cleanup为空、最终无Gazebo/MoveIt/ROS仿真节点残留。多个节点SIGINT记录exit -2/KeyboardInterrupt，这是主动保护清理，不能谎称所有进程自然正常完成8件。

## 验收结论

通过的部分：重复戳可去重且仍核查所有载荷；初始稳定窗口真实通过；PART_PLACED在返回故障后保留；ARM_FAULT停止新派发并保持臂槽。

未通过：返回物理静止、正向ARM_READY→释放的真实路径、两臂并发、完整8件四箱各2件、正常批次退出和物理故障恢复。未运行8件、Q、16组，也未追加重跑。建议先让Yuri审核底层位置控制振荡/position接口终态判据；不要放宽速度/位置/窗口，未证明生产可用。
