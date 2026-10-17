# 格位配置与容量管理

配置：ros2/src/a1_sorting_core/config/four_slot_candidate.json。唯一键为bin_name/slot_1..4，每格携带arm、xyz与carry/settle_top/settle_bottom关节目标。每臂两箱共8组，按身份选锚，不按最近距离猜箱。

| 箱 | 格位1 XY | 格位2 XY | 格位3 XY | 格位4 XY |
|---|---|---|---|---|
| bin_blue | (0.205, 0.235) | (0.295, 0.235) | (0.205, 0.325) | (0.295, 0.325) |
| bin_yellow | (1.005, 0.235) | (1.095, 0.235) | (1.005, 0.325) | (1.095, 0.325) |
| bin_red | (0.205, -0.325) | (0.295, -0.325) | (0.205, -0.235) | (0.295, -0.235) |
| bin_green | (1.005, -0.325) | (1.095, -0.325) | (1.005, -0.235) | (1.095, -0.235) |

所有格位名义中心z=0.745 m；tool0三阶段高度分别1.080、0.990、0.958 m。退出候选为LIN升到1.080 m，再PTP返回；保留原世界参数。

`SlotBank`实现EMPTY→RESERVED→OCCUPIED，异常转FAULT。持久化库存与几何校验；重启后残留RESERVED转FAULT。第五件拒绝，不取模覆盖。清空必须匹配part_id、删除成功ACK、更新的源时间戳及新帧里该件确实消失。没有按时钟自动释放格位。

新overlay里recycle_delay=0，源码核对只有>0才创建自动回收功能。新VoiceBatchState(retain_capacity=4)绕过旧模式“同色必须先回收”的要求，仍要求前件PLACED、READY、槽释放齐全。旧默认retain_capacity=0以及原每色上限3保持原行为。

位置确认提出XY±8 mm/Z±10 mm与0.2 sim秒新鲜度门控，这是候选落格验证条件，不是已测得精度。相机Z尚未与native验证，因此不能打开运行许可。motion_validated=false使executor/dispatcher新模式拒绝启动。库存路径必须独立；本次没有任何真实库存文件或真实工件计数。


状态：独立候选工程，第二关未完整通过，禁止动态抓放。此次没有启动 Gazebo，没有新的实际 PLACED/READY 或接触测量。原工作树 e4dad347d6ef50d5faa4000fbb7888042e815df6 保持不动。
