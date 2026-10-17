# Gazebo 可见窗口验收

2026-10-17；基于 fd6e244 的独立分支 feature/voice-ui-idle-review。没有四件抓放、没有云调用。

实际启动三次隔离空闲世界：第一次验证启动/暂停但 Xwayland 根窗口截图全黑；第二次改抓 Gazebo 窗口，确认真实画面但默认视角太远；第三次采用独立 GUI 相机配置并补测真实 ROS 停止服务。三次原始数据均保留，均无任务提交和工件生成。

最终证据目录：`/home/yu/yuri_gazebo_performance/voice_batch_four/ui_idle_runs/idle-20261017T083902-04a4ec`。

![真实空闲Gazebo窗口](/home/yu/yuri_gazebo_performance/voice_batch_four/ui_idle_runs/idle-20261017T083902-04a4ec/desktop.png)

截图可见两台UR5e、传送带/推板、蓝黄红绿箱、俯视相机杆与工作台，窗口是实时 Gazebo 客户端，不是视频。无工件是本轮零送料的预期结果。截图保留全部真实像素；相机视角仅为GUI配置，不修改SDF、URDF或机器人路径。

服务端使用 cell_bringup 的 gazebo_gui=false；另启 `gz sim -g --gui-config .../idle_gui.config`，二者由同一父进程继承同一 GZ_PARTITION、ROS_DOMAIN_ID=89；桥接 `/world/sorting_cell`，截图与 native stats 对应同一世界。主启动支持gazebo_gui参数；独立显示客户端让服务端性能与显示开销可分别观察。RViz、MoveIt、抓放执行器均未启动。送料节点显式 spawn_mode=service，没有请求端。最后只为停止RPC启动调度器，不提供完整硬件READY条件，不提交任务。

软件渲染环境：DISPLAY=:0、既有XAUTHORITY、LIBGL_ALWAYS_SOFTWARE=1、GALLIUM_DRIVER=llvmpipe、LP_NUM_THREADS=2、QT_QPA_PLATFORM=xcb；去除NVIDIA offload/DRI覆盖。沿用gain1/500Hz和原1ms世界。GUI日志含EGL软件渲染选择及反序列化警告，但实际截图成功；本轮收到339帧RGB，未重新进行颜色识别准确率验收。

最终轮按native sim_time/接收墙钟差分，无GUI采样约 1.136，GUI约 1.084，本次下降约 4.6%。这是短时空闲、同轮顺序观察，含高频记录且期间有本地UI测试，不是严格重复性能基准，不能预测抓放时RTF。未见严重降至不可用的情况。

进程资源每秒记录于 resources.txt；Gazebo Ruby包装进程峰值快照：{"178636": {"max_RSS_MiB": 1077.99609375, "max_ps_lifetime_CPU_percent": 237.0}, "179008": {"max_RSS_MiB": 1087.28125, "max_ps_lifetime_CPU_percent": 150.0}}。CPU字段是ps进程生命周期平均值（100%=一核），不是逐秒利用率；RSS含启动峰值。原始资源数据比摘要更完整。

启动：
```bash
bash /home/yu/yuri_gazebo_performance/voice_batch_four/run_idle_gui.sh
```
约一分钟空闲检查后自动暂停和清理；Ctrl+C走同一有界清理。没有任务参数，也不加载旧审批文件。本入口不能用于四件执行。窗口可用鼠标调整观察视角，勿手工生成实体或发送运动命令。最终轮观察器、GUI、场景进程均退出0；调度器SIGINT后的重复shutdown异常详见停止报告。
