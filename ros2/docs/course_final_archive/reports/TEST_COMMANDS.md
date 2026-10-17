# 测试命令与环境（自动从实际 metadata.json 导出）

# 每轮使用独立目录；以下精确实例命令用于审计。建议复现使用 tools/run_case.py，自动处理采样、独立 CSV 和停止。

## 20261016T122139-B-headless-1x

目标=1；实例=1；LP线程=default；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T122139-B-headless-1x/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T122139-B-headless-1x-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T122139-B-headless-1x/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_1.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T122139-B-headless-1x/instance-1/results.csv'
```

## 20261016T122615-C-target2-screen

目标=2；实例=1；LP线程=default；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T122615-C-target2-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T122615-C-target2-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T122615-C-target2-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_2.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T122615-C-target2-screen/instance-1/results.csv'
```

## 20261016T122743-C-target4-screen

目标=4；实例=1；LP线程=default；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T122743-C-target4-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T122743-C-target4-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T122743-C-target4-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T122743-C-target4-screen/instance-1/results.csv'
```

## 20261016T122911-C-unthrottled-screen

目标=0；实例=1；LP线程=default；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T122911-C-unthrottled-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T122911-C-unthrottled-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T122911-C-unthrottled-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T122911-C-unthrottled-screen/instance-1/results.csv'
```

## 20261016T123041-C-unthrottled-LP2-screen

目标=0；实例=1；LP线程=2；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T123041-C-unthrottled-LP2-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T123041-C-unthrottled-LP2-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T123041-C-unthrottled-LP2-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T123041-C-unthrottled-LP2-screen/instance-1/results.csv'
```

## 20261016T123209-C-cyclone-LP2-screen

目标=0；实例=1；LP线程=2；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T123209-C-cyclone-LP2-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T123209-C-cyclone-LP2-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T123209-C-cyclone-LP2-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T123209-C-cyclone-LP2-screen/instance-1/results.csv'
```

## 20261016T123339-C-cyclone-LP2-EGL-screen

目标=0；实例=1；LP线程=2；观测=continuous-high-frequency-pilot

```bash
env PERF_EGL=1 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T123339-C-cyclone-LP2-EGL-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T123339-C-cyclone-LP2-EGL-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T123339-C-cyclone-LP2-EGL-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T123339-C-cyclone-LP2-EGL-screen/instance-1/results.csv'
```

## 20261016T123450-C-fastdds-large-LP2-screen

目标=0；实例=1；LP线程=2；观测=continuous-high-frequency-pilot

```bash
env 'FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=2MB&sockets_size=4MB&non_blocking=false' XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T123450-C-fastdds-large-LP2-screen/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T123450-C-fastdds-large-LP2-screen-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T123450-C-fastdds-large-LP2-screen/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T123450-C-fastdds-large-LP2-screen/instance-1/results.csv'
```

## 20261016T123619-C-optimized-full

目标=4；实例=1；LP线程=2；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T123619-C-optimized-full/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T123619-C-optimized-full-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T123619-C-optimized-full/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T123619-C-optimized-full/instance-1/results.csv'
```

## 20261016T124032-C-fast-single-full

目标=4；实例=1；LP线程=default；观测=continuous-high-frequency-pilot

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T124032-C-fast-single-full/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T124032-C-fast-single-full-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T124032-C-fast-single-full/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T124032-C-fast-single-full/instance-1/results.csv'
```

## 20261016T124244-B-formal-headless-1x

目标=1；实例=1；LP线程=default；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T124244-B-formal-headless-1x/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T124244-B-formal-headless-1x-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T124244-B-formal-headless-1x/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_1.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T124244-B-formal-headless-1x/instance-1/results.csv'
```

## 20261016T124659-C-formal-target4

目标=4；实例=1；LP线程=default；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T124659-C-formal-target4/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T124659-C-formal-target4-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T124659-C-formal-target4/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T124659-C-formal-target4/instance-1/results.csv'
```

## 20261016T125109-D-isolation-check

目标=4；实例=2；LP线程=2；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T125109-D-isolation-check-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-1/results.csv auto_start:=false'
```

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-2/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=62 GZ_PARTITION=20261016T125109-D-isolation-check-2 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-2/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125109-D-isolation-check/instance-2/results.csv auto_start:=false'
```

## 20261016T125208-D-formal-dual

目标=4；实例=2；LP线程=2；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T125208-D-formal-dual-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-1/results.csv'
```

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-2/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=62 GZ_PARTITION=20261016T125208-D-formal-dual-2 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-2/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125208-D-formal-dual/instance-2/results.csv'
```

## 20261016T125832-E-formal-triple

目标=4；实例=3；LP线程=2；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T125832-E-formal-triple-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-1/results.csv'
```

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-2/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=62 GZ_PARTITION=20261016T125832-E-formal-triple-2 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-2/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-2/results.csv'
```

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-3/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=63 GZ_PARTITION=20261016T125832-E-formal-triple-3 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-3/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T125832-E-formal-triple/instance-3/results.csv'
```

## 20261016T130501-C-formal-target2

目标=2；实例=1；LP线程=default；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T130501-C-formal-target2/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T130501-C-formal-target2-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T130501-C-formal-target2/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_2.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T130501-C-formal-target2/instance-1/results.csv'
```

## 20261016T130857-C-formal-unthrottled

目标=0；实例=1；LP线程=default；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T130857-C-formal-unthrottled/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T130857-C-formal-unthrottled-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T130857-C-formal-unthrottled/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_0.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T130857-C-formal-unthrottled/instance-1/results.csv'
```

## 20261016T131301-C-formal-target4-LP2

目标=4；实例=1；LP线程=2；观测=light-with-10s-latency-window

```bash
env XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.XTD9W3 HOME=/home/yu/yuri_gazebo_performance/runs/20261016T131301-C-formal-target4-LP2/instance-1/home ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST DISPLAY=:0 ROS_DOMAIN_ID=61 GZ_PARTITION=20261016T131301-C-formal-target4-LP2-1 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/runs/20261016T131301-C-formal-target4-LP2/instance-1/ros LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe QT_QPA_PLATFORM=xcb LP_NUM_THREADS=2 bash -c 'source /home/yu/yuri_gazebo_performance/activate.sh; source /home/yu/yuri_gazebo_performance/workspace/ros2/install/setup.bash; export PATH=/home/yu/yuri_gazebo_performance/tools/bin:$PATH; exec ros2 launch a1_sorting_core sorting_demo.launch.py gpu_offload:=false gazebo_gui:=false launch_rviz:=false world_file:=performance_rtf_4.sdf result_csv:=/home/yu/yuri_gazebo_performance/runs/20261016T131301-C-formal-target4-LP2/instance-1/results.csv'
```
