# 运行与录像说明：目前仅云链路/接口可演示

## 环境与构建

```bash
cd /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
colcon build --symlink-install --base-paths src
source install/setup.bash
python -m pytest src/a1_sorting_core/test src/a1_vision/test research_tools/test_voice_contract.py -q
```

API Key已由用户本机不回显导入。不要将Key放共享目录或聊天。需要重新导入时：

```bash
python3 /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/import_private_key.py
```

当前授权最多4次，实际已用3次，不应为录像反复付费调用。录音/云处理CLI不导入ROS，不送料；实际本轮原始记录在audio/voice-primary/，成功复核audit/real-speech-validated.json。可直接展示已有真实文字/JSON及日志，不需重新调用。

## 当前安全检查入口（会阻断，不启动Gazebo）

```bash
YURI_PIXI_PREFIX=/home/yu/ros_a1/ros2/.pixi/envs/default bash /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/run_voice.sh
```

返回green_third_slot_safety_not_cleared是本版预期安全结果。此入口目前不是已联调的完整仿真启动器，不能改成跳过错误继续跑。主计划人工确认客户端也会先过同一几何门：

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/install/setup.bash
python /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/submit_cli.py --candidate /home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/real-speech-validated.json --receipt /home/yu/yuri_gazebo_performance/voice_batch_aliyun/operator/main-request.json
```

当前会在ROS提交/真人确认之前拒绝绿3；未来安全修复后才允许用户在TTY输入CONFIRM。客户端会先保存request_id意图，断连不自动重发。付费API的YES授权不是CONFIRM，不批准机械臂运动。

无Gazebo服务通信复现（与主任务物理执行无关）：

```bash
ROS_DOMAIN_ID=84 GZ_PARTITION=voice-offline-service-20261017 ROS_LOG_DIR=/home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/ros-smoke python /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/transport_smoke.py
```

该工具是隔离测试，不是实际五件演示，硬件readiness永远不满足，0送料，结束自动销毁本工具ROS节点。不要与其他实例共用domain/partition。

## 几何证据复现

```bash
source /home/yu/yuri_gazebo_performance/voice_batch_aliyun/activate.sh
python /home/yu/yuri_gazebo_performance/voice_batch_aliyun/tools/static_green.py
g++ -O2 /home/yu/yuri_gazebo_performance/voice_batch_aliyun/worktree/ros2/research_tools/aliyun_voice/mesh_box_check.cc -o /home/yu/yuri_gazebo_performance/voice_batch_aliyun/tools/mesh_box_check $(pkg-config --cflags --libs fcl assimp) -std=c++17 -Wl,-rpath,$CONDA_PREFIX/lib
python /home/yu/yuri_gazebo_performance/voice_batch_aliyun/tools/make_mesh_queries.py
/home/yu/yuri_gazebo_performance/voice_batch_aliyun/tools/mesh_box_check /home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/mesh-queries.txt > /home/yu/yuri_gazebo_performance/voice_batch_aliyun/audit/mesh-results-reproduced.csv
python /home/yu/yuri_gazebo_performance/voice_batch_aliyun/tools/verify_box_overlap.py
```

这些是只读模型查询，不能当实际Gazebo。原始首次mesh-results.csv保留，复现输出另存。生成URDF源与命令见audit/，本机工具ROOT固定实验目录；未伪造缺失G0文件。

## 录像与PPT

现在可开：Ubuntu录音/识别终端、已保存ASR文本和LLM JSON、本地校验/安全阻断输出、几何风险报告。字幕应写“云链路已完成，物理联合验证因绿3安全检查暂停”。不要拼接旧8件Gazebo画面称本次5件成功。

第三格修复并完成真实验收后，录像才开独立Gazebo GUI（如果GUI路径未验证应明确新显示条件）、可选RViz与实时批次状态/结果CSV终端；显示黄色PLACED+READY后才生成红色、最终绿3红1黄1/5READY。GUI不能启动另一套单臂执行器，不使用旧gain2入口；倍速明确标注。

任何已授权后续运动轮的安全停止应通过当前batch stop阻断新派发/送料，并由有界监督器暂停该GZ_PARTITION世界、SIGINT本轮进程组；不全局pkill，不把ROS/world reset当故障恢复。当前无Gazebo可停，没有运行中机械臂。
