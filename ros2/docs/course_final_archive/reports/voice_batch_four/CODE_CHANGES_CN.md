# 本轮代码与运行版本

4c89f4c为原语音项目；62987d07fbc5dd2b4518d81405aef01e904f573e为本轮唯一真实运行版本，启动时工作树干净。

四件新增：独立启动器/观察器/事件台账/人工批准与单次消费；voice_batch_plan支持“先完成”；voice_batch_state回收源时间；dispatcher仅voice模式重新确认推板home与新鲜消失帧；测试与报告。

中止后仅修正监督器停止顺序与记录窗口：先pause世界，再ROS停新派发，最后退出观察器；增加bounded停止ACK/时间线、cleanup_ok和两个离线测试。未变更机械臂几何、轨迹、碰撞参数、控制器profile、READY阈值或8件入口。该补丁没有Gazebo验证，不能替代62987d0的真实源码身份；观察器-6尚未查明。

最终SHA以LOCAL_COMMIT.txt为准。原始metadata/summary保留，不回写为新版源码。

原始CSV的CRLF和原始日志末尾空行均按字节保留；完整diff的空白提示仅涉及这些原始证据。源码和手写文档的diff空白检查通过，不为消除提示改写证据。
