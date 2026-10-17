# 独立四件文字任务

基线：4c89f4c99b5ac261494fe9c5a42b1c6c4664c614；新分支feature/voice-batch-four；第三格只读研究971243c保持独立，不继续改造。

新原文：“开始分拣，一个黄色、一个红色、两个绿色，先完成黄色。”

|颜色|数量|执行臂|
|---|---:|---|
|黄色|1|arm1|
|红色|1|arm2|
|绿色|2|arm2|
|蓝色|0|无|

顺序yellow, red, green, green，总数4。黄色优先定义为黄色PART_PLACED + 经原阈值验证的ARM_READY + 原sequencer释放任务槽，之后才允许下一次SPAWN_REQUEST。其余各件也采用同样强串行，保留全局取件区锁、单送料槽与独立双臂执行槽。

这是新文字任务的静态测试数据，不是新ASR或LLM输出。原三绿五件录音、JSON和哈希均未修改。记录：tasks/four-part-text.json；request_id=b1badf735a9244a2b1d328fddd3b695c；计划SHA256=b4dd1a60370d344c7aafc36fb9a98b7bffbd4b10383ceca7f1c5e91a7f879d8b。任务文件human_confirmed仍为false；未创建实际运行授权。ROS服务受理时还必须绑定新鲜session_id，断连不自动重发。

中文审计器新增识别“先完成黄色”的显式优先表达，严格数量、禁止控制字段及含糊指令的拒绝规则保留；没有用失败的云解析猜测补全计划。green>=3原安全阻断原样保留，没有新增“两件容量”产品限制。

新四件运行入口绑定本演示批次，以避免误载旧五件计划；不改变通用schema允许的其他合法任务，也不修改原8件入口。
