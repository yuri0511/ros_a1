# Q、DP与真实物理结果

旧Yuri计时训练prefeed=true，ROS运行prefeed=false不一致；72ab整合在prefeed=false下从空表重新训练，原始Q保留。旧简化模型Rule126.49秒、Q122.32秒、时间DP122.32秒；仅模型内最优。

真实72ab三配对Rule184.59墙钟秒，Q187.96秒（+3.37秒/+1.83%），双方24/24正确，抓取重试3/7。Q决策24/24命中，无WAIT/未知回退；另独立功能轮8次命中。不能声称RL提速。

V2使用pair1/2整轮32件校准，pair3整轮16件留出，接触精密16件仅诊断。推板回位、抓取成功阶段、重试prefix、安全离区、放置和return_tail显式分段。训练各50000回合、seed42，本次不重复训练。

| 均值模型 | 秒 |
|---|---:|
| Rule/V2 demo | 168.488 |
| 旧Q/V2 general/时间DP | 164.015 |

V2 general并未超过旧Q。默认表在模型中与Rule同耗时，不作为优化候选。普通Q只能读取已观察ETA（配置均值减已历时），不能访问未来profile；完整事件队列DP如知道未来样本属于额外信息上界。经验重采样不是已校准的真实失败概率，永久失败/物理READY故障未充分建模。

Q−Rule关键路径分解：额外重试prefix均值+13.417墙钟秒，其他阶段抵消，整批净+3.37秒。4次额外重试的累计prefix差约40.25秒，算术上足以覆盖10.11秒总差，但并非确定因果；不能删除重试后称物理反事实。留出仅2轮，MAE6.482仿真秒，不代表普适预测精度。

V2配置canonical SHA ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b；general文件SHA18c4b343b0c359b2c740d9d6eeabef273be3d93e4e573ee28a32b53216977e2c。完整训练元数据见training/v2。本次物理诊断结果见DIAGNOSTIC_PAIR_REPORT。
