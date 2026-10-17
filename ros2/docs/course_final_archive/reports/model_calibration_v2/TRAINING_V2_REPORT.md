# V2离线训练与独立评估

两张表从空表分别训练50,000回合，seed=42，alpha=.2、gamma=1、epsilon线性.5→.05，未追加回合、未筛选种子重训。训练情景为经验整件profile重采样；没有运行真实Gazebo训练。

| 表 | 状态数 | 回合 | 训练墙钟秒 | 配置canonical SHA256 |
|---|---:|---:|---:|---|
| demo | 292 | 50000 | 11.99 | ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b |
| general | 1806 | 50000 | 13.06 | ce65cec200920de7f3116ddedffcee900aba21b8d07af405bc84359017c1ca0b |

配置文件字节SHA256：`574a24115d663c8a73c30f897f6b824bd32a6357803147e4a35e76b6339f6b01`。两张表各自文件哈希、参数和源文件哈希见TRAINING_MANIFEST.json及SOURCE_CODE_HASHES.json。deployment_ready/overlap_validated保持false；旧表未覆盖。

## 独立种子/场景

评估使用202700..202899共同场景，独立重复使用303700系列及313700系列；与训练seed不同。每色同一ordinal的未来profile预抽并对普通策略隐藏。共同场景对比控制随机样本；独立场景只作随机波动敏感性。

| 策略 | 共同200场景平均仿真秒 | 标准差 | 未知/决策 |
|---|---:|---:|---:|
| rule | 168.524 | 10.393 | —/— |
| q_v1_prefeed_false | 164.150 | 10.512 | 0/1600 |
| q_yuri_history | 164.150 | 10.512 | 0/1600 |
| q_v2_demo | 168.524 | 10.393 | 0/1600 |
| q_v2_general | 164.150 | 10.512 | 0/1600 |

默认V2表在该共同场景下与Rule相同时间，没有模型提速；通用V2表与旧Q相近，并未超过旧Q。本次默认表均值场景未达DP最短时间，尚无收敛证明；不通过继续训练/挑种子制造优胜。通用表最短时间较好也不能当作物理提速或自动改为生产默认。

## 其他8件需求与合法性

165种总数8的库存全部完成模型任务。

| 表 | 未知回退/决策 | 回退率 |
|---|---|---:|
| q_v2_demo | 1210/1849 | 65.441% |
| q_v2_general | 4/1822 | 0.220% |

全部评估动作均断言属于当前合法动作；这只证明资源门控合法，不证明几何安全。通用表第3+同色件虽能执行，但对应放置/恢复时长未有实测支持。32条旧ROS观察经原decide_feed与新schema2包映射一致，ETA key未变、回退0；这是适配器观察测试，不是新策略轨迹的Gazebo回放。

复现：python3 tools/freeze_model.py；python3 tools/train_v2.py；python3 tools/evaluate_v2.py；python3 tools/adapter_audit.py；python3 tools/assess.py。代码生成在临时目录重建后字节一致，见audit/REPRODUCIBLE_GENERATION.json。普通训练/选择只读state()/legal_actions，未来样本与精确事件队列只由环境推进和oracle DP访问。
