# 双臂出料调度离线原型

标准库 Python 3.10+，无需 ROS、Gazebo、numpy 或安装第三方依赖。按上一级 PLAN.md 的现行修订实现；不导入、不修改组长 ROS 包。

## 运行

在此目录执行：

```bash
python3 -m unittest discover -s tests -v
python3 -m dual_arm_rl.train --episodes 50000 --epsilon-start 0.5 --epsilon-end 0.05 --seed 42
python3 -m dual_arm_rl.qtable
python3 -m dual_arm_rl.evaluate --batches 30 --seed 2026
```

产物：`artifacts/q_table.json`、`artifacts/comparison.csv`。重复运行相同命令可复现（JSON training 部分也记录超参数）。默认训练随机生成总数为 8 的四色批次，评估使用独立种子。训练未覆盖的状态自动回退简单规则，不保证任意批次最优。

## 文件与职责

- `config/timing.placeholder.json`：唯一的默认时间参数配置，包括四色送料、取件、离区后耗时、后续请求延迟、ETA 分箱。占位值未来自 Gazebo。
- `dual_arm_rl/model.py`：事件队列、互斥资源、单件送料容量、状态编码、串行及规则基线。
- `dual_arm_rl/train.py`：Q-learning，固定 gamma=1、有限回合、合法动作 bootstrap。
- `dual_arm_rl/qtable.py`：JSON 读写、校验、合法动作选择、未知状态回退。
- `dual_arm_rl/evaluate.py`：相同库存批次的三个策略对比，输出 CSV。
- `tests/test_scheduler.py`：时间算例、81 种批次数量组合的资源不变量、Q 表及训练测试。

## 模型语义

固定颜色需求，动作选择下一件送料颜色。只有一个送料槽，包括已请求未到达、运输中、等待抓取及取件中。每臂最多执行一个任务，取件区只有一个 owner。PICK_EXIT 是抽象的安全退出事件，不等于现有 ROS CARRYING。

`step(FEED_*)` 一直处理事件到该件安全退出取件区；期间同时推进另一臂的完成事件。可以给忙臂提前送料，待目标臂 READY 再取件；该行为由 `allow_prefeed_busy_arm` 控制。`step(WAIT)` 推进到下一个事件，不允许零时间 WAIT。最后一次请求之后会排空所有收尾事件，计入最终奖励及总时间。

`allow_arm_overlap=false` 令两臂全作业周期互斥，但仍允许送料重叠；仅为保守场景，不能代表实际 MoveIt 执行槽的全部行为。

串行基线保留“上一件返回后才请求下一件”的时序，可传入原颜色顺序。简单规则优先 READY 臂，之后 ETA 档小者优先，同档按颜色序。两者和 Q 策略共享时长与资源约束。串行基线是原版调度时序的抽象，不是完整 ROS 仿真。

时钟单位统一为模型秒；计时从首件请求开始，排除进程启动等待，后续请求延迟另计。idle 是有本臂未服务需求时的 READY 时间之和，两臂相加可能大于批次时长。空闲臂若无剩余本臂需求，则不扣空等分。

## Q 表接口

PLAN.md 定义十元素状态、五动作顺序、ETA 边界含义。状态 key 使用 `state_key()`，例如 `[1,1,1,1,"POST_PICK","READY",2,0,0,-1]`；每行五个 Q 值。颜色数字 0..3；臂数字 0/1 对应 arm1/arm2。

先 `load(path, config)`，再 `choose(table, model)`。未知状态回退简单规则。配置哈希或版本错误直接拒绝；不要静默用错表。原型接口参数 model 需提供 state()/legal_actions()/config，实际 ROS 适配由 Grok 实现。线上动作掩码必须由真实资源状态计算，不能照抄模型估计来解锁。

## 等待实测的限制

交付表始终 `deployment_ready=false`；`is_placeholder=true` 和 `overlap_validated=false` 明确标注未经验证。不得把本次比较当作 Gazebo 成果，也不承诺 Q 表优于简单规则。

当前只含确定性成功路径。未建模随机失败、恢复轨迹、箱格耗时、实际碰撞及共享 MoveIt 逐轨迹排队，因此没有失败奖励或实测成功率。RECOVERY 不在 Q 表决策状态中：出现异常必须由确定性逻辑处置，不得靠 Q 表解锁。

Grok 在 HANDOFF.md 提供数据后，先核对安全退出/READY/送料预取语义，再更新配置及时长版本、重训、重新验证。配置哈希改变后旧 Q 表会被拒绝。若箱格、执行槽或异常对时间有明显影响，需要扩展模型及 schema，不能仅替换平均值。此轮不制作分析图，CSV 已为后续 analysis.py 留出输入。


## 2026-10-15 训练与回退计数更新

默认 50,000 局，ε 每局线性衰减，首局 0.5、末局 0.05；局内保持不变。训练元数据记录两端值及 schedule。评估不做 ε 探索。
`comparison.csv` 新增 `q_decision_steps` 和 `unknown_state_fallback_steps`，分母是 Q 策略实际调用的决策次数（包括 WAIT，不含内部自动处理的事件）。已知状态 Q 值并列时使用规则打破平局，不算未知状态回退。串行/规则行这两列为 0，不表示它们使用 Q 表。
评估同时输出 `comparison.summary.json`，按策略汇总均值与回退率。旧 5,000 局表的相同种子复评保存在 `comparison_5000.csv` 及对应 summary；旧表可从前一 Git 提交获取。


## Gazebo v1 成功段均值实验（2026-10-16）

配置 `config/timing.gazebo-v1.json` 使用 HANDOFF 末尾修绕远路后的仿真秒均值，样本绿3/蓝6/黄5/红9，失败和重试未计入。原占位配置和产物保持不变。

```bash
python3 -m dual_arm_rl.train --config config/timing.gazebo-v1.json --output artifacts/gazebo-v1/q_table.json --episodes 50000 --epsilon-start 0.5 --epsilon-end 0.05 --seed 42
python3 -m dual_arm_rl.qtable --config config/timing.gazebo-v1.json --table artifacts/gazebo-v1/q_table.json
python3 -m dual_arm_rl.evaluate --config config/timing.gazebo-v1.json --table artifacts/gazebo-v1/q_table.json --batches 30 --seed 1 --output artifacts/gazebo-v1/seed_1.csv
```

评估命令的 seed 和输出文件依次替换为 1..5。配置为 is_placeholder=false、overlap_validated=false，Q 表仍 deployment_ready=false。模型源码与奖励没有改变。

注意测量边界与原模型假设不完全一致：feed 仅到停推进，缺推板回位/停稳时间；pick 到 carry_clear 时手仍在取件点上方，不代表已经验证安全退出。此轮按用户指定数值直接做成功路径模型实验，未补造回位时间，继承的 request_delay_seconds=1.0 不能解释为实测回位仿真秒。重叠及忙臂预送料仍是假设；输出不可当作实际 Gazebo 完整批次时间或安全验证。完整测量口径见配置 measurement_metadata 与 REVIEW.md。
