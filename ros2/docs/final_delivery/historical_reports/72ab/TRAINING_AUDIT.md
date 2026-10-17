# 离线重训练审计

完整基线72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd；本地分支perf/integration-72ab6f2。所有性能数字来自该提交时才标为本轮实测。

旧配置allow_prefeed_busy_arm=true；ROS q_feed.decide_feed复制配置并强制false。旧训练动作空间比实际多，虽加载校验通过，动作合法性语义并不一致。此次独立配置只修正该布尔值、计时版本及限制说明；原feed/pick/post_pick数值、overlap_validated=false原样保留。不覆盖旧配置或Q表。

| 表 | 离线回合 | seed | 状态数 | 本机训练墙钟s |
|---|---:|---:|---:|---:|
| general-8 | 50000 | 42 | 1806 | 10.41 |
| demo-2222 | 50000 | 42 | 292 | 9.22 |

两张表各自从空表训练50,000回合，不是相互续训。使用上游原训练器、alpha=.2、gamma=1、epsilon线性.5→.05。demo固定[2,2,2,2]供本次Gazebo；general为随机8件多重集供其他需求研究。**合计100,000个模型回合，不是Gazebo运行100,000回合。**

新配置canonical SHA256：`0a2a549827d9f23b8791fd66b48caaeab824095091f432f8b0e22b082ca38223`。
配置文件字节SHA256：`72c0fa53e8ed9b4956d5c9536d3275a62e7c3dddd8c748be27053a27bc4ece12`。

- general-8表文件SHA256：`66917abe31145a8c6e5513c7ac11ace5546a526c5783132eebc859036921478c`。
- demo-2222表文件SHA256：`f330039d79cb9518c89bc34013041f13206421f187f4474a4a673b4a4e571de9`。

qtable.load已针对新配置严格校验两张新表；旧表对新配置被拒绝，不能仅改JSON标签冒充重训练。training-manifest.json记录确切路径/时间/训练参数。新表deployment_ready=false，overlap_validated=false；旧文件及其字段未改。

默认2222新表评估8次决策全部命中，未知回退0，第一件由实际Q最大值选择red，未强制颜色。其他165种总数8的有效库存均用相同修正模型完成：

| 表 | 库存种类 | 决策数 | 未知回退 | 回退率 |
|---|---:|---:|---:|---:|
| new_q_demo | 165 | 1853 | 1210 | 65.300% |
| new_q_general | 165 | 1829 | 4 | 0.219% |
| old_q_legal_filtered_historical | 165 | 1840 | 4 | 0.217% |

因此demo表适合指定演示需求，不能对其他需求宣称全面覆盖；general表覆盖更广仍有少数未知状态，保留合法动作回退。实物运行中的ETA/事件分布不同，实际回退必须另从Q_DECISION日志统计。

数据来源：feed(4.20–4.44)、pick(8.09–10.36)、post_pick(7.66–11.83)为Yuri历史Gazebo成功样本均值，非本机72ab/v3重新测量。request_delay=1s、ETA边界[5,10,20]、idle_weight=.05及重叠许可属于模型假设。推板回位/settle、安全离区、失败恢复、待命失败成本缺失或边界不全。原is_placeholder=false仅继承原均值出处，不代表该新配置已完整校准。逐项见DATA_COMPATIBILITY.md。

