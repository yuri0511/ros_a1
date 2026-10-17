# 原始证据与审核包索引

大型原始日志留在Ubuntu，不上传Git。下面是精确路径、字节数和SHA256；原历史审核ZIP亦不覆盖。本轮最多一次Q轮，Rule复用既有真实8件，不进行筛选重跑。

- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/metadata.json`
  - 2447 bytes；SHA256 `920a8ad50a9c650c2f633a3fdc0dfca8751ec2df24586dcbd933331e91700dc2`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/summary.json`
  - 4890 bytes；SHA256 `af5b91fcbfce356102a6062f1f2e9f02064a2fbc4e957bc2db79361e2b850258`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/results.csv`
  - 533 bytes；SHA256 `d07b7fbb77885a2e3b319410d85a81e7ff40edfd13c31d95ba1de7947e00eb01`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/launch.log`
  - 612921 bytes；SHA256 `0c671c70363ddf68a7e6d739217dc7979e03ce4b5a14f1e9d8f1533ef21657a0`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/high-rate.jsonl.gz`
  - 72716344 bytes；SHA256 `8510d8506a9941f8bc28e51c346197e83ff5ea9663e83d377ab631b4c74b10f9`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/native-joints.csv`
  - 248159355 bytes；SHA256 `6b2cb1a3bf572a3f1fd37f0410f7e1b810f4bea1fbbcf8479a505d7bc3d4104f`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/contacts.csv`
  - 152359455 bytes；SHA256 `a9992fe30f654ad6ebc68f6443d097c36ecf5685a847da8692d97abb2ffaec94`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/stats.jsonl`
  - 575345 bytes；SHA256 `01f6e37cc62048f055007dc4deb7641f87f1a344827c7b8a5c613b59958d0c98`
- `/home/yu/yuri_gazebo_performance/control_stability/runs/20261017T031609-gain1-rule-eight/instance-1/launch-environment.json`
  - 1530 bytes；SHA256 `11590bb3ebcf5cd882a1fa8714593dc95f4b20756f1fc9ff7d93191e2a869952`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/metadata.json`
  - 2690 bytes；SHA256 `71d440936b4e683f5b2168ae96746f0fb4eefbb79af1898738e121fc430d05e7`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/summary.json`
  - 5133 bytes；SHA256 `eefe2541fe566e0b1555b1744250bc82b829d67bc7d438f56ffc89fe69ae2e0b`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/results.csv`
  - 533 bytes；SHA256 `037e3c7ab8e71dc6a91b14aa3f1e953e062b16ddd3c6df06c6de1eb066a4895f`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/launch.log`
  - 620117 bytes；SHA256 `cb6acac20db8b72887152474d93dba8d1508a51ce4ba2fc1f40a651cc2ea8c12`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/high-rate.jsonl.gz`
  - 74930124 bytes；SHA256 `c55b20a29280717de22f014c4054c410a436bb37d1aa2ae425428ff69cb9e060`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/native-joints.csv`
  - 258419513 bytes；SHA256 `b220d9d0e900d0cf716ad27db7e6684b41b3fcee3c71196f9d5077109c69baaf`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/contacts.csv`
  - 160836253 bytes；SHA256 `23586f08dbdd72a28db2dbdb1d1d14d274503e08edbfe2937b250e7602c98f4e`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/stats.jsonl`
  - 383244 bytes；SHA256 `83c9a47f3062ff82ea8bc8e54e7cfbd0520aba57b089d3be79b10de9b40a0ef0`
- `/home/yu/yuri_gazebo_performance/final_delivery_v2/runs/20261017T034152-gain1-q-v2-general-eight/instance-1/launch-environment.json`
  - 1695 bytes；SHA256 `c81df78c753a7f5913aeb7d2f6712077b06b8c77587403f8e14473c5a2c6a0c2`

旧报告入口：control_stability/、ready_bugfix/、model_calibration_v2/、integration_72ab6f2/、fast_sim_8da998b/；均位于/home/yu/yuri_gazebo_performance。旧82.95MiB证据ZIP为yuri_review/Yuri_Evidence_72ab_V2.zip，不作为本次Git提交内容。

本次源码及小型审核ZIP见最终回执，压缩包可解压校验MANIFEST_SHA256。Git只含必要源码、gain1配置、原始小CSV/关键日志摘录、只读审计及中文报告；不含Pixi/build/install/log、视频、大CSV、native.so、原始全量high-rate流。
