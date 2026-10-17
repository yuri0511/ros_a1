# 复现与审阅

环境Ubuntu24.04、ROS2 Jazzy、Gazebo Harmonic8.10、MoveIt2.12.4，12vCPU/24GB VMware。锁定pixi.lock，不重新安装依赖。解压先运行 `sha256sum -c MANIFEST_SHA256.txt`；大证据对照EVIDENCE_FILES_SHA256.txt。

代码从独立上游72ab开始：
```
git clone --branch ros2-dual-parallel https://github.com/yuri0511/ros_a1.git ros_a1_review
cd ros_a1_review
git switch --detach 72ab6f2b4d21eefe1ebb78b83aa8a413e00d7ddd
git switch -c review/local-72ab-v2
git apply --check /ABS/review/source/72ab-to-v2.patch
git apply /ABS/review/source/72ab-to-v2.patch
```
或从source/local-experiment-snapshots.bundle导入三个独立快照；git bundle verify可核验，仍需上游历史。补丁无第三方依赖、build/install/cache。

本机共享环境为/home/yu/ros_a1/ros2/.pixi/envs/default；用experiment_tools/72ab相关activate.sh设置环境，在新ros2目录colcon build --symlink-install。launch必须显式feed_policy:=rule，q模式使用独立policy_params YAML指定schema2包/配置/通用表绝对路径。迁移机器须调整路径，不能依赖旧安装路径。

V2源码source/ros2/codex_rl_v2，配置和Q表training/v2；canonical配置哈希与文件字节SHA不同，两者均保留。历史复现命令会写输出和重新训练：请先复制到新目录，默认只读现有训练产物，本轮无需训练。

物理诊断：1ms/v3，无GUI/RViz，X11/llvmpipe、LP_NUM_THREADS=2，单实例独立HOME/ROS_DOMAIN_ID/GZ_PARTITION/ROS_LOG_DIR/CSV。具体命令和只读观察/停止补丁在diagnostic与其运行metadata。不可在存在未处理READY故障时直接启动16组。

Windows未配置VMware共享时，先在VMware设置→选项→共享文件夹添加Windows目录，Ubuntu的/mnt/hgfs/共享名可写后复制ZIP；本任务不修改Windows配置。也可在Ubuntu文件管理器找到ZIP后使用VMware已启用的拖放功能（是否可用取决于宿主机设置）。无可用共享/SSH服务时不宣称Windows可直接访问。

GitHub只检查自己的授权。CLI缺失时可安装官方GitHub CLI，再用 `gh auth login --web --hostname github.com` [浏览器授权](https://cli.github.com/manual/gh_auth_login)自己的账号；不要提供密码/Token给AI，不登录Yuri账号。不执行推送/建PR，PR草稿文本见github目录。


本地审核后的最终GitHub交付流程与身份要求已更新，以上旧Fork准备流程仅为历史背景，以docs/final_delivery/FINAL_WORK_HANDOFF_CN.md及当前PR为准。
