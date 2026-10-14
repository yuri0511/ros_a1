cmake_minimum_required(VERSION 3.0.2)
project(a1_sorting_core)
find_package(catkin REQUIRED)
catkin_python_setup()          # 安装 src/a1_sorting_core 纯逻辑模块(state_machine / bins / relay_sequencer / ...)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/task_dispatcher_node.py
  scripts/pick_place_executor_node.py
  scripts/scene_publisher_node.py
  scripts/result_recorder_node.py
  scripts/spawn_parts.py
  scripts/run_trials.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
