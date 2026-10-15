cmake_minimum_required(VERSION 3.8)
project(a1_cell_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds launch config srdf
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/spawn_parts.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
