cmake_minimum_required(VERSION 3.16)
project(nav_launcher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 校验脚本装成可执行，用 `ros2 run nav_launcher verify_xxx.py` 调用
install(PROGRAMS
  scripts/verify_planner.py
  scripts/verify_reach.py
  scripts/verify_tracking.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
