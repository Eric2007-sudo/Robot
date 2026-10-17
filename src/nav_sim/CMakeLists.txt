cmake_minimum_required(VERSION 3.16)
project(nav_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_common REQUIRED)

# 底盘仿真节点
add_library(robot_sim_component SHARED src/robot_sim_node.cpp)
target_include_directories(robot_sim_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_sim_component PUBLIC
  rclcpp rclcpp_components nav_msgs geometry_msgs tf2 tf2_ros nav_common)
rclcpp_components_register_node(robot_sim_component
  PLUGIN "nav_sim::RobotSimNode"
  EXECUTABLE robot_sim)

install(TARGETS robot_sim_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
