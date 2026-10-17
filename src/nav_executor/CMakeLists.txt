cmake_minimum_required(VERSION 3.16)
project(nav_executor)

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
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_common REQUIRED)
find_package(nav_planner REQUIRED)
find_package(nav_controller REQUIRED)

add_library(nav_executor_component SHARED src/nav_executor_node.cpp)
target_include_directories(nav_executor_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(nav_executor_component
  PRIVATE
    nav_planner::astar_planner
    nav_controller::pure_pursuit)
ament_target_dependencies(nav_executor_component PUBLIC
  rclcpp rclcpp_components nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_common)
rclcpp_components_register_node(nav_executor_component
  PLUGIN "nav_executor::NavExecutorNode"
  EXECUTABLE nav_executor)

install(TARGETS nav_executor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
