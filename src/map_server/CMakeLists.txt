cmake_minimum_required(VERSION 3.16)
project(map_server)

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
find_package(ament_index_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(msgpack-cxx REQUIRED)
find_package(nav_common REQUIRED)

# 地图加载：读比赛地形标注文件 -> 原始占据图（0 自由 / 100 障碍）
add_library(map_loader SHARED src/map_loader.cpp)
target_include_directories(map_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(map_loader PUBLIC nav_msgs nav_common)
target_link_libraries(map_loader PUBLIC msgpack-cxx)

# 代价图膨胀：原始障碍 -> 0/1..99/100 连续梯度代价图
add_library(costmap_inflation SHARED src/costmap_inflation.cpp)
target_include_directories(costmap_inflation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(costmap_inflation PUBLIC nav_common)

# 地图服务器节点
add_library(map_server_component SHARED src/map_server_node.cpp)
ament_target_dependencies(map_server_component PUBLIC
  rclcpp rclcpp_components nav_msgs nav_common ament_index_cpp)
target_link_libraries(map_server_component PUBLIC map_loader costmap_inflation)
rclcpp_components_register_node(map_server_component
  PLUGIN "map_server::MapServerNode"
  EXECUTABLE map_server_node)

install(TARGETS map_loader costmap_inflation map_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/convert_map.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_costmap_inflation test/test_costmap_inflation.cpp)
  target_link_libraries(test_costmap_inflation costmap_inflation)
endif()

ament_export_include_directories(include)
ament_export_dependencies(nav_msgs)

ament_package()
