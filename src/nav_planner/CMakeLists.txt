cmake_minimum_required(VERSION 3.16)
project(nav_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_common REQUIRED)

# A* 规划器（考生实现）
add_library(astar_planner SHARED src/astar_planner.cpp)
target_include_directories(astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(astar_planner PUBLIC nav_common)

install(TARGETS astar_planner EXPORT export_nav_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_nav_planner HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_link_libraries(test_astar_planner astar_planner)
endif()

ament_package()
