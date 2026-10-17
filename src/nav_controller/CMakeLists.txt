cmake_minimum_required(VERSION 3.16)
project(nav_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_common REQUIRED)

# 纯追踪控制器（考生实现）
add_library(pure_pursuit SHARED src/pure_pursuit.cpp)
target_include_directories(pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pure_pursuit PUBLIC nav_common)

install(TARGETS pure_pursuit EXPORT export_nav_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_nav_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(nav_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pure_pursuit test/test_pure_pursuit.cpp)
  target_link_libraries(test_pure_pursuit pure_pursuit)
endif()

ament_package()
