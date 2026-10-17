cmake_minimum_required(VERSION 3.16)
project(nav_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯头文件库：只安装头文件，不编译目标。导出 include 目录，
# 让下游包 ament_target_dependencies(... nav_common) 能拿到头文件路径。
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_package()
