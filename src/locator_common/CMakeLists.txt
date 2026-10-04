cmake_minimum_required(VERSION 3.10)
project(locator_common)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 纯头文件包，没有需要编译的 .cpp，只把 include/ 导出给下游
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_find_test_dependencies()
  ament_auto_add_gtest(test_tf_utils test/test_tf_utils.cpp)
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
)
