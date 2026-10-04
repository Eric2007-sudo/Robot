cmake_minimum_required(VERSION 3.10)
project(locator_sim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB sources "src/*.cpp")
ament_auto_add_library(${PROJECT_NAME} SHARED ${sources})

# 四个虚拟节点都注册成组件，既能各自独立跑，也能一起塞进同一个容器。
# 塞进同一个容器是默认用法：它们要共享 SimClock 的进程内 t0。
# 启动方式见 locator_launcher 包的 sim_only.launch.py。
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN locator_sim::VirtualSerialNode
  EXECUTABLE virtual_serial_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN locator_sim::VirtualImuNode
  EXECUTABLE virtual_imu_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN locator_sim::VirtualDetectorNode
  EXECUTABLE virtual_detector_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN locator_sim::SimVisualizerNode
  EXECUTABLE sim_visualizer_node
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
)
