cmake_minimum_required(VERSION 3.16)
project(car_detector CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 只依赖 OpenCV，dnn 模块用来加载 ONNX。装 libopencv-dev 即可
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs dnn)

add_executable(car_detector
  src/main.cpp
  src/detector.cpp
)

target_include_directories(car_detector PRIVATE include)
target_link_libraries(car_detector PRIVATE ${OpenCV_LIBS})
if(MSVC)
  target_compile_options(car_detector PRIVATE /W3 /utf-8)
else()
  target_compile_options(car_detector PRIVATE -Wall -Wextra)
endif()
