cmake_minimum_required(VERSION 3.10)
project(locator_launcher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/verify_accuracy.py
  scripts/verify_cassini.py
  scripts/verify_gimbal.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
