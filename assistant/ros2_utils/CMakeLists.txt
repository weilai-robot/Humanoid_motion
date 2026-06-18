cmake_minimum_required(VERSION 3.8)
project(ros2_utils)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
