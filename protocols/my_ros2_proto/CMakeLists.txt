cmake_minimum_required(VERSION 3.24)

# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_DIR})

project(${CUR_TARGET_NAME})

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Set build mode as shared library (ros2 only support shared library)
set(CUR_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
# cmake-format: off
rosidl_generate_interfaces(${CUR_TARGET_NAME}
  "msg/JointCommand.msg"
  "msg/JoyStickData.msg"
  "msg/JoyStickState.msg"
  "srv/MyRosRpc.srv"
  DEPENDENCIES std_msgs)
# cmake-format: on
# Restore build mode
set(BUILD_SHARED_LIBS ${CUR_BUILD_SHARED_LIBS})

if(NOT TARGET ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp)
  add_library(${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp ALIAS ${CUR_TARGET_NAME}__rosidl_typesupport_cpp)
endif()

if(NOT TARGET ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp)
  add_library(${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp ALIAS ${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp)
endif()

# Generate AimRT RPC stub code, target name: ${PROJECT_NAME}_aimrt_rpc_gencode
add_ros2_aimrt_rpc_gencode_target_for_one_file(
  TARGET_NAME ${CUR_TARGET_NAME}_aimrt_rpc_gencode # Generated target name
  PACKAGE_NAME ${CUR_TARGET_NAME} # ros2 package name
  PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/srv/MyRosRpc.srv # ros2 srv file
  GENCODE_PATH ${CMAKE_CURRENT_BINARY_DIR} # Generated code save path
  DEP_PROTO_TARGETS
    # Depend on ROS2 MSG
    rclcpp::rclcpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_generator_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_fastrtps_cpp
    ${CUR_TARGET_NAME}::${CUR_TARGET_NAME}__rosidl_typesupport_introspection_cpp)
add_library(${CUR_TARGET_ALIAS_NAME}_aimrt_rpc_gencode ALIAS ${CUR_TARGET_NAME}_aimrt_rpc_gencode)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
