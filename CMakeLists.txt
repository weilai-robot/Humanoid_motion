# Set namespace
set_root_namespace("xyber_x1_infer")

# assistant
add_subdirectory(assistant/native_ros2_channel_publisher)
add_subdirectory(assistant/native_ros2_rpc_client)
add_subdirectory(assistant/ros2_utils)
add_subdirectory(assistant/native_ros2_ankle_identifier)

# protocols
add_subdirectory(protocols/my_proto)
add_subdirectory(protocols/my_ros2_proto)
add_subdirectory(protocols/example_proto)

# module
add_subdirectory(module/dcu_driver_module)
add_subdirectory(module/ankle_identifier_module)
add_subdirectory(module/joy_stick_module)
add_subdirectory(module/control_module)
add_subdirectory(module/sim_module)

# pkg
add_subdirectory(pkg/pkg1)

# install
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUR_INSTALL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/install/linux)
else()
  message(FATAL_ERROR "Unsupport os")
endif()

install(
  DIRECTORY ${CUR_INSTALL_SOURCE_DIR}/bin
  DESTINATION ./
  USE_SOURCE_PERMISSIONS)

# build all
add_custom_target(
  x1_rl_infer_aimrt_build_all ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CUR_INSTALL_SOURCE_DIR}/bin ${CMAKE_BINARY_DIR}
  DEPENDS aimrt::runtime::main
          xyber_x1_infer::pkg1)
