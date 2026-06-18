# post_install_script.cmake
message("Executing post-install script...")

# 删除所有的静态库文件
# message(STATUS "-- Deleting: ${CMAKE_INSTALL_PREFIX}/lib/*.a")
# file(GLOB STATIC_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/*.a")
# file(REMOVE ${STATIC_LIBRARIES})

# 删除soem相关头文件
file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/include/soem)

# 删除cmake目录
message(STATUS "-- Deleting: ${CMAKE_INSTALL_PREFIX}/cmake")
file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/cmake")

# 删除share目录
# message(STATUS "-- Deleting: ${CMAKE_INSTALL_PREFIX}/share")
# file(REMOVE_RECURSE "${CMAKE_INSTALL_PREFIX}/share")

# 删除所有无用文件
message(
  STATUS
    "-- Deleting: ${CMAKE_INSTALL_PREFIX}/bin/simple_test eepromtool"
)
file(
  GLOB
  REDUNDANT_FILES
  "${CMAKE_INSTALL_PREFIX}/bin/simple_test"
  "${CMAKE_INSTALL_PREFIX}/bin/eepromtool"
  "${CMAKE_INSTALL_PREFIX}/bin/*.py")
file(REMOVE ${REDUNDANT_FILES})
