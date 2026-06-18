# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Set target name
set(CUR_TARGET_NAME "xyber_controller")

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB_RECURSE src src/*.cpp)

# Add target
add_library(${CUR_TARGET_NAME} STATIC)

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Set head files of target
target_sources(
  ${CUR_TARGET_NAME} INTERFACE FILE_SET HEADERS BASE_DIRS
                               ${CMAKE_CURRENT_SOURCE_DIR} FILES ${head_files})

# Set link libraries of target
target_link_libraries(${CUR_TARGET_NAME} PRIVATE soem)

install(
  TARGETS ${CUR_TARGET_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin COMPONENT ${CUR_TARGET_NAME})

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/${CUR_TARGET_NAME}
  PATTERN "include/internal" EXCLUDE
)
