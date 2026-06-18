cmake_minimum_required(VERSION 3.23)
message("Configuring Xyber Controller project")

project(xyber_controller LANGUAGES CXX C)

# Set cmake path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wl,--disable-new-dtags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wl,--disable-new-dtags")

add_subdirectory(third_party/soem)
add_subdirectory(xyber_api)
# add_subdirectory(example)

install(SCRIPT "cmake/post_install_script.cmake")

# CPack
set(CPACK_PACKAGE_VERSION_MAJOR 0)
set(CPACK_PACKAGE_VERSION_MINOR 5)
set(CPACK_PACKAGE_VERSION_PATCH 0)
set(CPACK_GENERATOR "ZIP")
set(CPACK_PACKAGE_NAME "XyberController")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "A brief description of XyberController: version: v0.5.x")
set(CPACK_PACKAGE_VENDOR "AgibotTech")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Richie.Li")

include(CPack)
