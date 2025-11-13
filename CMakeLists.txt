cmake_minimum_required(VERSION 3.8)

set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

project(fpga_driver C CXX)
set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/build)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_PREFIX_PATH}/include/yaml-cpp)
include_directories(${CMAKE_PREFIX_PATH}/include/eigen)

set(HOME_DIR $ENV{HOME})
add_definitions(-DCONFIG_PATH=\"${HOME_DIR}/corgi_ws/fpga_driver/config/config.yaml\")
add_subdirectory(src)


