cmake_minimum_required(VERSION 2.8)
project(auxiliary)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_library(auxiliary STATIC
src/debug.cpp src/memory.cpp src/threadpool.cpp src/plywriter.cpp src/ocv_tools.cpp)

cs_install()
cs_export()
