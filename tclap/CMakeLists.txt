cmake_minimum_required(VERSION 2.8)
project(tclap)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


include_directories(${CMAKE_CURRENT_BINARY_DIR})

#cs_add_library(${PROJECT_NAME}	src/camerautils.cpp)				

cs_install()
cs_export()
