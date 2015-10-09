cmake_minimum_required(VERSION 2.8)
project(camerautils)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories("/usr/include/eigen3")

include_directories(${PROJECT_SOURCE_DIR}/src/camerautils)
include_directories("/usr/include/eigen3")

add_library(camerautils STATIC
						src/camerautils.cpp)
target_link_libraries(camerautils
)

cs_install()
cs_export()
