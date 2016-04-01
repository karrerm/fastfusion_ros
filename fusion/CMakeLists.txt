cmake_minimum_required(VERSION 2.8)
project(fusion)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories("/usr/include/eigen3")
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

add_library(geometryfusion_aos STATIC
								src/geometryfusion_aos.cpp
								src/treeandbrick.cpp
								src/treeandbrick_indexed.cpp
								src/treeandbrick_incremental.cpp
								src/treeandbrick_incremental_recursive.cpp
								src/mesh.cpp
								#meshcelltraversal.cpp
								src/mesh_interleaved.cpp
								src/mesh_interleaved_meshcell.cpp)
						
add_library(geometryfusion_mipmap_cpu STATIC
								src/geometryfusion_mipmap_cpu.cpp
								src/loopclosure.cpp)
target_link_libraries(geometryfusion_mipmap_cpu
						boost_thread
						${PCL_LIBRARIES}
						)		
				
cs_install()
cs_export()
