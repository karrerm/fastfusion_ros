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
#include_directories("/usr/include/qt4/QtXml")
#include_directories("/usr/include/qt4")
#include_directories("/usr/include/qt4/Qt")
#include_directories("/usr/include/qt4/QtCore")
#include_directories("/usr/include/qt4/QtXml")
#include_directories("/usr/include/qt4/QtGui")
#include_directories("/usr/include/qt4/QtOpenGL")
#include_directories("/usr/include/")
#ADD_DEFINITIONS(${QT_DEFINITIONS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")
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

#cs_add_library(${PROJECT_NAME}	src/geometryfusion_aos.cpp
#								src/treeandbrick.cpp
#								src/treeandbrick_indexed.cpp
#								src/treeandbrick_incremental.cpp
#								src/treeandbrick_incremental_recursive.cpp
#								src/mesh.cpp
#								src/mesh_interleaved.cpp
#								src/mesh_interleaved_meshcell.cpp
#								src/geometryfusion_mipmap_cpu.cpp
#								src/loopclosure.cpp)
							

				

cs_install()
cs_export()
