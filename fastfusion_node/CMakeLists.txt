cmake_minimum_required(VERSION 2.8)
project(fastfusion_node)

#################################################
#	Find the necessary Pkgs 					#		
#################################################
find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen REQUIRED)


#################################################
# 	Perform all set commands                   	#
#################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_BUILD_TYPE Debug)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)
#set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")


#################################################
# 	Include Directories 	                   	#
#################################################
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
#include_directories("/usr/include/eigen3")


catkin_simple(ALL_DEPS_REQUIRED)

#################################################
# 	Add Executables 	 	                   	#
#################################################
add_executable(fastfusion_node
				src/main_fastfusion_node.cpp
				src/fastfusion_node.cpp
				src/online_fusion_ros.cpp)



#################################################
# 	Link to target Libraries                   	#
#################################################
target_link_libraries(fastfusion_node
										${catkin_LIBRARIES}
										${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_mipmap_cpu.a
										${CATKIN_DEVEL_PREFIX}/lib/libgeometryfusion_aos.a
										${CATKIN_DEVEL_PREFIX}/lib/libcamerautils.a
										${CATKIN_DEVEL_PREFIX}/lib/libauxiliary.a
										${OpenCV_LIBS}
										boost_thread
										boost_system
										${PCL_LIBRARIES}
										pcl_common
										pcl_io
										pcl_visualization
										pcl_surface)

cs_install()
cs_export()
