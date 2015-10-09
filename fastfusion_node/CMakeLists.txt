cmake_minimum_required(VERSION 2.8)
project(fastfusion_node)
#set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED)


#find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#add_executable(pcd_write_test pcd_write.cpp)
#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})


find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(cmake_modules REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen REQUIRED)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_calib3d)
set(CMAKE_AUTOMOC ON)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories("/usr/include/eigen3")
include_directories("/usr/include/")
include_directories("/home/karrer/catkin_ws/src/fastfusion/auxiliary/inlcude/")
include_directories("/home/karrer/catkin_ws/src/fastfusion/camerautils/inlcude/")
include_directories("/home/karrer/catkin_ws/src/fastfusion/tclap/inlcude/")
include_directories("/home/karrer/catkin_ws/src/fastfusion/fusion/inlcude/")
include_directories("/home/karrer/catkin_ws/src/fastfusion/fastfusion_node/inlcude/")
include_directories("/home/karrer/catkin_ws/build/fastfusion_node/include/")
#ADD_DEFINITIONS(${QT_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mavx2 -funroll-loops -DOWNAVX")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")


#if(EXISTS "${ROOT}/usr/lib/x86_64-linux-gnu/libQGLViewer.so")
#message(STATUS "Found qglviewer2, linking QGLViewer")
#set(QGLVIEWER QGLViewer)
#else()
#message(STATUS "Did not find qglviewer2, linking qglviewer-qt4")
#set(QGLVIEWER qglviewer-qt4)
#endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -msse4.2 -funroll-loops")

#FIND_PACKAGE(Qt4 REQUIRED QtCore)
#qt4_wrap_cpp(onlinefusion_node_HEADERS_MOC /home/karrer/catkin_ws/src/fastfusion/fastfusion_node/include/fastfusion_node/online_fusion_ros.hpp)



#cs_add_library(${PROJECT_NAME}	src/main_fastfusion_node.cpp
#								src/fastfusion_node.cpp
#								src/online_fusion_ros.cpp)

add_executable(fastfusion_node
				src/main_fastfusion_node.cpp
				src/fastfusion_node.cpp
				src/online_fusion_ros.cpp)
#				${onlinefusion_node_HEADERS_MOC})


target_link_libraries(fastfusion_node
${catkin_LIBRARIES}
/home/karrer/catkin_ws/devel/lib/libgeometryfusion_mipmap_cpu.a
/home/karrer/catkin_ws/devel/lib/libgeometryfusion_aos.a
/home/karrer/catkin_ws/devel/lib/libcamerautils.a
/home/karrer/catkin_ws/devel/lib/libauxiliary.a
${OpenCV_LIBS}
#${QGLVIEWER}
#QtCore QtGui QtOpenGL
#GL GLU glut GLEW
boost_thread
boost_system
${PCL_LIBRARIES}
pcl_common
pcl_io
pcl_visualization
)

cs_install()
cs_export()
