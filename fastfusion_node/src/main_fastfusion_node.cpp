/*
 * main_fastfusion_node.cpp
 *
 *  Created on: Sep 27, 2015
 *      Author: karrerm
 */

#include "fastfusion_node/fastfusion_node.hpp"
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>


/*
 * ROS-Node including the fastfusion mapping algorithm presented in
 * "Volumetric 3D Mapping in Real-Time on a CPU (Steinbrücker,Sturm, Cremers).
 * The node assumes depth and pose data is available
 */
int main(int argc, char **argv)
{
std::cout << "Starting" << std::endl;
  ros::init(argc, argv, "fastfusion_node");

  ROS_INFO("\nStarting fastfusion node with name %s\n", ros::this_node::getName().c_str());

  FastFusionWrapper fastfusion;

  fastfusion.run();
  return 0;
}

