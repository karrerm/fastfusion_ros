/*
 * fastfusion_node.hpp
 *
 *  Created on: Sep 27, 2015
 *      Author: karrer
 */

#ifndef INCLUDE_FASTFUSION_NODE_HPP_
#define INCLUDE_FASTFUSION_NODE_HPP_


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "camerautils/camerautils.hpp"
#include "online_fusion_ros.hpp"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "fastfusion_node/online_fusion_ros.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


class FastFusionWrapper {
public:
	FastFusionWrapper(void);
	~FastFusionWrapper(){};
	//-- Main Function
	void run(void);
protected:
	//-- Image Message Callback
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_cam0, const sensor_msgs::ImageConstPtr& msg_cam1);
	void getRGBImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgRGB, cv::Mat *rgbImg);
	void getDepthImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgDepth, cv::Mat *dephtImg);


	//-- Pose Message to eigen (Rotation Matrix + Translation)
	tf::TransformListener tfListener;
	CameraInfo convertTFtoCameraInfo(const tf::Transform& transform);
	//-- ROS node handle
	ros::NodeHandle node_, nodeLocal_;
	cv::Mat intrinsic_;
	OnlineFusionROS onlinefusion_;

	std::string world_id_;
	std::string cam_id_;
};




#endif /* INCLUDE_FASTFUSION_NODE_HPP_ */
