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
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "fastfusion_node/online_fusion_ros.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/ply_io.h>


class FastFusionWrapper {
public:
	FastFusionWrapper(void);
	~FastFusionWrapper(){};
	//-- Main Function
	void run(void);
protected:
	//-- Image Message Callback
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_cam0, const sensor_msgs::ImageConstPtr& msg_cam1);
	void imageCallbackPico(const sensor_msgs::ImageConstPtr& msg_depth);
	void pclCallback(sensor_msgs::PointCloud2 pcl_msg);
	void getRGBImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgRGB, cv::Mat *rgbImg);
	void getDepthImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgDepth, cv::Mat *dephtImg);


	//-- Pose Message to eigen (Rotation Matrix + Translation)
	tf::TransformListener tfListener;
	CameraInfo convertTFtoCameraInfo(const tf::Transform& transform);
	//-- ROS node handle
	ros::NodeHandle node_, nodeLocal_;
	ros::Time previous_ts_;
	cv::Mat intrinsic_;
	cv::Mat distCoeff_;
	OnlineFusionROS onlinefusion_;
	image_transport::Subscriber *subscriberOnlyDepth_;
	message_filters::Subscriber<sensor_msgs::Image> *subscriberRGB_;
	message_filters::Subscriber<sensor_msgs::Image> *subscriberDepth_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > *sync_;

	void broadcastTFchain(ros::Time timestamp);
	tf::TransformBroadcaster tfBroadcaster1_, tfBroadcaster2_;
	tf::Transform tf_cam0_imu, tf_depth_cam0;
	Eigen::Matrix3d R_cam0_imu, R_depth_cam0;
	Eigen::Vector3d t_cam0_imu, t_depth_cam0;
	std::string world_id_;
	std::string cam_id_;

	bool use_pmd_;
	bool testing_point_cloud_;
};




#endif /* INCLUDE_FASTFUSION_NODE_HPP_ */
