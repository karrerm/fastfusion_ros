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
#include <nav_msgs/Odometry.h>
#include <Eigen/SVD>
#include "fastfusion_node/online_fusion_ros.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/ply_io.h>
#include "fastfusion_node/valueToColor.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
class FastFusionWrapper {
public:
	FastFusionWrapper(void);
	~FastFusionWrapper(){};
	//-- Main Function
	void run(void);
protected:
	//-- Image Message Callback
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_cam0, const sensor_msgs::ImageConstPtr& msg_cam1);
	void imageCallbackPicoSLAM(const sensor_msgs::ImageConstPtr& msgDepth, const sensor_msgs::ImageConstPtr& msgConf,
			const sensor_msgs::ImageConstPtr& msgNoise, const nav_msgs::Odometry::ConstPtr& msgOdom);
	void imageCallbackPico(const sensor_msgs::ImageConstPtr& msgDepth, const sensor_msgs::ImageConstPtr& msgConf,
				const sensor_msgs::ImageConstPtr& msgNoise);
	void imageCallbackPicoSLAM(const sensor_msgs::ImageConstPtr& msgDepth, const sensor_msgs::ImageConstPtr& msgConf,
			const nav_msgs::Odometry::ConstPtr& msgOdom);
	void imageCallbackPico(const sensor_msgs::ImageConstPtr& msgDepth, const sensor_msgs::ImageConstPtr& msgConf);
	void pclCallback(sensor_msgs::PointCloud2 pcl_msg);
	void getRGBImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgRGB, cv::Mat *rgbImg);
	void getConfImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgConf, cv::Mat *confImg);
	void getDepthImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgDepth, cv::Mat *dephtImg);
	void getNoiseImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgNoise, cv::Mat *noiseImg);
	void registerPointCloudCallbackSLAM(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg,
			const nav_msgs::Odometry::ConstPtr& msgOdom);
	void registerPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg);
	void depthImageCorrection(cv::Mat & imgDepth, cv::Mat * imgDepthCorrected);

	//-- Color encoding
	void jetColorNoise(const cv::Mat &imgNoise, cv::Mat *imgRGB, const float min, const float max);

	//-- Pose Message to eigen (Rotation Matrix + Translation)
	tf::TransformListener tfListener;
	CameraInfo convertTFtoCameraInfo(const tf::Transform& transform);
	//-- ROS node handle
	ros::NodeHandle node_, nodeLocal_;
	ros::Time previous_ts_;
	cv::Mat intrinsic_, intrinsicRGB_;
	cv::Mat distCoeff_, distCoeffRGB_;
	cv::Mat depthCorrection_;
	double imageScale_;

	OnlineFusionROS onlinefusion_;
	//-- Subscribers
	message_filters::Subscriber<sensor_msgs::Image> *subscriberRGB_;
	message_filters::Subscriber<sensor_msgs::Image> *subscriberDepth_;
	message_filters::Subscriber<sensor_msgs::Image> *subscriberNoise_;
	message_filters::Subscriber<sensor_msgs::Image> *subscriberConfidence_;
	message_filters::Subscriber<nav_msgs::Odometry> *subscriberOdometry_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> *subscriberPointCloud_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
			sensor_msgs::Image, nav_msgs::Odometry> > *syncNoiseSLAM_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
				sensor_msgs::Image> > *syncNoise_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, nav_msgs::Odometry> > *syncSLAM_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> > *sync_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> > *syncPointCloudSLAM_;
	void broadcastTFchain(ros::Time timestamp);
	tf::TransformBroadcaster tfBroadcaster_;
	tf::Transform tf_cam0_imu;
	tf::Transform tf_depth_cam0;				// When using ToF camera
	tf::Transform tf_rgb_cam0;				// When using Realsense
	tf::Transform tf_body_cam;				// Transfrom VICON-Body to imu
	Eigen::Matrix3d R_cam0_imu, R_depth_cam0, R_rgb_cam0, R_body_cam;
	Eigen::Vector3d t_cam0_imu, t_depth_cam0, t_rgb_cam0, t_body_cam;
	std::string world_id_;
	std::string cam_id_;

	bool runMapping_;
	int frameCounter_;
	bool use_pmd_, depth_noise_;
	bool testing_point_cloud_;
	pcl::PointCloud<pcl::PointXYZRGB> pointCloudFrame_;
	pcl::PointCloud<pcl::PointXYZRGB> pointCloudFrameTrans_;

	//-- Running estimate for decay time
	bool useSlam_;
	double *latestDerivativeSingularVals_;
	double lastSingularVal_;
	int singValCounter_;
};




#endif /* INCLUDE_FASTFUSION_NODE_HPP_ */
