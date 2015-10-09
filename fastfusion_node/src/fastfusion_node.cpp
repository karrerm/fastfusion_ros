/*
 * fastfusion_node.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: karrer
 */

#include "fastfusion_node/fastfusion_node.hpp"


FastFusionWrapper::FastFusionWrapper():  nodeLocal_("~") {
//Constructor FastFusionWrapper
//-- Load Parameters
	// dummy: so far no parameter
	intrinsic_ = cv::Mat::eye(3,3,cv::DataType<double>::type);
	bool loadSuccess = true;
	bool threadFusion, threadMeshing;
	int depthChecks;
	double imageScale,scale,threshold;
	loadSuccess &= nodeLocal_.getParam("threadFusion", threadFusion);
	loadSuccess &= nodeLocal_.getParam("threadMeshing",threadMeshing);
	loadSuccess &= nodeLocal_.getParam("imageScale",imageScale);
	loadSuccess &= nodeLocal_.getParam("scale",scale);
	loadSuccess &= nodeLocal_.getParam("threshold",threshold);
	loadSuccess &= nodeLocal_.getParam("fx", intrinsic_.at<double>(0,0));
	loadSuccess &= nodeLocal_.getParam("fy", intrinsic_.at<double>(1,1));
	loadSuccess &= nodeLocal_.getParam("cx", intrinsic_.at<double>(0,2));
	loadSuccess &= nodeLocal_.getParam("cy", intrinsic_.at<double>(1,2));
	loadSuccess &= nodeLocal_.getParam("depthConsistencyChecks", depthChecks);
	if (loadSuccess) {
		ROS_INFO("\nFastfusion: Could read the parameters.\n");
		onlinefusion_.setupFusion(threadFusion, threadMeshing,(float) imageScale, (float) scale, (float) threshold, depthChecks);

	} else {
		ROS_ERROR("\nFastfusion: Could not read parameters, abort.\n");
	}
	//-- Set the corresponding fields

}

void FastFusionWrapper::run() {
	ROS_INFO("\nRun Fastfusion .....");
	// Synchronize the image messages received from Realsense Sensor
	message_filters::Subscriber<sensor_msgs::Image>
	subscriberRGB(node_, node_.resolveName("/image/rgb_raw"), 5);
	message_filters::Subscriber<sensor_msgs::Image>
	subscriberDepth(node_, node_.resolveName("/image/depth_raw"), 5);
	message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
	sync(subscriberRGB, subscriberDepth, 10);
	sync.registerCallback(boost::bind(&FastFusionWrapper::imageCallback, this,  _1,  _2));
	ros::spin();
	//-- Stop the fusion process
	onlinefusion_.stop();
	ros::shutdown();


}

void FastFusionWrapper::imageCallback(const sensor_msgs::ImageConstPtr& msgRGB, 
										const sensor_msgs::ImageConstPtr& msgDepth) {
//-- Callback function to receive depth image with corresponding RGB frame as ROS-Messages
//-- Convert the messages to cv::Mat and wait for tf-transform corresponding to the frames
//-- Push the data into the fastfusion pipeline for processing.
	if (onlinefusion_.isReady()){

		cv::Mat imgRGB, imgDepth;

		//-- Convert the incomming messages
		getRGBImageFromRosMsg(msgRGB, &imgRGB);
		getDepthImageFromRosMsg(msgDepth, &imgDepth);


		//-- Get Pose (tf-listener)
		tf::StampedTransform transform;
		try{
		  tfListener.lookupTransform("/world", "/cam",
								   ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		}
		//-- Convert tf to CameraInfo (fastfusion Class in camerautils.hpp)
		CameraInfo incomingFramePose;
		incomingFramePose = convertTFtoCameraInfo(transform);
		//-- Fuse the imcoming Images into existing map
		onlinefusion_.updateFusion(imgRGB, imgDepth, incomingFramePose);
	} else {
		std::cout << "Dropped Frame " << std::endl;
	}
	
}


void FastFusionWrapper::getRGBImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgRGB, cv::Mat *rgbImg) {
//-- Function to convert ROS-image (RGB) message to OpenCV-Mat.
	*rgbImg = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8)->image;
}

void FastFusionWrapper::getDepthImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgDepth, cv::Mat *depthImg) {
//-- Function to convert ROS-image(depth) message to OpenCV-Mat.
	cv::Mat depthImg2 = cv_bridge::toCvCopy(msgDepth, sensor_msgs::image_encodings::MONO8)->image;
	//-- Convert the image to 16 bit (fastfusion requires 16 bit depth images)
	depthImg2.convertTo(*depthImg,CV_16UC1);
	*depthImg = *depthImg*255;
}

CameraInfo FastFusionWrapper::convertTFtoCameraInfo(const tf::Transform& transform) {
//-- Function to convert a tf-transformation to the data type CameraInfo (defined in camerautils.hpp in fastfusion)
//-- The intrinsics are assumed to be constant and are loaded from a parameter file.
	//TODO: Why are the images never undistorted??
	CameraInfo result;
	result.setIntrinsic(intrinsic_);
	//-- Convert Message (first convert to eigen)
	Eigen::Quaterniond q;
	Eigen::Matrix3d R;
	Eigen::Vector3d c;
 	tf::vectorTFToEigen(transform.getOrigin(), c);
 	tf::quaternionTFToEigen(transform.getRotation(), q);
 	R = q.toRotationMatrix();

	//-- Convert Eigen Matrices to member type of CameraInfo
	cv::Mat rotation = cv::Mat::eye(3,3,cv::DataType<double>::type);
	for(int i=0;i<3;i++) {
		for(int j=0;j<3;j++) { 
			rotation.at<double>(i,j) = R(i,j);
		}
	}
	result.setRotation(rotation);
	cv::Mat translation = cv::Mat::zeros(3,1,cv::DataType<double>::type);
	for(int i=0;i<3;i++) {
		translation.at<double>(i,0) = c(i);
	}
	result.setTranslation(translation);
	//-- Return the camera intrinsics/extrinsics in the CameraInfo format
	return result;
}
