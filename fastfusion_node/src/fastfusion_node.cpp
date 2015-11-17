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
	intrinsic_ = cv::Mat::eye(3,3,cv::DataType<double>::type);
	distCoeff_ = cv::Mat::zeros(5,1,cv::DataType<double>::type);

	bool loadSuccess = true;
	bool threadFusion, threadMeshing, saveMesh;
	std::string fileLocation;
	int depthChecks;
	double imageScale,scale,threshold;
	loadSuccess &= nodeLocal_.getParam("threadFusion", threadFusion);
	loadSuccess &= nodeLocal_.getParam("threadMeshing",threadMeshing);
	loadSuccess &= nodeLocal_.getParam("imageScale",imageScale);
	loadSuccess &= nodeLocal_.getParam("scale",scale);
	loadSuccess &= nodeLocal_.getParam("threshold",threshold);
	/*
	loadSuccess &= nodeLocal_.getParam("fx", intrinsic_.at<double>(0,0));
	loadSuccess &= nodeLocal_.getParam("fy", intrinsic_.at<double>(1,1));
	loadSuccess &= nodeLocal_.getParam("cx", intrinsic_.at<double>(0,2));
	loadSuccess &= nodeLocal_.getParam("cy", intrinsic_.at<double>(1,2));
	*/
	loadSuccess &= nodeLocal_.getParam("depthConsistencyChecks", depthChecks);
	loadSuccess &= nodeLocal_.getParam("saveMesh",saveMesh);
	loadSuccess &= nodeLocal_.getParam("fileLocation",fileLocation);
	loadSuccess &= nodeLocal_.getParam("world_id",world_id_);
	loadSuccess &= nodeLocal_.getParam("cam_id",cam_id_);
	loadSuccess &= nodeLocal_.getParam("use_pmd",use_pmd_);

	//-- Read the camera poses
	XmlRpc::XmlRpcValue T_cam0_imu;
	loadSuccess &= nodeLocal_.getParam("cam0/T_cam_imu", T_cam0_imu);
	R_cam0_imu(0, 0) = (double) T_cam0_imu[0][0];
	R_cam0_imu(0, 1) = (double) T_cam0_imu[0][1];
	R_cam0_imu(0, 2) = (double) T_cam0_imu[0][2];
	R_cam0_imu(1, 0) = (double) T_cam0_imu[1][0];
	R_cam0_imu(1, 1) = (double) T_cam0_imu[1][1];
	R_cam0_imu(1, 2) = (double) T_cam0_imu[1][2];
	R_cam0_imu(2, 0) = (double) T_cam0_imu[2][0];
	R_cam0_imu(2, 1) = (double) T_cam0_imu[2][1];
	R_cam0_imu(2, 2) = (double) T_cam0_imu[2][2];
	t_cam0_imu(0) = (double) T_cam0_imu[0][3];
	t_cam0_imu(1) = (double) T_cam0_imu[1][3];
	t_cam0_imu(2) = (double) T_cam0_imu[2][3];
	t_cam0_imu = -R_cam0_imu.transpose()*t_cam0_imu;
	tf_cam0_imu.setOrigin(tf::Vector3(t_cam0_imu(0), t_cam0_imu(1), t_cam0_imu(2)));
	Eigen::Quaterniond q_cam0_imu(R_cam0_imu.transpose());
	tf_cam0_imu.setRotation(tf::Quaternion(q_cam0_imu.x(), q_cam0_imu.y(), q_cam0_imu.z(), q_cam0_imu.w()));
	if (use_pmd_) {
		XmlRpc::XmlRpcValue T_depth_cam0;
		loadSuccess &= nodeLocal_.getParam("depth/T_depth_cam0", T_depth_cam0);
		R_depth_cam0(0, 0) = (double) T_depth_cam0[0][0];
		R_depth_cam0(0, 1) = (double) T_depth_cam0[0][1];
		R_depth_cam0(0, 2) = (double) T_depth_cam0[0][2];
		R_depth_cam0(1, 0) = (double) T_depth_cam0[1][0];
		R_depth_cam0(1, 1) = (double) T_depth_cam0[1][1];
		R_depth_cam0(1, 2) = (double) T_depth_cam0[1][2];
		R_depth_cam0(2, 0) = (double) T_depth_cam0[2][0];
		R_depth_cam0(2, 1) = (double) T_depth_cam0[2][1];
		R_depth_cam0(2, 2) = (double) T_depth_cam0[2][2];
		t_depth_cam0(0) = (double) T_depth_cam0[0][3];
		t_depth_cam0(1) = (double) T_depth_cam0[1][3];
		t_depth_cam0(2) = (double) T_depth_cam0[2][3];
		t_depth_cam0 = -R_depth_cam0.transpose()*t_depth_cam0;
		tf_depth_cam0.setOrigin(tf::Vector3(t_depth_cam0(0), t_depth_cam0(1), t_depth_cam0(2)));
		Eigen::Quaterniond q_depth_cam0(R_depth_cam0.transpose());
		tf_depth_cam0.setRotation(tf::Quaternion(q_depth_cam0.x(), q_depth_cam0.y(), q_depth_cam0.z(), q_depth_cam0.w()));
	} else {
		XmlRpc::XmlRpcValue T_rgb_cam0;
		loadSuccess &= nodeLocal_.getParam("rgb/T_rgb_cam0", T_rgb_cam0);
		R_rgb_cam0(0, 0) = (double) T_rgb_cam0[0][0];
		R_rgb_cam0(0, 1) = (double) T_rgb_cam0[0][1];
		R_rgb_cam0(0, 2) = (double) T_rgb_cam0[0][2];
		R_rgb_cam0(1, 0) = (double) T_rgb_cam0[1][0];
		R_rgb_cam0(1, 1) = (double) T_rgb_cam0[1][1];
		R_rgb_cam0(1, 2) = (double) T_rgb_cam0[1][2];
		R_rgb_cam0(2, 0) = (double) T_rgb_cam0[2][0];
		R_rgb_cam0(2, 1) = (double) T_rgb_cam0[2][1];
		R_rgb_cam0(2, 2) = (double) T_rgb_cam0[2][2];
		t_rgb_cam0(0) = (double) T_rgb_cam0[0][3];
		t_rgb_cam0(1) = (double) T_rgb_cam0[1][3];
		t_rgb_cam0(2) = (double) T_rgb_cam0[2][3];
		t_rgb_cam0 = -R_rgb_cam0.transpose()*t_rgb_cam0;
		tf_rgb_cam0.setOrigin(tf::Vector3(t_rgb_cam0(0), t_rgb_cam0(1), t_rgb_cam0(2)));
		Eigen::Quaterniond q_rgb_cam0(R_rgb_cam0.transpose());
		tf_rgb_cam0.setRotation(tf::Quaternion(q_rgb_cam0.x(), q_rgb_cam0.y(), q_rgb_cam0.z(), q_rgb_cam0.w()));
	}

	//-- Camera Intrinsics
	XmlRpc::XmlRpcValue intrinsics_cam0, intrinsics_cam1, intrinsics_depth;
	loadSuccess &= nodeLocal_.getParam("cam0/intrinsics", intrinsics_cam0);
	loadSuccess &= nodeLocal_.getParam("cam1/intrinsics", intrinsics_cam1);
	if (use_pmd_) {
		loadSuccess &= nodeLocal_.getParam("depth/intrinsics", intrinsics_depth);
	} else {
		XmlRpc::XmlRpcValue intrinsics_rgb;
		loadSuccess &= nodeLocal_.getParam("rgb/intrinsics", intrinsics_rgb);
		intrinsicRGB_ = cv::Mat::eye(3,3,cv::DataType<double>::type);
		intrinsicRGB_.at<double>(0,0) = (double)intrinsics_rgb[0];
		intrinsicRGB_.at<double>(1,1) = (double)intrinsics_rgb[1];
		intrinsicRGB_.at<double>(0,2) = (double)intrinsics_rgb[2];
		intrinsicRGB_.at<double>(1,2) = (double)intrinsics_rgb[3];
	}
	intrinsic_.at<double>(0,0) = (double)intrinsics_depth[0];
	intrinsic_.at<double>(1,1) = (double)intrinsics_depth[1];
	intrinsic_.at<double>(0,2) = (double)intrinsics_depth[2];
	intrinsic_.at<double>(1,2) = (double)intrinsics_depth[3];
	XmlRpc::XmlRpcValue distortion_cam0, distortion_cam1, distortion_depth;
	loadSuccess &= nodeLocal_.getParam("cam0/distortion_coeffs", distortion_cam0);
	loadSuccess &= nodeLocal_.getParam("cam1/distortion_coeffs", distortion_cam1);
	loadSuccess &= nodeLocal_.getParam("depth/distortion_coeffs", distortion_depth);
	distCoeff_.at<double>(0,0) = (double)distortion_depth[0];
	distCoeff_.at<double>(1,0) = (double)distortion_depth[1];
	distCoeff_.at<double>(2,0) = (double)distortion_depth[2];
	distCoeff_.at<double>(3,0) = (double)distortion_depth[3];
	distCoeff_.at<double>(4,0) = (double)distortion_depth[4];

	if (loadSuccess) {
		ROS_INFO("\nFastfusion: Could read the parameters.\n");
		onlinefusion_.setupFusion(threadFusion, threadMeshing,(float) imageScale, (float) scale, (float) threshold, depthChecks,
				saveMesh, fileLocation);

	} else {
		ROS_ERROR("\nFastfusion: Could not read parameters, abort.\n");
	}
	while (!onlinefusion_.isSetup()){
		//-- Waiting for onlinefusion to be setup
	}
	testing_point_cloud_ = false;
}

void FastFusionWrapper::run() {
	ROS_INFO("\nRun Fastfusion .....");
	if (use_pmd_) {
		//-- Subscribe to depth image callback
		image_transport::ImageTransport it(node_);
		subscriberOnlyDepth_ = new image_transport::Subscriber;
		*subscriberOnlyDepth_ =	it.subscribe(node_.resolveName("image_depth"), 5, &FastFusionWrapper::imageCallbackPico, this);

	} else {
		//-- Synchronize the image messages received from Realsense Sensor
		subscriberDepth_ = new message_filters::Subscriber<sensor_msgs::Image>;
		subscriberRGB_ = new message_filters::Subscriber<sensor_msgs::Image>;

		subscriberDepth_->subscribe(node_,node_.resolveName("image_depth"),5);
		subscriberRGB_->subscribe(node_,node_.resolveName("image_rgb"),5);

		sync_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> >
		(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(5),*subscriberRGB_,*subscriberDepth_);
		sync_->registerCallback(boost::bind(&FastFusionWrapper::imageCallback, this,  _1,  _2));
	}

	//ros::Subscriber subscriberPCL = node_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 5, &FastFusionWrapper::pclCallback,this);
	std::cout << "Start Spinning" << std::endl;
	ros::spin();
	//-- Stop the fusion process
	onlinefusion_.stop();
	ros::shutdown();
}

void FastFusionWrapper::imageCallbackPico(const sensor_msgs::ImageConstPtr& msgDepth) {
	if ((msgDepth->header.stamp - previous_ts_).toSec() <= 0.05){
		return;
	}
	cv::Mat imgDepthDist, imgDepth;
	ros::Time timeMeas;
	//-- Convert the incomming messages
	getDepthImageFromRosMsg(msgDepth, &imgDepthDist);
	//-- Undistort the depth image
	cv::undistort(imgDepthDist, imgDepth, intrinsic_, distCoeff_);
	cv::imshow("Test",imgDepth);
	cv::waitKey(5);
	//imgDepth = imgDepthDist;
	//-- Compute Point Cloud for testing
	if (testing_point_cloud_) {
		cv::imwrite("/home/karrer/imgDepth.png",imgDepth);
		cv::imwrite("/home/karrer/imgDepthDist.png",imgDepthDist);
		float fx = (float)intrinsic_.at<double>(0,0);
		float fy = (float)intrinsic_.at<double>(1,1);
		float cx = (float)intrinsic_.at<double>(0,2);
		float cy = (float)intrinsic_.at<double>(1,2);
		pcl::PointXYZ tempPoint;
		pcl::PointCloud<pcl::PointXYZ> pointCloud;
		for (int i = 0; i < imgDepth.rows; i++) {
			for (int j = 0; j < imgDepth.cols; j++) {
				tempPoint.z = (float)imgDepth.at<unsigned short>(i,j)/1000.0f;
				tempPoint.x = (float)(j-cx)/fx*tempPoint.z;
				tempPoint.y = (float)(i-cy)/fy*tempPoint.z;
				pointCloud.push_back(tempPoint);
			}
		}
		pcl::io::savePLYFile ("/home/karrer/PointCloudImg.ply", pointCloud, false);
		pcl::io::savePCDFile ("/home/karrer/PointCloudImg.pcd", pointCloud, false);
		pointCloud.clear();
	}
	// Create Dummy RGB Frame
	cv::Mat imgRGB(imgDepth.rows, imgDepth.cols, CV_8UC3, CV_RGB(200,200,200));
	//-- Get time stamp of the incoming images
	ros::Time timestamp = msgDepth->header.stamp;
	broadcastTFchain(timestamp);
	previous_ts_ = timestamp;
	//-- Get Pose (tf-listener)
	tf::StampedTransform transform;
	try{
		ros::Time nowTime = ros::Time::now();
		tfListener.waitForTransform(world_id_,cam_id_,
				timestamp, ros::Duration(2.0));
		tfListener.lookupTransform(world_id_, cam_id_,
				timestamp, transform);
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
}

void FastFusionWrapper::imageCallback(const sensor_msgs::ImageConstPtr& msgRGB, 
										const sensor_msgs::ImageConstPtr& msgDepth) {
//-- Callback function to receive depth image with corresponding RGB frame as ROS-Messages
//-- Convert the messages to cv::Mat and wait for tf-transform corresponding to the frames
//-- Push the data into the fastfusion pipeline for processing.
	if ((msgRGB->header.stamp - previous_ts_).toSec() <= 0.03){
		return;
	}

	cv::Mat imgRGB, imgDepth;
	ros::Time timeMeas;
	//-- Convert the incomming messagesb
	getRGBImageFromRosMsg(msgRGB, &imgRGB);
	getDepthImageFromRosMsg(msgDepth, &imgDepth);
	if (testing_point_cloud_) {
		cv::imwrite("/home/karrer/imgDepthRealsense.png",imgDepth);
		cv::imwrite("/home/karrer/imgRGB.png",imgRGB);
	}
	//-- Get time stamp of the incoming images
	ros::Time timestamp = msgRGB->header.stamp;
	broadcastTFchain(timestamp);
	previous_ts_ = timestamp;
	//-- Get Pose (tf-listener)
	tf::StampedTransform transform;
	try{
		ros::Time nowTime = ros::Time::now();
		tfListener.waitForTransform(world_id_,cam_id_,
                    				timestamp, ros::Duration(2.0));
		tfListener.lookupTransform(world_id_, cam_id_,
									timestamp, transform);
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
}


void FastFusionWrapper::pclCallback(const sensor_msgs::PointCloud2 pcl_msg) {
	pcl::PointCloud<pcl::PointXYZRGB>  pcl_cloud;
	pcl::fromROSMsg (pcl_msg,pcl_cloud);
	if (testing_point_cloud_) {
		pcl::io::savePLYFile ("/home/karrer/PointCloudRealsense.ply", pcl_cloud, false);
		pcl::io::savePCDFile ("/home/karrer/PointCloudRealsense.pcd", pcl_cloud, false);

	}
}

void FastFusionWrapper::getRGBImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgRGB, cv::Mat *rgbImg) {
//-- Function to convert ROS-image (RGB) message to OpenCV-Mat.
	cv::Mat rgbImg2 = cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8)->image;
	//*rgbImg= cv_bridge::toCvCopy(msgRGB, sensor_msgs::image_encodings::BGR8)->image;
	cv::resize(rgbImg2,*rgbImg,cv::Size(480,360),cv::INTER_LINEAR);
}

void FastFusionWrapper::getDepthImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgDepth, cv::Mat *depthImg) {
//-- Function to convert ROS-image(depth) message to OpenCV-Mat.
	*depthImg = cv_bridge::toCvCopy(msgDepth, sensor_msgs::image_encodings::TYPE_16UC1)->image;
	//cv::Mat depthImg2 = cv_bridge::toCvCopy(msgDepth,sensor_msgs::image_encodings::TYPE_16UC1)->image;
	//depthImg2 = depthImg2;
	//depthImg2.convertTo(*depthImg,CV_16UC1);
	//cv::imwrite("/home/karrer/depth.png",*depthImg);
	//-- Convert the image to 16 bit (fastfusion requires 16 bit depth images)
	//depthImg2.convertTo(*depthImg,CV_16UC1);
	//*depthImg = *depthImg*5000;

}

void FastFusionWrapper::broadcastTFchain(ros::Time timestamp) {
	if (use_pmd_) {
		tfBroadcaster_.sendTransform(tf::StampedTransform(tf_depth_cam0, timestamp, "cam0", "depth"));
	} else {
		tfBroadcaster_.sendTransform(tf::StampedTransform(tf_rgb_cam0, timestamp, "cam0", "camera_color_optical_frame"));
	}
	tfBroadcaster_.sendTransform(tf::StampedTransform(tf_cam0_imu, timestamp, "camera_imu", "cam0"));
}


CameraInfo FastFusionWrapper::convertTFtoCameraInfo(const tf::Transform& transform) {
//-- Function to convert a tf-transformation to the data type CameraInfo (defined in camerautils.hpp in fastfusion)
//-- The intrinsics are assumed to be constant and are loaded from a parameter file.
	CameraInfo result;
	result.setIntrinsic(intrinsic_);

	//-- Convert Message (first convert to eigen)
	Eigen::Quaterniond q;
	Eigen::Matrix3d R_temp,R;
	Eigen::Vector3d c_temp, c;
 	tf::vectorTFToEigen(transform.getOrigin(), c);
 	//c = R_depth_cam0.transpose()*( R_cam0_imu.transpose()*(c_temp - t_cam0_imu)-t_depth_cam0);
 	tf::quaternionTFToEigen(transform.getRotation(), q);
 	R = q.toRotationMatrix();
 	//R = R_depth_cam0*R_cam0_imu*R_temp;

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
	onlinefusion_.cameraCenter_.x = c(0);
	onlinefusion_.cameraCenter_.y = c(1);
	onlinefusion_.cameraCenter_.z = c(2);

	//-- Return the camera intrinsics/extrinsics in the CameraInfo format
	return result;
}
