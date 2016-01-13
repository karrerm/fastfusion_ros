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
	intrinsicRGB_ = cv::Mat::eye(3,3,cv::DataType<double>::type);
	distCoeff_ = cv::Mat::zeros(5,1,cv::DataType<double>::type);
	distCoeffRGB_ = cv::Mat::zeros(5,1,cv::DataType<double>::type);
	depthCorrection_ = cv::Mat::zeros(6,1,cv::DataType<double>::type);
	bool loadSuccess = true;
	bool threadFusion, threadMeshing, saveMesh;
	std::string fileLocation;
	int depthChecks;
	double scale,distThreshold;
	//-- Fastfusion Parameters
	loadSuccess &= nodeLocal_.getParam("threadFusion", threadFusion);
	loadSuccess &= nodeLocal_.getParam("threadMeshing",threadMeshing);
	loadSuccess &= nodeLocal_.getParam("imageScale",imageScale_);
	loadSuccess &= nodeLocal_.getParam("scale",scale);
	loadSuccess &= nodeLocal_.getParam("distanceThreshold",distThreshold);
	loadSuccess &= nodeLocal_.getParam("depthConsistencyChecks", depthChecks);
	//-- Additional Parameters
	loadSuccess &= nodeLocal_.getParam("saveMesh",saveMesh); 					// Save the mesh when exiting
	loadSuccess &= nodeLocal_.getParam("fileLocation",fileLocation);			// Location for mesh save
	loadSuccess &= nodeLocal_.getParam("world_id",world_id_);					// Frame id of origin
	loadSuccess &= nodeLocal_.getParam("cam_id",cam_id_);						// Frame id of depth camera
	loadSuccess &= nodeLocal_.getParam("use_pmd",use_pmd_);						// Use ToF of RGBD
	std::cout << "Parameters: " << loadSuccess << std::endl;
	//-- Read Camera to IMU transformation
	XmlRpc::XmlRpcValue T_cam0_imu;
	loadSuccess &= nodeLocal_.getParam("cam0/T_cam_imu", T_cam0_imu);
	std::cout << "T_cam0_imu: " << loadSuccess << std::endl;
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
	t_cam0_imu = t_cam0_imu;
	Eigen::Quaterniond q_cam0_imu(R_cam0_imu);
	tf_cam0_imu.setOrigin(tf::Vector3(t_cam0_imu(0), t_cam0_imu(1), t_cam0_imu(2)));
	tf_cam0_imu.setRotation(tf::Quaternion(q_cam0_imu.x(), q_cam0_imu.y(), q_cam0_imu.z(), q_cam0_imu.w()));
	//-- Read camera to depth camera transformation
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
		std::cout << R_depth_cam0 << std::endl;
		tf_depth_cam0.setRotation(tf::Quaternion(q_depth_cam0.x(), q_depth_cam0.y(), q_depth_cam0.z(), q_depth_cam0.w()));
		//-- Depth Correction
		XmlRpc::XmlRpcValue depthCorrection;
		loadSuccess &= nodeLocal_.getParam("depth/depth_correction", depthCorrection);
		depthCorrection_.at<double>(0,0) = (double)depthCorrection[0];
		depthCorrection_.at<double>(1,0) = (double)depthCorrection[1];
		depthCorrection_.at<double>(2,0) = (double)depthCorrection[2];
		depthCorrection_.at<double>(3,0) = (double)depthCorrection[3];
		depthCorrection_.at<double>(4,0) = (double)depthCorrection[4];
		depthCorrection_.at<double>(5,0) = (double)depthCorrection[5];
	} else {
		XmlRpc::XmlRpcValue T_rgb_cam0;
		loadSuccess &= nodeLocal_.getParam("rgb/T_rgb_cam0", T_rgb_cam0);
		std::cout << "T_rgb_cam0: " << loadSuccess << std::endl;
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
		t_rgb_cam0 = t_rgb_cam0;
		tf_rgb_cam0.setOrigin(tf::Vector3(t_rgb_cam0(0), t_rgb_cam0(1), t_rgb_cam0(2)));
		Eigen::Quaterniond q_rgb_cam0(R_rgb_cam0);
		tf_rgb_cam0.setRotation(tf::Quaternion(q_rgb_cam0.x(), q_rgb_cam0.y(), q_rgb_cam0.z(), q_rgb_cam0.w()));
	}
	//-- Read vicon body to camera transformation
	XmlRpc::XmlRpcValue T_body_cam;
	loadSuccess &= nodeLocal_.getParam("vicon/T_vicon_cam", T_body_cam);
	R_body_cam(0, 0) = (double) T_body_cam[0][0];
	R_body_cam(0, 1) = (double) T_body_cam[0][1];
	R_body_cam(0, 2) = (double) T_body_cam[0][2];
	R_body_cam(1, 0) = (double) T_body_cam[1][0];
	R_body_cam(1, 1) = (double) T_body_cam[1][1];
	R_body_cam(1, 2) = (double) T_body_cam[1][2];
	R_body_cam(2, 0) = (double) T_body_cam[2][0];
	R_body_cam(2, 1) = (double) T_body_cam[2][1];
	R_body_cam(2, 2) = (double) T_body_cam[2][2];
	t_body_cam(0) = (double) T_body_cam[0][3];
	t_body_cam(1) = (double) T_body_cam[1][3];
	t_body_cam(2) = (double) T_body_cam[2][3];
	t_body_cam = t_body_cam;
	tf_body_cam.setOrigin(tf::Vector3(t_body_cam(0), t_body_cam(1), t_body_cam(2)));
	Eigen::Quaterniond q_body_cam(R_body_cam);
	tf_body_cam.setRotation(tf::Quaternion(q_body_cam.x(), q_body_cam.y(), q_body_cam.z(), q_body_cam.w()));

	//-- Camera Intrinsics
	XmlRpc::XmlRpcValue intrinsics_cam0, intrinsics_cam1, intrinsics_depth;
	loadSuccess &= nodeLocal_.getParam("cam0/intrinsics", intrinsics_cam0);
	//loadSuccess &= nodeLocal_.getParam("cam1/intrinsics", intrinsics_cam1);
	loadSuccess &= nodeLocal_.getParam("depth/intrinsics", intrinsics_depth);
	intrinsic_.at<double>(0,0) = (double)intrinsics_depth[0];
	intrinsic_.at<double>(1,1) = (double)intrinsics_depth[1];
	intrinsic_.at<double>(0,2) = (double)intrinsics_depth[2];
	intrinsic_.at<double>(1,2) = (double)intrinsics_depth[3];
	XmlRpc::XmlRpcValue distortion_cam0, distortion_cam1, distortion_depth;
	loadSuccess &= nodeLocal_.getParam("cam0/distortion_coeffs", distortion_cam0);
	//loadSuccess &= nodeLocal_.getParam("cam1/distortion_coeffs", distortion_cam1);
	loadSuccess &= nodeLocal_.getParam("depth/distortion_coeffs", distortion_depth);
	distCoeff_.at<double>(0,0) = (double)distortion_depth[0];
	distCoeff_.at<double>(1,0) = (double)distortion_depth[1];
	distCoeff_.at<double>(2,0) = (double)distortion_depth[2];
	distCoeff_.at<double>(3,0) = (double)distortion_depth[3];
	distCoeff_.at<double>(4,0) = (double)distortion_depth[4];
	//-- Read depth camera intrinsics
	if (!use_pmd_) {
		XmlRpc::XmlRpcValue intrinsics_rgb, distortion_rgb;
		loadSuccess &= nodeLocal_.getParam("rgb/intrinsics", intrinsics_rgb);
		intrinsicRGB_ = cv::Mat::eye(3,3,cv::DataType<double>::type);
		intrinsicRGB_.at<double>(0,0) = (double)intrinsics_rgb[0];
		intrinsicRGB_.at<double>(1,1) = (double)intrinsics_rgb[1];
		intrinsicRGB_.at<double>(0,2) = (double)intrinsics_rgb[2];
		intrinsicRGB_.at<double>(1,2) = (double)intrinsics_rgb[3];
		loadSuccess &= nodeLocal_.getParam("rgb/distortion_coeffs", distortion_rgb);
		distCoeffRGB_.at<double>(0,0) = (double)distortion_rgb[0];
		distCoeffRGB_.at<double>(1,0) = (double)distortion_rgb[1];
		distCoeffRGB_.at<double>(2,0) = (double)distortion_rgb[2];
		distCoeffRGB_.at<double>(3,0) = (double)distortion_rgb[3];
		distCoeffRGB_.at<double>(4,0) = (double)distortion_rgb[4];
	}
	if (loadSuccess) {
		ROS_INFO("\nFastfusion: Could read the parameters.\n");
		//-- Configure fastfusion framework
		onlinefusion_.setupFusion(threadFusion, threadMeshing,(float) imageScale_, (float) scale, (float) distThreshold, depthChecks,
				saveMesh, fileLocation);
	} else {
		ROS_ERROR("\nFastfusion: Could not read parameters, abort.\n");
	}
	while (!onlinefusion_.isSetup()){
		//-- Waiting for onlinefusion to be setup
	}
	//cv::waitKey(10000);
	testing_point_cloud_ = false;
}

void FastFusionWrapper::run() {
	ROS_INFO("\nRun Fastfusion .....");
	if (use_pmd_) {
		//-- Subscribe to depth image callback
		subscriberDepth_ = new message_filters::Subscriber<sensor_msgs::Image>;
		subscriberConfidence_ = new message_filters::Subscriber<sensor_msgs::Image>;
		subscriberNoise_ = new message_filters::Subscriber<sensor_msgs::Image>;
		subscriberDepth_->subscribe(node_,node_.resolveName("image_depth"),5);
		subscriberConfidence_->subscribe(node_,node_.resolveName("image_conf"),5);
		subscriberNoise_->subscribe(node_,node_.resolveName("image_noise"),5);
		sync_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> >
			(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>(5),
					*subscriberDepth_,*subscriberConfidence_, *subscriberNoise_);
		sync_->registerCallback(boost::bind(&FastFusionWrapper::imageCallbackPico, this,  _1,  _2, _3));
	} else {
		//-- Synchronize the image messages received from Realsense Sensor
		/*
		subscriberDepth_ = new message_filters::Subscriber<sensor_msgs::Image>;
		subscriberRGB_ = new message_filters::Subscriber<sensor_msgs::Image>;

		subscriberDepth_->subscribe(node_,node_.resolveName("image_depth"),5);
		subscriberRGB_->subscribe(node_,node_.resolveName("image_rgb"),5);

		sync_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> >
		(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>(5),*subscriberRGB_,*subscriberDepth_);
		sync_->registerCallback(boost::bind(&FastFusionWrapper::imageCallback, this,  _1,  _2));
		*/
	}

	ros::Subscriber subscriberPCL = node_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 5, &FastFusionWrapper::registerPointCloudCallback,this);
	std::cout << "Start Spinning" << std::endl;
	ros::spin();
	//-- Stop the fusion process
	onlinefusion_.stop();
	pcl::io::savePLYFile ("/home/karrer/PointCloudImg.ply", pointCloudFrameTrans_, false);
	ros::shutdown();
}


void FastFusionWrapper::imageCallbackPico(const sensor_msgs::ImageConstPtr& msgDepth,
										  const sensor_msgs::ImageConstPtr& msgConf,
										  const sensor_msgs::ImageConstPtr& msgNoise) {
	if ((msgDepth->header.stamp - previous_ts_).toSec() <= 0.05){
		return;
	}
//-- Callbackfunction for the use of the ToF camera. Assumes 16 bit input depth image.
//-- The depth image is undistorted according to the intrinsics of the depth camera.
//--
	cv::Mat imgDepthDist, imgDepth, imgConfDist, imgConf;
	ros::Time timeMeas;
	//-- Convert the incomming messages
	getDepthImageFromRosMsg(msgDepth, &imgDepthDist);
	getConfImageFromRosMsg(msgConf, &imgConfDist);
	//-- Undistort the depth image
	cv::undistort(imgDepthDist, imgDepth, intrinsic_, distCoeff_);
	cv::undistort(imgConfDist, imgConf, intrinsic_, distCoeff_);

	cv::Mat imgDepthCorr = cv::Mat::zeros(imgDepth.rows,imgDepth.cols,cv::DataType<unsigned short>::type);
	depthImageCorrection(imgDepthDist, &imgDepthCorr);
	for (int u = 0; u < imgDepthCorr.cols; u++) {
		for (int v = 0; v < imgDepthCorr.rows; v++) {
			if (imgConf.at<unsigned char>(v,u)!= 255) {
				imgDepthCorr.at<unsigned short>(v,u) = 65000;
			}
		}
	}
	//imgDepth = imgDepthDist;
	// Create Dummy RGB Frame
	cv::Mat imgRGB(imgDepthCorr.rows, imgDepthCorr.cols, CV_8UC3, CV_RGB(200,200,200));

	//-- Get time stamp of the incoming images
	ros::Time timestamp = msgDepth->header.stamp;
	broadcastTFchain(timestamp);
	previous_ts_ = timestamp;
	//-- Get Pose (tf-listener)
	tf::StampedTransform transform;
	try{
		ros::Time nowTime = ros::Time::now();
		tfListener.waitForTransform(world_id_, cam_id_,
				timestamp, ros::Duration(2.0));
		tfListener.lookupTransform(world_id_, cam_id_,
				timestamp, transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}
	//-- Convert tf to CameraInfo (fastfusion Class in camerautils.hpp)
	CameraInfo incomingFramePose;
	incomingFramePose = convertTFtoCameraInfo(transform);

	//-- Compute Point Cloud for testing
	if (testing_point_cloud_) {
		//cv::imwrite("/home/karrer/imgDepth.png",imgDepth);
		//cv::imwrite("/home/karrer/imgDepthDist.png",imgDepthDist);
		Eigen::Quaterniond q;
		Eigen::Matrix3d R;
		Eigen::Vector3d c;
		tf::vectorTFToEigen(transform.getOrigin(), c);
		//c = R_depth_cam0.transpose()*( R_cam0_imu.transpose()*(c_temp - t_cam0_imu)-t_depth_cam0);
		tf::quaternionTFToEigen(transform.getRotation(), q);
		R = q.toRotationMatrix();
		Eigen::Matrix4d transformMat = Eigen::Matrix4d::Identity();
		transformMat(0,0) = R(0,0); transformMat(0,1) = R(0,1); transformMat(0,2) = R(0,2); transformMat(0,3) = c(0);
		transformMat(1,0) = R(1,0); transformMat(1,1) = R(1,1); transformMat(1,2) = R(1,2); transformMat(1,3) = c(1);
		transformMat(2,0) = R(2,0); transformMat(2,1) = R(2,1); transformMat(2,2) = R(2,2); transformMat(2,3) = c(2);

		float fx = (float)intrinsic_.at<double>(0,0);
		float fy = (float)intrinsic_.at<double>(1,1);
		float cx = (float)intrinsic_.at<double>(0,2);
		float cy = (float)intrinsic_.at<double>(1,2);
		pcl::PointXYZRGB tempPoint;
		pointCloudFrame_.clear();
		pointCloudFrameTrans_.clear();
		for (int i = 0; i < imgDepthCorr.rows; i++) {
			for (int j = 0; j < imgDepthCorr.cols; j++) {
				if (imgDepthCorr.at<unsigned short>(i,j) > 0) {
					tempPoint.z = (float)imgDepthCorr.at<unsigned short>(i,j)/5000.0f;
					tempPoint.x = (float)(j-cx)/fx*tempPoint.z;
					tempPoint.y = (float)(i-cy)/fy*tempPoint.z;
					tempPoint.g = 255;
					tempPoint.r = tempPoint.b = 0;
					pointCloudFrame_.push_back(tempPoint);
				}
			}
		}
		pcl::transformPointCloud (pointCloudFrame_, pointCloudFrameTrans_, transformMat);
		//pcl::io::savePLYFile ("/home/karrer/PointCloudImg.ply", pointCloud, false);
		//pcl::io::savePCDFile ("/home/karrer/PointCloudImg.pcd", pointCloud, false);
		//pointCloud.clear();
	}




	//-- Fuse the imcoming Images into existing map
	onlinefusion_.updateFusion(imgRGB, imgDepthCorr, incomingFramePose);
}


void FastFusionWrapper::depthImageCorrection(cv::Mat & imgDepth, cv::Mat * imgDepthCorrected) {
//-- Function to apply the polynomial depth correction from the ToF calibration using the MIP toolbox.
	const double fx = intrinsic_.at<double>(0,0);
	const double fy = intrinsic_.at<double>(1,1);
	const double cx = intrinsic_.at<double>(0,2);
	const double cy = intrinsic_.at<double>(1,2);
	const double d0 = depthCorrection_.at<double>(0,0);
	const double d1 = depthCorrection_.at<double>(1,0);
	const double d2 = depthCorrection_.at<double>(2,0);
	const double d3 = depthCorrection_.at<double>(3,0);
	const double d4 = depthCorrection_.at<double>(4,0);
	const double d5 = depthCorrection_.at<double>(5,0);

	double x,y;
	double lambda, depthCorrected;
	//unsigned short *depthPtr;
	//unsigned short *depthCorrPtr;
	for (int v = 0; v < imgDepth.rows; v++) {
		//depthPtr = imgDepth.ptr<unsigned short>(v);
		//depthCorrPtr = imgDepthCorrected->ptr<unsigned short>(v);
		for (int u = 0; u < imgDepth.cols; u++) {
			x = (((double) u) - cx)/fx;
			y = (((double) v) - cy)/fy;
			//lambda = ((double) depthPtr[u])/imageScale_*1000.0; 		// Parameters assume depth in [mm]
			lambda = ((double) imgDepth.at<unsigned short>(v,u))*std::sqrt(x*x + y*y + 1)/imageScale_;
			depthCorrected = d0 + (1.0 + d1)*lambda + d2*x + d3*y + d4*lambda*lambda + d5*lambda*lambda*lambda;
			depthCorrected = depthCorrected*imageScale_/std::sqrt(x*x + y*y + 1);			// Conversion back to scaled uint16
			imgDepthCorrected->at<unsigned short>(v,u) = (unsigned short) depthCorrected;
			//depthCorrPtr[v] = (unsigned short) depthCorrected;
		}
	}
	//cv::imshow("imgDepth",imgDepth); cv::waitKey(1);
	//cv::imshow("imgDepthCorr",*imgDepthCorrected); cv::waitKey(1);
}


void FastFusionWrapper::registerPointCloudCallback(const sensor_msgs::PointCloud2 pcl_msg) {
	if ((pcl_msg.header.stamp - previous_ts_).toSec() <= 0.05){
			return;
	}
	std::cout << "in registerPointCloudCallback" << std::endl;
	std::cout << "size of unsigned short = " << sizeof(unsigned short) << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB>  pcl_cloud;
	pcl::fromROSMsg (pcl_msg,pcl_cloud);
	//-- Create RGB and Depth image from point cloud
	unsigned int width = pcl_msg.width;
	unsigned int height = pcl_msg.height;
	cv::Mat imgDepth(height,width,CV_16UC1);
	cv::Mat imgRGB(height, width,CV_8UC3);
	unsigned int indexRGB = 0;
	unsigned int indexDepth = 0;
	for (unsigned int v = 0; v < height; v++) {
		for (unsigned int u = 0; u < width; u++) {
			indexRGB = 3*(width*v + u);
			indexDepth = width*v + u;
			imgRGB.data[indexRGB + 0] = pcl_cloud.points[indexDepth].b;
			imgRGB.data[indexRGB + 1] = pcl_cloud.points[indexDepth].g;
			imgRGB.data[indexRGB + 2] = pcl_cloud.points[indexDepth].r;
			if (pcl_cloud.points[indexDepth].z > 0) {
				imgDepth.at<unsigned short>(v,u) = (unsigned short)(pcl_cloud.points[indexDepth].z*1000.0f);
				//imgDepth.data[indexDepth] = (float)(pcl_cloud.points[indexDepth].z);
			} else {
				imgDepth.at<unsigned short>(v,u) = (unsigned short)0.0;
			}
		}
	}
	//cv::imwrite("/home/karrer/Desktop/depht.png",imgDepth);
	//cv::imshow("RegCallback", imgDepth);
	//cv::waitKey(5);
	//-- Get time stamp of the incoming images
	ros::Time timestamp = pcl_msg.header.stamp;
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
		return;
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
	cv::imshow("bla", imgRGB);
	cv::waitKey(5);
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

void FastFusionWrapper::getConfImageFromRosMsg(const sensor_msgs::ImageConstPtr& msgConf, cv::Mat *confImg) {
//-- Function to convert ROS-image (Confidence) message to OpenCV-Mat.
	*confImg = cv_bridge::toCvCopy(msgConf, sensor_msgs::image_encodings::TYPE_8UC1)->image;
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
		tfBroadcaster_.sendTransform(tf::StampedTransform(tf_depth_cam0, timestamp, "cam0", cam_id_));
	} else {
		tfBroadcaster_.sendTransform(tf::StampedTransform(tf_rgb_cam0, timestamp, "cam0", cam_id_));
	}
	tfBroadcaster_.sendTransform(tf::StampedTransform(tf_body_cam, timestamp, "camera_imu", "cam0"));
	//tfBroadcaster_.sendTransform(tf::StampedTransform(tf_cam0_imu, timestamp, "imu", "cam0"));

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
