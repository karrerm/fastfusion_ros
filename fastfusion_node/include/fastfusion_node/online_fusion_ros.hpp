/*
 * online_fusion_ros.hpp
 *
 *  Created on: Sep 29, 2015
 *      Author: karrerm
 */
#ifndef INCLUDE_ONLINE_FUSION_ROS_HPP_
#define INCLUDE_ONLINE_FUSION_ROS_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camerautils/camerautils.hpp>
#include <auxiliary/multivector.h>
//#include <boost/thread/thread.hpp>
//-- PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/Vertices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//-- KdTree for NN-search
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <opencv2/opencv.hpp>
#include <omp.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

//-- Dirty Trick to get rid of compiler error:
// The preprocessor symbol 'Success' is defined, possibly by the X11 header file X.h
#ifdef Success
  #undef Success
#endif

#include <Eigen/Geometry>
#include <auxiliary/debug.hpp>

#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <tclap/CmdLine.h>
#include <fusion/mesh.hpp>
#include <camerautils/camerautils.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <fusion/mesh.hpp>

#define BOXDELTA 0.001
//#define VOLUMERADIUS 1.5
#define VOLUMERADIUS 1.4
#define USE_ORIGINAL_VOLUME 1

#include <fusion/definitions.h>
#include <deque>
#include <list>
#include <thread>
#include <mutex>
#include <condition_variable>



class OnlineFusionROS
{
public:
	//-- Constructor and Destructor
	OnlineFusionROS(bool createMeshList = false);
	~OnlineFusionROS();

	//-- Initialization of the parameters read from the ROS parameter file
	void setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float distThreshold,
			   bool saveMesh, std::string fileName, bool usePclVisualizer);

	//-- Starting new Map
	bool startNewMap();

	//-- Members related to Meshing
	std::vector<float> _boundingBox;
	MeshSeparate *_currentMeshForSave;
	MeshInterleaved *_currentMeshInterleaved;
	std::vector<PointerMeshDraw*> _pointermeshes;

	//-- The actual fusion member
	FusionMipMapCPU* _fusion;

	//-- Depth image scale and maximal distance for the reconstruction
	float _imageDepthScale;
	float _maxCamDistance;

	//-- Checking Members
	bool _newMesh;
	bool _fusionActive;
	bool _fusionAlive;
	bool _saveScreenshot;		// unused
	bool _threadImageReading;	// unused
	bool isSetup(){ return _isSetup;};
	bool isReady(){ return _isReady;};
	bool isAlive(){ return _fusionAlive;};

	//-- Properties for the fusion
	float _scale;				// Voxel size at highest res.
	float _distThreshold;		// Truncation length
	bool _threadMeshing;		// Thread the meshing? (if not threaded->slow)

	//-- Update without noise data
	void updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose, double time, double decayTime, ros::Time timestamp);

	//-- Update with Noise Data
	void updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, cv::Mat &noiseImg,CameraInfo &pose, double time, double decayTime, ros::Time timestamp);

	//-- Get the current point cloud (vertices of the current mesh)
	pcl::PointCloud<pcl::PointXYZRGB> getCurrentPointCloud() {
		return _fusion->getCurrentPointCloud();
	};

	//-- Information of the current model
	int _frameCounter, _modelCounter;
	CameraInfo _currentPose;
	cv::Mat _currentDepthImg;
	pcl::PointXYZ cameraCenter_;

	//-- Function to stop the fusion
	void stop();


protected :
	bool _isSetup;
	//-- Visualization Members
	void visualize();
	std::thread * _visualizationThread;
	bool _update;
	bool _runVisualization;
	bool _use_pcl_visualizer;
	std::mutex _visualizationUpdateMutex;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void drawCameraFrustum(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat &R, cv::Mat &t);

	void pointPickCallback(const pcl::visualization::PointPickingEvent& event, void*);
	bool pointIsClicked, sphereIsInitialized;
	std::vector<pcl::PointXYZ> clickedPoints;
	unsigned int numberClickedPoints;
	float cubeSideLength;
	Eigen::Vector3f cubePos;
	Eigen::Quaternionf cubePose;

	//-- Fusion Thread Members
	bool _threadFusion;
	std::thread * _fusionThread;
	std::mutex _fusionUpdateMutex;
	bool _newDataInQueue;
	std::condition_variable _fusionThreadCondition;
	std::queue<cv::Mat> _queueRGB;
	std::queue<cv::Mat> _queueDepth;
	std::queue<cv::Mat> _queueNoise;
	std::queue<CameraInfo> _queuePose;
	std::queue<double> _queueTime;
	std::queue<double> _queueDecayTime;
	void fusionWrapperROS(void);
	bool _isReady;
	float3 _offset;

	//-- Meshing Members
	std::vector<PointerMeshDraw*> _meshesDraw;
	PointerMeshDraw *_currentMesh;
	unsigned int _currentNV;
	unsigned int _currentNF;
	int _currentMeshType;
	unsigned int _meshNumber;
	unsigned int _fusionNumber;
	float _cx; float _cy; float _cz;
	bool _saveMesh;
	std::string _fileName;

	//-- Unused Variables (taken from original fastfusion...)
	long long _lastComputedFrame;
	bool _verbose;
	bool _showCameraFrustum;
	bool _showGridBoundingbox;
	bool _showDepthImage;
	bool _showColor;
	int _displayMode;
	unsigned int _vertexBufferSize;
	unsigned int _faceBufferSize;
	unsigned int _edgeBufferSize;
	bool _onTrack;
	bool _onInterpolation;
	bool _saving;
	bool _runFusion;
	bool _createMeshList;
	bool _lightingEnabled;
	bool _colorEnabled;
	struct cameraFrustum{
		Eigen::Vector3f tl0,tr0,br0,bl0,c0;
	} cameraFrustum_;
};

#endif /* INCLUDE_ONLINE_FUSION_ROS_HPP_ */
