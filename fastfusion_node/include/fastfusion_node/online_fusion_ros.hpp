/*
 * online_fusion_ros.hpp
 *
 *  Created on: Sep 29, 2015
 *      Author: karrer
 */
#ifndef INCLUDE_ONLINE_FUSION_ROS_HPP_
#define INCLUDE_ONLINE_FUSION_ROS_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "camerautils/camerautils.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
//#include <pcl/console/parse.h>

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


#include <auxiliary/multivector.h>

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

//#include <qapplication.h>

#include <tclap/CmdLine.h>
#include <fusion/mesh.hpp>

//#include <QGLViewer/qglviewer.h>
#include <camerautils/camerautils.hpp>
//#include <fusionGPU/geometryfusion_single_aos.hpp>
#include <fusion/geometryfusion_mipmap_cpu.hpp>
#include <fusion/mesh.hpp>
//#include <QtCore/QTimer>

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
	OnlineFusionROS(bool createMeshList = false);
	~OnlineFusionROS();
	//-- Initialization of the parameters read from the ROS parameter file
	void setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float distThreshold, int depthChecks,
			   bool saveMesh, std::string fileName);
	std::vector<float> _boundingBox;
	MeshSeparate *_currentMeshForSave;
	MeshInterleaved *_currentMeshInterleaved;
	std::vector<PointerMeshDraw*> _pointermeshes;
	FusionMipMapCPU* _fusion;
	float _imageDepthScale;
	float _maxCamDistance;

	bool _newMesh;
	bool _fusionActive;
	bool _fusionAlive;

	bool _saveScreenshot;

	bool _threadImageReading;

	//-- Starting new Map
	bool startNewMap();
	float _scale;
	float _distThreshold;
	bool _threadMeshing;
//-- Update Fusion
	// No Noise Data available
	void updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose, double time);
	// With Noise Data
	void updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, cv::Mat &noiseImg,CameraInfo &pose, double time);

	bool isSetup(){ return _isSetup;};
	bool isReady(){ return _isReady;};
	bool isAlive(){ return _fusionAlive;};
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
	std::mutex _visualizationUpdateMutex;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	void drawCameraFrustum(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat &R, cv::Mat &t);
	//
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



	//-- Probably unused variables (maybe can get rid of them)
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
