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
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>

#include <pcl/PolygonMesh.h>
#include <pcl/PCLPointCloud2.h>

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


class OnlineFusionROS
{
public:
	OnlineFusionROS(bool createMeshList = false);
	~OnlineFusionROS();
	//-- Initialization of the parameters read from the ROS parameter file
	void setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float threshold, int depthChecks);
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

	bool _threadImageReading;
	void updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose);

	bool isReady(){ return _isReady;};
	int _frameCounter;
	CameraInfo _currentPose;
	cv::Mat _currentDepthImg;

	//-- Function to stop the fusion
	void stop();


protected :
	//-- Visualization Members
	void visualize();
	boost::thread * _visualizationThread;
	bool _update = false;
	boost::mutex _visualizationUpdateMutex;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	//-- Fusion Thread Members
	bool _threadFusion;
	boost::thread *_fusionThread;
	boost::mutex _fusionUpdateMutex;
	std::queue<cv::Mat> _queueRGB;
	std::queue<cv::Mat> _queueDepth;
	std::queue<CameraInfo> _queuePose;
	void fusionWrapperROS(void);
	bool _isReady;

	//-- Meshing Members
	std::vector<PointerMeshDraw*> _meshesDraw;
	PointerMeshDraw *_currentMesh;
	unsigned int _currentNV;
	unsigned int _currentNF;
	int _currentMeshType;
	unsigned int _meshNumber;
	unsigned int _fusionNumber;
	float _cx; float _cy; float _cz;

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

};

#endif /* INCLUDE_ONLINE_FUSION_ROS_HPP_ */
