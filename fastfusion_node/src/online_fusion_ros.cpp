/*
 * online_fusion_ros.cpp
 *
 *  Created on: Sep 29, 2015
 *      Author: karrer
 */

#include <math.h>
#include <stdio.h>

//#define PREPROCESS_IMAGES

//#define DEBUG_NO_MESHES
//#define DEBUG_NO_MESH_VISUALIZATION
#include "fastfusion_node/online_fusion_ros.hpp"

#include <opencv2/opencv.hpp>

OnlineFusionROS::OnlineFusionROS(bool createMeshList):
_currentMeshForSave(NULL),
_currentMeshInterleaved(NULL),
	_fusion(NULL),
	_imageDepthScale(5000.0f), _maxCamDistance(MAXCAMDISTANCE),
	//_currentFrame(-1),
	//_currentTrajectory(-1),
//	_firstFrame(-1),
	//_nextStopFrame(0),
	_threadFusion(false),
	_fusionThread(NULL),
	_newMesh(false),
	_fusionActive(true),
	_fusionAlive(true),
	_threadImageReading(false),
	_lastComputedFrame(-1),
		_verbose(true),
		_showCameraFrustum(true), _showDepthImage(true),
		_currentMesh(NULL),_currentNV(0),_currentNF(0), _currentMeshType(0),
		//_vertexBuffer(0), _faceBuffer(0), _edgeBuffer(0), _colorBuffer(0),
		_vertexBufferSize(0), _faceBufferSize(0), _edgeBufferSize(0),
		_onTrack(false), _onInterpolation(false), _saving(false),
		 _runFusion(false), _createMeshList(createMeshList)
,_lightingEnabled(false)
,_colorEnabled(true)
,_isSetup(false)
{
	_meshNumber = 0;
	_fusionNumber = 0;
	_cx = 0.0f; _cy = 0.0f; _cz = 0.0f;
	_isReady = true;
	_frameCounter = 0;
	_modelCounter = 0;
	_fusionActive = true;
	_fusionAlive = true;
	_update = false;
	_runVisualization = true;
	pointIsClicked = false;
	sphereIsInitialized = true;
	numberClickedPoints = 0;
	//-- Create Visualization Thread
	_visualizationThread = new std::thread(&OnlineFusionROS::visualize, this);
}

OnlineFusionROS::~OnlineFusionROS()
{
	std::cout << "in OnlineFusionROS destructor " << std::endl;
	//-- Delete Data
	if(_fusion) delete _fusion;
	if(_currentMeshForSave) delete _currentMeshForSave;
	if(_currentMeshInterleaved) delete _currentMeshInterleaved;

	fprintf(stderr,"\nEnd of OnlineFusionROS Destructor.");
#ifdef INTERACTIVE_MEMORY_MANAGEMENT
	if(INTERACTIVE_MEMORY_MANAGEMENT){
		fprintf(stderr,"\nInput:");
		char input[256];
		fprintf(stderr,"%s",fgets(input,256,stdin));
	}
#endif
}

void OnlineFusionROS::stop() {
//-- Stop the fusion process and save the current mesh if required
	_runVisualization = false;
	if (_threadFusion) {
		if (_fusionThread) {
			std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
			_newMesh = false;
			_runFusion = false;
			_newDataInQueue = true;
			_fusionThreadCondition.notify_one();
			updateLock.unlock();
			_fusionThread->join();
			delete _fusionThread;
			_fusionThread = NULL;

		}
	}
	_runFusion = false;
	//-- wait for last frame fusion is finished.
	while (_fusionActive);
	_fusionAlive = false;

	//-- Save current Mesh
	if (_saveMesh) {
		_currentMeshInterleaved->writePLY(_fileName,false);
	}
	while(!_fusion->meshUpdateFinished()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	if(_fusion) {
		_offset = _fusion->offset();
		delete _fusion;
		_fusion = NULL;
	}
	if(_currentMeshForSave) {
		delete _currentMeshForSave;
		_currentMeshForSave = NULL;
	}
	if(_currentMeshInterleaved) {
		delete _currentMeshInterleaved;
		_currentMeshInterleaved = NULL;
	}

	//-- End Visualization Thread
	//viewer->close();
	//_visualizationThread->join();
	//while (_update);
	//delete _visualizationThread;
}

bool OnlineFusionROS::startNewMap() {
//-- Start a new map, only if initialization was performed before. Also Check if currently a process is running.  If any
//-- of these conditions is not met, return false
	//-- Check if a FusionMipMapCPU object exists
	//delete _fusion;
	//_fusion = NULL;
	if (_fusion || !_isSetup) {
		return false;
	}
	if (!_queueDepth.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueDepth, empty );
	}
	if (!_queueRGB.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueRGB, empty );
	}
	if (!_queueNoise.empty()) {
		std::queue<cv::Mat> empty;
		std::swap( _queueNoise, empty );
	}
	if (!_queuePose.empty()) {
		std::queue<CameraInfo> empty;
		std::swap( _queuePose, empty );
	}
	if (!_queueTime.empty()) {
		std::queue<double> empty;
		std::swap( _queueTime, empty );
	}
	_fusion = new FusionMipMapCPU(_offset.x,_offset.y,_offset.z,_scale, _distThreshold,0,true);
	_fusion->setThreadMeshing(_threadMeshing);
	_fusion->setIncrementalMeshing(true);
	_fusionActive = true;
//	_fusionAlive = true;
	_update = false;
	_runFusion = true;
	return true;
}

void OnlineFusionROS::setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float distThreshold, int depthChecks,
								bool saveMesh, std::string fileName){
//-- Initialize FusionMipMapCPU-class (_fusion)
	_imageDepthScale = imageScale;
	_scale = scale;
	_distThreshold = distThreshold;
	_threadMeshing = meshingThread;
	_threadFusion = fusionThread;
	_saveMesh = saveMesh;
	_fileName = fileName;
	_isSetup = true;
	_offset.x = _offset.y = _offset.z = 0.0f;
	//startNewMap();
}
/*  Not needed in this implementation
typedef struct FusionParameter_{
  FusionMipMapCPU *fusion;
  float imageDepthScale;
  float maxCamDistance;
  bool threadImageReading;
  size_t stopFrame;
  FusionParameter_(FusionMipMapCPU *fusion_p, float imageDepthScale_p, float maxCamDistance_p, bool threadImageReading_p, size_t stopFrame_p)
  :fusion(fusion_p),imageDepthScale(imageDepthScale_p),maxCamDistance(maxCamDistance_p), threadImageReading(threadImageReading_p)
  ,stopFrame(stopFrame_p){}
} FusionParameter;
*/



void OnlineFusionROS::fusionWrapperROS(void) {
//-- Fusion-Wrapper member to enable threading the fusion process. To buffer the data needed for the fusion
//-- (rgb-Image,depth-image, camera-pose), a std::queue with Mutex locking is used.
	//-- Initialize datatypes to store the current values
	_fusionActive = true;
	cv::Mat currImgRGB, currImgDepth, currNoise;
	CameraInfo currPose;
	double currTime;
	unsigned int framesProcessed = 0;
	//-- Perform thread, as long as fusion is active
	while (_runFusion) {
		//-- Check if there is data available in the queue
		std::unique_lock<std::mutex> locker(_fusionUpdateMutex);
		while (!_newDataInQueue) {
			_fusionThreadCondition.wait(locker);
		}
		if (!_runFusion){
			std::cout << "tries break" << std::endl;
			locker.unlock();
			break;
		}
		std::queue<cv::Mat> queueRGB, queueDepth, queueNoise;
		std::queue<CameraInfo> queuePose;
		std::queue<double> queueTime;
		for (size_t i = 0; i < _queueRGB.size(); i++) {
			queueRGB.push(_queueRGB.front());
			_queueRGB.pop();
			queueDepth.push(_queueDepth.front());
			_queueDepth.pop();
			queuePose.push(_queuePose.front());
			_queuePose.pop();
			queueTime.push(_queueTime.front());
			_queueTime.pop();
			if (!_queueNoise.empty()) {
				queueNoise.push(_queueNoise.front());
				_queueNoise.pop();
			}
		}
		_newDataInQueue = false;
		locker.unlock();
		for (size_t i = 0; i < queueRGB.size(); i++) {
			framesProcessed++;
			if (!queueNoise.empty()) {
				//-- Depth-Noise Data is available
				currImgRGB = queueRGB.front();
				queueRGB.pop();
				currImgDepth = queueDepth.front();
				queueDepth.pop();
				currPose = queuePose.front();
				queuePose.pop();
				currNoise = queueNoise.front();
				queueNoise.pop();
				currTime = queueTime.front();
				queueTime.pop();
				//-- Add Map and perform update
				_fusion->addMap(currImgDepth, currNoise,currPose,currImgRGB,1.0f/_imageDepthScale,_maxCamDistance, currTime);
				_newMesh = _fusion->updateMeshes();
			} else {
				//-- No Depth Noise Data is available
				currImgRGB = queueRGB.front();
				queueRGB.pop();
				currImgDepth = queueDepth.front();
				queueDepth.pop();
				currPose = queuePose.front();
				queuePose.pop();
				currTime = queueTime.front();
				queueTime.pop();
				//updateLock.unlock();
				//-- Add Map and perform update
				_fusion->addMap(currImgDepth,currPose,currImgRGB,1.0f/_imageDepthScale,_maxCamDistance,currTime);
				_newMesh = _fusion->updateMeshes();
			}
		}
	}
	_fusionActive = false;
	
}	

void OnlineFusionROS::drawCameraFrustum(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat &R_cv, cv::Mat &t_cv) {
	Eigen::Vector3f tl1,tr1,br1,bl1,c1,t;
	Eigen::Matrix3f R;
	double linewidth = 3.0;
	for (int i = 0;i < 3; i++) {
		for(int j = 0; j<3;j++) {
			R(i,j) = (float)R_cv.at<double>(i,j);
		}
	}
	t(0) = (float)t_cv.at<double>(0,0); t(1) = (float)t_cv.at<double>(1,0); t(2) = (float)t_cv.at<double>(2,0);
	tl1 = (R*cameraFrustum_.tl0 + t); tr1 = (R*cameraFrustum_.tr0 + t); br1 = (R*cameraFrustum_.br0 + t);
	bl1 = (R*cameraFrustum_.bl0 + t); c1 = (R*cameraFrustum_.c0 + t);
	//-- Draw Camera Frustum
	viewer->removeShape("t",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
			pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),0.0, 1.0, 0.0, "t", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"t",0);
	viewer->removeShape("r",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
			pcl::PointXYZ(br1(0),br1(1),br1(2)),0.0, 1.0, 0.0, "r", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"r",0);
	viewer->removeShape("b",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
			pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),0.0, 1.0, 0.0, "b", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"b",0);
	viewer->removeShape("l",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
			pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),0.0, 1.0, 0.0, "l", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"l",0);
	viewer->removeShape("tl_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "tl_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"tl_c",0);
	viewer->removeShape("tr_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "tr_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"tr_c",0);
	viewer->removeShape("bl_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "bl_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"bl_c",0);
	viewer->removeShape("br_c",0);
	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
			pcl::PointXYZ(c1(0),c1(1),c1(2)),0.0, 1.0, 0.0, "br_c", 0);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"br_c",0);

}


void OnlineFusionROS::visualize() {
//-- Preliminary visualization: Generate colored Point cloud from the mesh vertexes
//-- Careful!: No check whether the mesh changes during the point cloud generation is performed--> may cause Problems!!
	//-- Initialize PCL-Viewer
	double linewidth = 3.0;
	bool pointcloudInit = false;
	//-- Set Camera Viewing Parameters
	pcl::visualization::Camera camera;
	camera.focal[0] = -0.365056; camera.focal[1] = -0.026536; camera.focal[2] = -0.0145319;
	camera.pos[0] = -0.160066; camera.pos[1] = -0.540765; camera.pos[2] = 0.599112;
	camera.view[0] = -0.336011; camera.view[1] = 0.663623; camera.view[2] = 0.668357;
	camera.clip[0] = 0.00431195; camera.clip[1] = 4.31195;
	camera.fovy = 0.8575;
	camera.window_size[0] = 1600; camera.window_size[1] = 1200;
	camera.window_pos[0] = 0; camera.window_pos[1] = 0;

	std::vector<pcl::visualization::Camera> cameras;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("visualization pc"));
	viewer->setCameraParameters(camera,0);
	viewer->setShowFPS (false);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 30);
	viewer->registerPointPickingCallback(&OnlineFusionROS::pointPickCallback, *this);
	viewer->addCoordinateSystem (1);
	viewer->addSphere(cameraCenter_,0.1,1.0,0.0,0.0,"camerCenter",0);
	//viewer->initCameraParameters ();

	cv::Mat K_cv,Ext_cv, R_cv,t_cv;
	Eigen::Matrix4f Ext;
	Eigen::Matrix3f K,R;
	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ camPointTemp;
	//-- Define Cornerpoints for camera frustum
	//Eigen::Vector3f tl1,tr1,br1,bl1,c1,t;
	cameraFrustum_.tl0 << -0.24,-0.17,0.4;
	cameraFrustum_.tr0 << 0.24,-0.17,0.4;
	cameraFrustum_.br0 << 0.24,0.17,0.4;
	cameraFrustum_.bl0 << -0.24,0.17,0.4;
	cameraFrustum_.c0 << 0.0,0.0,0.0;

    while ((!viewer->wasStopped ()) && _runVisualization) {
    	viewer->spinOnce (10);
    	//viewer->setCameraParameters(camera,0);
    	//viewer->setCameraParameters(camera,0);
    	//-- Get Extrinsics
    	R_cv = _currentPose.getRotation();
    	t_cv = _currentPose.getTranslation();
    	drawCameraFrustum(viewer, R_cv, t_cv);
    	viewer->updateSphere(cameraCenter_,0.1,1.0,0.0,0.0,"camerCenter");

    	/*
    	switch (numberClickedPoints) {
    	case 1:
    		viewer->removeShape("point0",0);
    		viewer->addSphere(clickedPoints[0],0.05,1.0,0.0,0.0,"point0",0);
    		break;
    	case 2:
    		viewer->removeShape("point0",0);
    		viewer->addSphere(clickedPoints[0],0.05,1.0,0.0,0.0,"point0",0);
    		viewer->removeShape("point1",0);
    		viewer->addSphere(clickedPoints[1],0.05,1.0,0.0,0.0,"point1",0);
    		break;
    	case 3:
    		viewer->removeShape("point0",0);
    		viewer->addSphere(clickedPoints[0],0.05,1.0,0.0,0.0,"point0",0);
    		viewer->removeShape("point1",0);
    		viewer->addSphere(clickedPoints[1],0.05,1.0,0.0,0.0,"point1",0);
    		viewer->removeShape("point2",0);
    		viewer->addSphere(clickedPoints[2],0.05,1.0,0.0,0.0,"point2",0);
    		break;
    	case 0:
    		viewer->removeShape("point0",0);
    		viewer->removeShape("point1",0);
    		viewer->removeShape("point2",0);
    		break;
    	}
    	if (pointIsClicked) {
    	    		pointIsClicked = false;
    		if (sphereIsInitialized) {
    			std::cout << "Actualize Cube " << std::endl;
    			viewer->removeShape("cube",0);
    			viewer->addCube(cubePos,cubePose, cubeSideLength, cubeSideLength, cubeSideLength, "cube", 0);
    			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"cube",0);
    			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,0.0,1.0,"cube",0);
    			//viewer->addSphere(clickedPoint,0.1,1.0,0.0,0.0,"sphere",0);
    		} else {
    			sphereIsInitialized = true;
    			viewer->addCube(cubePos,cubePose, cubeSideLength, cubeSideLength, cubeSideLength, "cube", 0);
    			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,linewidth,"cube",0);
    			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.0,0.0,1.0,"cube",0);
    			//viewer->addSphere(clickedPoint,0.1,1.0,0.0,0.0,"sphere",0);
    		}
    	}
		*/

    	//-- Set Camera viewpoint
    	/*
    	viewer->setCameraPosition(t_cv.at<double>(0,0)-R_cv.at<double>(0,2)*2.5,t_cv.at<double>(1,0)-R_cv.at<double>(1,2)*2.5,
    			t_cv.at<double>(2,0)-R_cv.at<double>(2,2)*2.5,t_cv.at<double>(0,0)+R_cv.at<double>(0,3),t_cv.at<double>(1,0)+R_cv.at<double>(1,2),
				t_cv.at<double>(2,0)+R_cv.at<double>(2,2),0,0,1,0);
		*/
    	//-- Check whether the point cloud should be updated (only every 20 Frames!)

    	if(_update) {
    		//-- Lock Data queue

    		//std::lock_guard<std::mutex> updateLock(_visualizationUpdateMutex);
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    		pcl::PointXYZRGB pointTemp;
    		std::vector<pcl::Vertices>  polygons;
    		pcl::Vertices tempFace;
    		//-- Generate Point cloud from vertexes (!!! O(n)-operation !!!)
    		//std::cout << "Size of current point cloud = " << _currentPointCloud->size() << std::endl;
    		/*//-- Not needed since parallel updated in fusion
     		for (unsigned int i = 0; i < _currentMeshInterleaved->vertices.size(); i++ ) {
    			pointTemp.x = _currentMeshInterleaved->vertices[i].x;
    			pointTemp.y = _currentMeshInterleaved->vertices[i].y;
    			pointTemp.z = _currentMeshInterleaved->vertices[i].z;
    			pointTemp.r = _currentMeshInterleaved->colors[i].r;
    			pointTemp.g = _currentMeshInterleaved->colors[i].g;
    			pointTemp.b = _currentMeshInterleaved->colors[i].b;
    			point_cloud_ptr->points.push_back (pointTemp);
    		}
			*/
     		/*
    		for (unsigned int i = 0; i < _currentMeshInterleaved->faces.size();i+= _currentMeshInterleaved->_verticesPerFace){
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+0]);
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+1]);
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+2]);
    			polygons.push_back(tempFace);
    			tempFace.vertices.clear();
    		}
			*/

    		//updateLock.unlock();


    		/*
    		pcl::PolygonMesh triangles;
    		pcl::PCLPointCloud2 msg;
    		pcl::toPCLPointCloud2 (*point_cloud_ptr, msg);
    		triangles.cloud = msg;
    		triangles.polygons = polygons;
    		*/
    		//-- Update Viewer
    		pcl::PointCloud<pcl::PointXYZRGB> points = _fusion->getCurrentPointCloud();
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_ptr (new pcl::PointCloud<pcl::PointXYZRGB>(points));
    		if (points_ptr->points.size() > 0) {
    			std::string modelName = "/home/karrer/PointClouds/Models/Model" + std::to_string(_modelCounter)  + ".ply";
    			pcl::io::savePLYFile(modelName,*points_ptr);
    			_modelCounter++;

    		//-- KdTree for NN-search
    		/*
    		std::clock_t begin = std::clock();
    		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    		kdtree.setInputCloud (points_ptr);
    		//-- Search nearest Point
    		pcl::PointXYZRGB searchPoint;
    		searchPoint.x = 1.0; searchPoint.y = 2.1; searchPoint.z = 0; searchPoint.r = 1; searchPoint.g = 0; searchPoint.b = 0;
    		std::vector<int> pointIdxNKNSearch(5);
    		std::vector<float> pointNKNSquaredDistance(5);
    		kdtree.nearestKSearch (searchPoint, 5, pointIdxNKNSearch, pointNKNSquaredDistance);
    		std::clock_t end = std::clock();
    		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    		std::cout << "Time for kd-tree: " << elapsed_secs << std::endl;
			*/
    		if (pointcloudInit) {
    			//viewer->updatePointCloud(point_cloud_ptr,"visualization pc");
    			viewer->updatePointCloud(points_ptr,"visualization pc");
    			/*if (_saveScreenshot) {
    			viewer->saveScreenshot("/home/karrer/Desktop/fbf_video/pointCloud.png");
    			_saveScreenshot = false;
    			}*/
    			// Mesh visualization --> slow
    			//viewer->removePolygonMesh("visualization pc");
    			//viewer->addPolygonMesh(triangles,"visualization pc");
    		} else {
    			std::cout << "Crashes before adding point cloud " << std::endl;
    			//viewer->addPointCloud(point_cloud_ptr, "visualization pc");
    			viewer->addPointCloud(points_ptr, "visualization pc");
    			// Mesh visualization --> slow
    			//viewer->addPolygonMesh(triangles,"visualization pc");
    			pointcloudInit = true;
    		}
    		}
            _update = false;

            point_cloud_ptr->clear();
            //polygons.clear();
        }
    	viewer->getCameras(cameras);
    }
    viewer->removePointCloud("visualization pc",0);
    viewer->close();
    //-- Output the last camera parameters
    std::cout << " \n \n \n";
    std::cout << "----- All Camera View Parameters --------" << std::endl;
    std::cout << "camera.focal[0] = " << cameras[0].focal[0] << "; camera.focal[1] = " << cameras[0].focal[1] << "; camera.focal[2] = " << cameras[0].focal[2] << ";" << std::endl;
    std::cout << "camera.pos[0] = " << cameras[0].pos[0] << "; camera.pos[1] = " << cameras[0].pos[1] << "; camera.pos[2] = " << cameras[0].pos[2] <<";" << std::endl;
    std::cout << "camera.view[0] = " << cameras[0].view[0] << "; camera.view[1] = " << cameras[0].view[1] << "; camera.view[2] = " << cameras[0].view[2] <<";"<< std::endl;
    std::cout << "camera.clip[0] = " << cameras[0].clip[0] << "; camera.clip[1] = " << cameras[0].clip[1] << ";" << std::endl;
    std::cout << "camera.fovy = " << cameras[0].fovy << ";" <<std::endl;
    std::cout << "camera.window_size[0] = " << cameras[0].window_size[0] << "; camera.window_size[1] = " << cameras[0].window_size[1] << ";" << std::endl;
    std::cout << "camera.window_pos[0] = " << cameras[0].window_pos[0] << "; camera.window_pos[1] = " << cameras[0].window_pos[1] << ";" << std::endl;
}


void OnlineFusionROS::pointPickCallback(const pcl::visualization::PointPickingEvent& event, void*){
	if (event.getPointIndex () == -1) {
		return;
	}
	if (numberClickedPoints <= 3) {
		std::cout << "Adding Points" << std::endl;
		pcl::PointXYZ clickedPoint;
		event.getPoint(clickedPoint.x,clickedPoint.y,clickedPoint.z);
		clickedPoints.push_back(clickedPoint);
		numberClickedPoints++;
		/*
		if (numberClickedPoints == 3) {
			//-- Estimate Plane and Dimensions of Cube
			cubeSideLength = std::sqrt((clickedPoints[0].x-clickedPoints[1].x) + (clickedPoints[0].y-clickedPoints[1].y) +
					(clickedPoints[0].z-clickedPoints[1].z));
			std::cout << "Cube Length = " << cubeSideLength << std::endl;
			Eigen::Vector3f x_axis, y_axis, z_axis, temp_vec;
			for (int i = 0; i< 3; i++) {
				std::cout << "point("<< i+1 << ") = [" << clickedPoints[i].x << "; " << clickedPoints[i].y << "; " <<
						clickedPoints[i].z << "];" << std::endl;
			}

			x_axis(0) = (float)(clickedPoints[1].x - clickedPoints[0].x);
			x_axis(1) = (float)(clickedPoints[1].y - clickedPoints[0].y);
			x_axis(2) = (float)(clickedPoints[1].z - clickedPoints[0].z);
			x_axis.normalize();
			temp_vec(0) = clickedPoints[2].x - clickedPoints[0].x;
			temp_vec(1) = clickedPoints[2].y - clickedPoints[0].y;
			temp_vec(2) = clickedPoints[2].z - clickedPoints[0].z;
			z_axis = x_axis.cross(temp_vec);
			z_axis.normalize();
			y_axis = x_axis.cross(z_axis);
			y_axis.normalize();
			Eigen::Matrix3f R;
			R << x_axis, y_axis, z_axis;
			std::cout << "R = " << R <<std::endl;
			if (R.determinant() < 0) {
				R.col(2) = R.col(2)*(-1);
			}
			cubePose = R;
			cubePos(0) = clickedPoints[0].x; //+ cubeSideLength/2.0f * (x_axis(0) + y_axis(0) + z_axis(0));
			cubePos(1) = clickedPoints[0].y; // + cubeSideLength/2.0f * (x_axis(1) + y_axis(1) + z_axis(1));
			cubePos(2) = clickedPoints[0].z; // + cubeSideLength/2.0f * (x_axis(2) + y_axis(2) + z_axis(2));

			numberClickedPoints = 0;
			clickedPoints.clear();
			pointIsClicked = true;
		}
		*/
	}
}


void OnlineFusionROS::updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose, double time) {
//-- Adapted version of the updateSlot()-function in order to process the current images.
	if (!_threadFusion) {
	//-- Unthreaded Fusion
		_fusionActive = true;
		_frameCounter++;
		_isReady = false;
		_currentPose = pose;
		depthImg.copyTo(_currentDepthImg);
		//-- Lock visualization Mutex
		{
		std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
		//-- Add and update Map
		_fusion->addMap(depthImg,pose,rgbImg,1.0f/_imageDepthScale,_maxCamDistance,time);
		_fusion->updateMeshes();
		if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
		if(_pointermeshes[0]) delete _pointermeshes[0];
		if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
		if(!_currentMeshInterleaved) {
			std::cout << "Create New Mesh Interleaved" << std::endl;;
			_currentMeshInterleaved = new MeshInterleaved(3);
		}
		//-- Generate new Mesh
		*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 3) {
			_update = true;
			_frameCounter = 0;
		}
		_isReady = true;
		_fusionActive = false;
	} else {
	//-- The fusion process is threaded
		if(!_fusionThread){
			//-- If not yet initialize --> initialize fusion thread
			_fusionThread = new std::thread(&OnlineFusionROS::fusionWrapperROS, this);
			_runFusion = true;
		}
		_currentPose = pose;
		//-- Lock and update data-queue
		{
		std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
		_queueRGB.push(rgbImg);
		_queueDepth.push(depthImg);
		_queuePose.push(pose);
		_queueTime.push(time);
		_newDataInQueue = true;
		_fusionThreadCondition.notify_one();
		}
		//updateLock.unlock();
		_frameCounter++;
		//--Update Mesh
		if(_newMesh){
			_newMesh = false;
			{ // visualization scope begin
			std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
			if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
			if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
			*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			//pcl::PointCloud<pcl::PointXYZRGB> points = _fusion->getCurrentPointCloud();
			} // visualization scope end
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 3) {
			_update = true;
		}
	}
}



void OnlineFusionROS::updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, cv::Mat &noiseImg,CameraInfo &pose, double time) {
//-- Update Fusion function when using it with noise data (from ToF camera)
	if (!_threadFusion) {
		//-- Unthreaded Fusion
		_fusionActive = true;
		_frameCounter++;
		_isReady = false;
		_currentPose = pose;
		depthImg.copyTo(_currentDepthImg);
		//-- Lock visualization Mutex
		{
		std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
		//-- Add and update Map
		_fusion->addMap(depthImg, noiseImg,pose,rgbImg,1.0f/_imageDepthScale,_maxCamDistance,time);
		_fusion->updateMeshes();
		if(!_pointermeshes.size()) _pointermeshes.resize(1,NULL);
		if(_pointermeshes[0]) delete _pointermeshes[0];
		if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
		if(!_currentMeshInterleaved) {
			_currentMeshInterleaved = new MeshInterleaved(3);
		}
		//-- Generate new Mesh
		*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 3) {
			_update = true;
			_frameCounter = 0;
		}
		_isReady = true;
		_fusionActive = false;
	} else {
		//-- The fusion process is threaded
		if(!_fusionThread){
			//-- If not yet initialize --> initialize fusion thread
			_fusionThread = new std::thread(&OnlineFusionROS::fusionWrapperROS, this);
			_runFusion = true;
		}
		_currentPose = pose;
		//-- Lock and update data-queue
		{
		std::unique_lock<std::mutex> updateLock(_fusionUpdateMutex);
		_queueTime.push(time);
		_queueRGB.push(rgbImg);
		_queueDepth.push(depthImg);
		_queueNoise.push(noiseImg);
		_queuePose.push(pose);
		_newDataInQueue = true;
		_fusionThreadCondition.notify_one();
		}
		_frameCounter++;
		//--Update Mesh
		if(_newMesh){
			_newMesh = false;
			{
			std::lock_guard<std::mutex> updateLockVis(_visualizationUpdateMutex);
			if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
			if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
			*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			//pcl::PointCloud<pcl::PointXYZRGB> points = _fusion->getCurrentPointCloud();
			}
			//updateLockVis.unlock();
		}
		//-- Check whether to update Visualization
		if (_frameCounter > 3) {
			_update = true;
		}
	}
}
