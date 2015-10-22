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
{
	_meshNumber = 0;
	_fusionNumber = 0;
	_cx = 0.0f; _cy = 0.0f; _cz = 0.0f;
	_isReady = true;
	_frameCounter = 0;
	_fusionActive = true;
	_fusionAlive = true;
	_update = false;
	_runVisualization = true;

	//-- Create Visualization Thread
	_visualizationThread = new boost::thread(&OnlineFusionROS::visualize, this);
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
	_runFusion = false;
	_runVisualization = false;
	if (_threadFusion) {
		_fusionThread->join();
	}

	//-- wait for last frame fusion is finished.
	while (_fusionActive);
	_fusionAlive = false;
	delete _fusionThread;

	//-- Save current Mesh
	if (_saveMesh) {
		_currentMeshInterleaved->writePLY(_fileName,false);
	}
	//-- End Visualization Thread
	//viewer->close();
	//_visualizationThread->join();
	//while (_update);
	//delete _visualizationThread;
}


void OnlineFusionROS::setupFusion(bool fusionThread, bool meshingThread,float imageScale, float scale, float threshold, int depthChecks,
								bool saveMesh, std::string fileName){
//-- Initialize FusionMipMapCPU-class (_fusion)
	_fusion = new FusionMipMapCPU(0,0,0,scale,threshold,0,true);
	_fusion->setThreadMeshing(meshingThread);
	_fusion->setDepthChecks(depthChecks);
	_imageDepthScale = imageScale;
	_fusion->setIncrementalMeshing(true);
	_threadFusion = fusionThread;
	_saveMesh = saveMesh;
	_fileName = fileName;

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
	cv::Mat currImgRGB, currImgDepth;
	CameraInfo currPose;
	unsigned int framesProcessed = 0;
	//-- Perform thread, as long as fusion is active
	while (_runFusion) {
		//-- Check if there is data available in the queue
		if ((_queueRGB.size() >=1) && (_queueDepth.size() >= 1) && (_queuePose.size() >=1) ) {
			framesProcessed++;
			//-- Data is available --> Perform fusion
			// Lock Mutex to extract data
			boost::mutex::scoped_lock updateLock(_fusionUpdateMutex);
			currImgRGB = _queueRGB.front();
			_queueRGB.pop();
			currImgDepth = _queueDepth.front();
			_queueDepth.pop();
			currPose = _queuePose.front();
			_queuePose.pop();
			// Unlock Mutex
			updateLock.unlock();
			//-- Add Map and perform update
			_fusion->addMap(currImgDepth,currPose,currImgRGB,1.0f/_imageDepthScale,_maxCamDistance);
			_newMesh = _fusion->updateMeshes();
		}
	}
	_fusionActive = false;
	
}	



void OnlineFusionROS::visualize() {
//-- Preliminary visualization: Generate colored Point cloud from the mesh vertexes
//-- Careful!: No check whether the mesh changes during the point cloud generation is performed--> may cause Problems!!
	//-- Initialize PCL-Viewer
	bool pointcloudInit = false;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("visualization pc"));
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15);
	viewer->addCoordinateSystem (0.1);
	viewer->initCameraParameters ();
	cv::Mat K_cv,Ext_cv, R_cv,t_cv;
	Eigen::Matrix4f Ext;
	Eigen::Matrix3f K,R;
	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ camPointTemp;
	//-- Define Cornerpoints for camera frustum
	Eigen::Vector3f tl0,tr0,br0,bl0,c0,tl1,tr1,br1,bl1,c1,t;
	tl0 << -0.24,-0.17,0.4;
	tr0 << 0.24,-0.17,0.4;
	br0 << 0.24,0.17,0.4;
	bl0 << -0.24,0.17,0.4;
	c0 << 0.0,0.0,0.0;

    while ((!viewer->wasStopped ()) && _runVisualization) {
    	viewer->spinOnce (10);
    	viewer->setCameraPosition(0.0268573,26.8573,0,1/0.686973,-1.44275,2.42881/-0.078733,0.682304,0.726817/0.8575/960,540/65,52);
    	//-- Get Extrinsics
    	R_cv = _currentPose.getRotation();
    	t_cv = _currentPose.getTranslation();
    	for (int i = 0;i < 3; i++) {
    		for(int j = 0; j<3;j++) {
    			R(i,j) = (float)R_cv.at<double>(i,j);
    		}
    	}
    	t(0) = (float)t_cv.at<double>(0,0); t(1) = (float)t_cv.at<double>(1,0); t(2) = (float)t_cv.at<double>(2,0);
    	//-- Compute Camera Frustum
    	tl1 = (R*tl0 + t)*0.7; tr1 = (R*tr0 + t)*0.7; br1 = (R*br0 + t)*0.7; bl1 = (R*bl0 + t)*0.7; c1 = (R*c0 + t)*0.7;
    	//-- Draw Camera Frustum
    	viewer->removeShape("t",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
    			pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),1.0, 0.0, 0.0, "t", 0);
    	viewer->removeShape("r",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
    			pcl::PointXYZ(br1(0),br1(1),br1(2)),1.0, 0.0, 0.0, "r", 0);
    	viewer->removeShape("b",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
    			pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),1.0, 0.0, 0.0, "b", 0);
    	viewer->removeShape("l",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
    			pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),1.0, 0.0, 0.0, "l", 0);
    	viewer->removeShape("tl_c",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tl1(0),tl1(1),tl1(2)),
    			pcl::PointXYZ(c1(0),c1(1),c1(2)),1.0, 0.0, 0.0, "tl_c", 0);
    	viewer->removeShape("tr_c",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(tr1(0),tr1(1),tr1(2)),
    			pcl::PointXYZ(c1(0),c1(1),c1(2)),1.0, 0.0, 0.0, "tr_c", 0);
    	viewer->removeShape("bl_c",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(bl1(0),bl1(1),bl1(2)),
    			pcl::PointXYZ(c1(0),c1(1),c1(2)),1.0, 0.0, 0.0, "bl_c", 0);
    	viewer->removeShape("br_c",0);
    	viewer->addLine<pcl::PointXYZ> (pcl::PointXYZ(br1(0),br1(1),br1(2)),
    			pcl::PointXYZ(c1(0),c1(1),c1(2)),1.0, 0.0, 0.0, "br_c", 0);
    	/*
    	//-- Set Camera viewpoint
    	viewer->setCameraPosition(t_cv.at<double>(0,0)-R_cv.at<double>(0,2)*2.5,t_cv.at<double>(1,0)-R_cv.at<double>(1,2)*2.5,
    			t_cv.at<double>(2,0)-R_cv.at<double>(2,2)*2.5,t_cv.at<double>(0,0)+R_cv.at<double>(0,3),t_cv.at<double>(1,0)+R_cv.at<double>(1,2),
				t_cv.at<double>(2,0)+R_cv.at<double>(2,2),0,0,1,0);
		*/
    	//-- Check whether the point cloud should be updated (only every 20 Frames!)
    	if(_update) {
    		//-- Lock Data queue
    		boost::mutex::scoped_lock updateLock(_visualizationUpdateMutex);
    		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    		pcl::PointXYZRGB pointTemp;
    		//std::vector<pcl::Vertices>  polygons;
    		//pcl::Vertices tempFace;
    		//-- Generate Point cloud from vertexes (!!! O(n)-operation !!!)
     		for (unsigned int i = 0; i < _currentMeshInterleaved->vertices.size(); i++ ) {
    			pointTemp.x = _currentMeshInterleaved->vertices[i].x;
    			pointTemp.y = _currentMeshInterleaved->vertices[i].y;
    			pointTemp.z = _currentMeshInterleaved->vertices[i].z;
    			pointTemp.r = _currentMeshInterleaved->colors[i].r;
    			pointTemp.g = _currentMeshInterleaved->colors[i].g;
    			pointTemp.b = _currentMeshInterleaved->colors[i].b;
    			point_cloud_ptr->points.push_back (pointTemp);
    		}
     		/*
    		for (unsigned int i = 0; i < _currentMeshInterleaved->faces.size();i+= _currentMeshInterleaved->_verticesPerFace){
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+0]);
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+1]);
    			tempFace.vertices.push_back(_currentMeshInterleaved->faces[i+2]);
    			polygons.push_back(tempFace);
    			tempFace.vertices.clear();
    		}
    		*/
    		updateLock.unlock();
    		//pcl::PolygonMesh triangles;
    		//pcl::PCLPointCloud2 msg;
    		//sensor_msgs::PointCloud2 msg;
    		//pcl::toPCLPointCloud2 (*point_cloud_ptr, msg);
    		//pcl::toROSMsg(*point_cloud_ptr, msg);
    		//triangles.cloud = msg;
    		//triangles.polygons = polygons;
    		//-- Update Viewer
    		if (pointcloudInit) {
    			viewer->updatePointCloud(point_cloud_ptr,"visualization pc");
    			/* Mesh visualization --> slow
    			viewer->removePolygonMesh("visualization pc");
    			viewer->addPolygonMesh(triangles,"visualization pc"); */
    		} else {
    			viewer->addPointCloud(point_cloud_ptr, "visualization pc");
    			/* Mesh visualization --> slow
    			viewer->addPolygonMesh(triangles,"visualization pc"); */
    			pointcloudInit = true;
    		}
            _update = false;
            point_cloud_ptr->clear();
            //polygons.clear();
        }
    }
    viewer->removePointCloud("visualization pc",0);
    viewer->close();
}



void OnlineFusionROS::updateFusion(cv::Mat &rgbImg, cv::Mat &depthImg, CameraInfo &pose) {
//-- Adapted version of the updateSlot()-function in order to process the current images.
	if (!_threadFusion) {
	//-- Unthreaded Fusion
		_fusionActive = true;
		_frameCounter++;
		_isReady = false;
		_currentPose = pose;
		depthImg.copyTo(_currentDepthImg);
		//-- Lock visualization Mutex
		boost::mutex::scoped_lock updateLockVis(_visualizationUpdateMutex);
		//-- Add and update Map
		_fusion->addMap(depthImg,pose,rgbImg,1.0f/_imageDepthScale,_maxCamDistance);
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
		updateLockVis.unlock();
		//-- Check whether to update Visualization
		if (_frameCounter > 20) {
			_update = true;
			_frameCounter = 0;
		}
		_isReady = true;
		_fusionActive = false;
	} else {
	//-- The fusion process is threaded
		if(!_fusionThread){
			//-- If not yet initialize --> initialize fusion thread
			_fusionThread = new boost::thread(&OnlineFusionROS::fusionWrapperROS, this);
			_runFusion = true;
		}
		std::cout << "Scale = " << _imageDepthScale << std::endl;
		_currentPose = pose;
		//-- Lock and update data-queue
		boost::mutex::scoped_lock updateLock(_fusionUpdateMutex);
		_queueRGB.push(rgbImg);
		_queueDepth.push(depthImg);
		_queuePose.push(pose);
		std::cout << "There are " << _queuePose.size() << " Frames in the queue" << std::endl;
		updateLock.unlock();
		_frameCounter++;
		//--Update Mesh
		if(_newMesh){
			_newMesh = false;
			boost::mutex::scoped_lock updateLockVis(_visualizationUpdateMutex);
			if(!_currentMeshForSave) _currentMeshForSave = new MeshSeparate(3);
			if(!_currentMeshInterleaved) _currentMeshInterleaved = new MeshInterleaved(3);
			*_currentMeshInterleaved = _fusion->getMeshInterleavedMarchingCubes();
			updateLockVis.unlock();
		}
		//-- Check whether to update Visualization
		if (_frameCounter >= 20) {
			_update = true;
		}
	}
}




