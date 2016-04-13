**fastfusion_ros**
===================
ROS-wrapper around TUM's fastfusion algorithm (https://github.com/tum-vision/fastfusion)
The user can decide whether to either use the original reconstruction algorithm for reconstruction by fusing all available data, or to only keep a local reconstruction consisting of the area observed within a certain time-window.
If some (pixelwise) uncertainty is available (for example when using a ToF camera) the user can select to perform the fusion in a weighted fashion taking the sensor noise into account.

**Installation:**
---
Required Packages:
* catkin_simple  : ```git clone https://github.com/ethz-asl/catkin_simple.git```
When using okvis for the pose estimation (requires VI-Sensor)
* okvis_ros      : ```git clone https://github.com/ethz-asl/okvis_ros```

Installation of fastfusion_ros:

```
git clone https://github.com/karrerm/fastfusion_ros
cd ..
catkin_make -DCMAKE_BUILD_TYPE=RELEASE
```
**Usage:**
---
###**Parameters**
```
  --threadFusion
    run the fusion in a separate thread
  --threadMeshing
    run the meshing in a separate thread (if possible recommended)
  --imageScale
    factor to convert from integer to metric value (e.g. 1000 for RealSense sensor)
  --scale
    sidelength of the voxel in the finest resolution
  --distanceThreshold
    truncation parameter of the SDF
  --saveMesh
    whether the mesh should be saved when exiting the node
  --fileLocation
    if saveMesh=true, where to save the mesh
  --use_vicon_pose
    use vicon measurements for the pose input (uses different extrinsic transformation)
  --world_id
    name of the world frame
  --tracker_id
    name of the reference frame of the SLAM
  --cam_id
    name of the frame for the depth measurement
  --decay_time
    time-window size in seconds (for full reconstruction set to <=0.0)
  --use_pmd
    use a ToF camera for the reconstruction (e.g. Picoflexx)
  --depth_noise
    perform the reconstruction using pixelwise-noise values (only if use_pmd=true)
  --use_pcl_visualizer
    show the current reconstruction using a pcl-visualizer
```
There are more paramaters which can be adapted, however, these are hardcoded in the file fusion/include/fusion/definitions.h. The most used are
```
  --MAXCAMDISTANCE
    Defines the maximal range up to which measurements are considered in the reconstruction
  --MIN_WEIGHT_FOR_SURFACE
    Minimal weight of a voxel in order to be triangulated. Higher weights make it more robust to outliers, but delay the reconstruction.
  --REFERENCE_NOISE
    Defines the scaling of the noise-adaptive weighting (only for ToF)
  --MIN_NOISE_LEVEL
    Minimal uncertainty. Measurements with less noise are considered with MIN_NOISE_LEVEL noise.
```
Note that changing these parameters requires re-compiling the code in order to be applied.

###**Run the system (fastfusion_node_dataset.launch)**
Intended for the use with a dataset including pose data for the sensor pose (e.g. Vicon poses).
In the launch file set the camera_name matching your sensor naming and adapt the renaming. Note that the images are only considered for the case when using a ToF camera, otherwise only the registered point-cloud topic is used. Set the correct path to the parameterfile as well as the calibration file and the bag-file containing your data. Then run the system using
```
roslaunch fastfusion_ros fastfusion_node_dataset.launch
```

###**Run the system with OKVIS (fastfusion_node_slam.launch)**
Intended for the use of a multisensor setup with a depth sensor mounted on a VI-sensor. In the launch file set the camera_name matching your sensor naming and adapt the renaming. Note that the images are only considered for the case when using a ToF camera, otherwise only the registered point-cloud topic is used. Set the correct path to the parameterfile as well as the calibration file and the bag-file containing your data. Setup the correct calibration for Okvis. Then run the system using
```
roslaunch fastfusion_ros fastfusion_node_slam.launch
```

**Remarks**
---
* When using the system performing the full data fusion (decay_time <=0.0), make sure you have either ground truth poses (Vicon) or SLAM poses with little drift. Otherwise the reconstruction will be bad (e.g. multiple reconstructions).
* Setting the decay_time to large values (> 4 sec) and fast motions will result in increase the computation time.
* If you use an RGB-D camera, make sure that the data is available as a colored point cloud, which is the expected input.
* Setting the voxel-resolution (scale) to small values (especially less than 0.005) will increase the computation time as well as the memory requirement.
* For the case with time-window based reconstruction, the current model (vertices) are available as a rostopic (fastfusion/pointCloud).
