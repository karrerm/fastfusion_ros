**fastfusion_ros**
===================
ROS-wrapper around TUM's fastfusion algorithm (https://github.com/tum-vision/fastfusion)
The user can decide whether to either use the original reconstruction algorithm for reconstruction by fusing all available data, or to only keep a local reconstruction consisting of the area observed within a certain time-window.
If some (pixelwise) uncertainty is available (for example when using a ToF camera) the user can select to perform the fusion in a weighted fashion taking the sensor noise into account.

**Installation:**
---
Required Packages:
* catkin_simple  : ```git clone https://github.com/ethz-asl/catkin_simple.git```

Installation of fastfusion_ros:

```
git clone https://github.com/karrerm/fastfusion_ros
cd ..
catkin_make -DCMAKE_BUILD_TYPE=RELEASE
```
**Usage**
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


