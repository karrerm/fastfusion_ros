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
'''
git clone https://github.com/karrerm/fastfusion_ros
cd ..
catkin_make -DCMAKE_BUILD_TYPE=RELEASE
'''

**Usage**
---



