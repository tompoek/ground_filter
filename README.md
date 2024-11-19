# Ground filtering + LiDAR Inertia based Odometry demonstration in ROS2 

In this demo, a mining vehicle equipped with LiDAR sensor is operating in a mining scene. 

I filter the LiDAR point clouds by distinguishing non-ground points from ground points. 

Then, [FAST-LIO2](https://github.com/tompoek/FAST_LIO) will update LiDAR-Inertia-based odometry.

Snapshot using RANSAC algorithm to filter ground, and FAST-LIO2 to update odometry:

![image](https://github.com/user-attachments/assets/ca43448a-fecf-4a10-92be-666b07d15299)

### Environment

Linux Ubuntu 24.04 ROS2 Jazzy, you are free to use other Ubuntu and ROS2 distributions, be sure to source the correct setups.

For all terminals you open later, assuming you have sourced ros2 by default, otherwise:

> echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

### Dependencies

* [ROS PCL / Point Cloud Library](http://wiki.ros.org/pcl_ros)
* [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)

### Build

> sudo apt update -y && sudo apt install ros-\<distro\>-pcl-ros -y

> mkdir -p ~/ws_livox/src

> cd ~/ws_livox/src/

> git clone https://github.com/Livox-SDK/livox_ros_driver2

> git clone https://github.com/tompoek/ground_filter

> git clone https://github.com/tompoek/FAST_LIO

> cd ~/ws_livox/src/FAST_LIO

> git submodule init

> git submodule update --remote --recursive

> cd ~/ws_livox/

> colcon build --symlink-install --packages-select livox_ros_driver2 ground_filter_pkg fast_lio

### Run

In one terminal:

> . install/setup.bash

Try the RANSAC implementation:

> ros2 run ground_filter_pkg ground_filter_ransac --ros-args -p distance_threshold:=0.1

In another terminal:

Run FAST-LIO2 with its Rviz2 visualization config.

> . install/setup.bash

> ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml

In another terminal:

Download the ROS2 bag file simple_lidar_custom from this link: https://drive.google.com/drive/folders/1OR1Ateea6Wpw9oPVopagO8DaP0R6-AN4?usp=sharing

> . install/setup.bash

> ros2 bag play simple_lidar_custom/

### What's done and what's to-do:

- DONE: Use height (Z) thresholding to filter out ground.
- DONE: Make the height (z) threshold tunable.
- DONE: Apply RANSAC to filter ground by calling PCL library.
- DONE: Make the distance threshold in RANSAC tunable.
- TODO: Expand other perception tasks e.g. object detection using YOLO3D
- TODO: Expand other perception tasks e.g. mapping and localization using SLAM
