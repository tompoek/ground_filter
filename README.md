# Simple ground filtering demonstration in ROS2 and RViz2

In a demo combined with [FAST-LIO2](https://github.com/tompoek/FAST_LIO), a vehicle is approaching an obstacle. 

I preprocess the LiDAR point clouds by distinguishing non-ground obstacles (shown in pink) from ground (shown in orange). 

Then, FAST-LIO2 will update LiDAR-inertia-based odometry.

Check the youtube videos for rviz recordings:

* Using height (Z) thresholding: https://youtu.be/_BsraDXbNPA
* Using RANSAC (Random Sampling Consensus) algorithm: https://youtu.be/YCeWqTxxGiU

### Environment

Linux Ubuntu 24.04 ROS2 Jazzy, you are free to use other Ubuntu and ROS2 distributions, be sure to source the correct setups.

For all terminals you open later, assuming you have sourced ros2 by default, otherwise:

> echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

### Dependencies

[Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)

### Build

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
