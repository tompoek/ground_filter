# Ground filtering demonstration in ROS2 and RViz2

In this demo, a mining vehicle equipped with LiDAR sensor is operating in a mining scene. 

I filter the LiDAR point clouds by distinguishing non-ground points (shown in pink) from ground points (shown in orange). 

![image](https://github.com/user-attachments/assets/56063519-433f-498a-a950-3ef27c93baf6)

Check the youtube videos for rviz recordings:

* Using height (Z) thresholding: https://youtu.be/_BsraDXbNPA
* Using RANSAC (Random Sampling Consensus) algorithm: https://youtu.be/YCeWqTxxGiU

### Environment

Linux Ubuntu 22.04 ROS2 Humble, you are free to use other Ubuntu and ROS2 distributions, be sure to source the correct setups.

For all terminals you open later, assuming you have sourced ros2 by default, otherwise:

> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

### Dependencies

* [ROS PCL / Point Cloud Library](http://wiki.ros.org/pcl_ros)

### Build

> sudo apt update -y && sudo apt install ros-\<distro\>-pcl-ros -y

> mkdir -p ~/ros2_ws/src

> cd ~/ros2_ws/src/

> git clone https://github.com/tompoek/ground_filter

> cd ~/ros2_ws/

> colcon build --packages-select ground_filter_pkg

### Run

In terminal 1:

> rviz2

Once rviz2 UI is opened, import the provided rviz2 config: rviz.rviz

In terminal 2:

> . install/setup.bash

Try the RANSAC implementation:

> ros2 run ground_filter_pkg ground_filter_ransac

Or, try the Z thresholding implementation:

> ros2 run ground_filter_pkg ground_filter

In terminal 3:

Download the ROS2 bag file simple.db3 from this link: https://drive.google.com/uc?id=1K7KTnqZr5pdYHFOYX8IQBcJLRl242YRp

> ros2 bag play simple.db3

Or, download the ROS2 bag file advanced.db3 from this link: https://drive.google.com/uc?id=12DoGBU6A0A8leMyhqwXASGQqoOyRcbTR

> ros2 bag play advanced.db3

Switch to rviz2 UI and see the replay (/ground and /nonground in different colours). You should be able to reproduce everything as in the recordings. ENJOY!

### What's done and what's to-do:

- DONE: Use height (Z) thresholding to filter out ground.
- DONE: Make the height (z) threshold tunable.
- DONE: Apply RANSAC to filter ground by calling PCL library.
- DONE: Make the distance threshold in RANSAC tunable.
- TODO: Expand other perception tasks e.g. object detection using YOLO3D
- TODO: Expand other perception tasks e.g. mapping and localization using SLAM
