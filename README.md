# Ground filtering + LiDAR Inertia based Odometry in a Mining Scene

Watch [video here](https://youtu.be/KCz5MW_5rRk?si=Q_fYpudGgoRbwFw-).

[FAST-LIO2 aka Fast LiDAR Inertia based Odometry](https://github.com/hku-mars/FAST_LIO/tree/ROS2) has been well tested in Hong Kong urban area with smooth ground surface.

However in this demo, the LiDAR sensor is installed on a mining vehicle which operates in a mining scene, where _**ground surface is rough and full of debris**_.

In this scenario, the odometry estimate of FAST-LIO2 is unlikely to converge.
![image](https://github.com/user-attachments/assets/ad8300eb-bb41-49ed-aec0-2dc50abb6444)

Only in some rare trials (after multiple running trials) would FAST-LIO2 manage to converge after initially noisy estimate.
![image](https://github.com/user-attachments/assets/7ecfae6c-690b-4e24-9c94-c63346e04579)

I filter some of the LiDAR point clouds that correspond to the rough ground surface, and feed the non-ground point clouds into FAST-LIO2.

This successfully helps guarantee FAST-LIO2 to converge its odometry.
![image](https://github.com/user-attachments/assets/0bae42ce-f8a3-45c1-838f-aa664f2a8c8b)

### Environment

Linux Ubuntu 24.04 ROS2 Jazzy, you are free to use other Ubuntu and ROS2 distributions, be sure to source the correct setups.

For all terminals you open later, assuming you have sourced ros2 by default, otherwise:

> echo "source /opt/ros/\<distro\>/setup.bash" >> ~/.bashrc

### Dependencies

* [ROS PCL / Point Cloud Library](http://wiki.ros.org/pcl_ros)
* [Livox ROS Driver2](https://github.com/Livox-SDK/livox_ros_driver2)

### Build

> sudo apt update -y && sudo apt install ros-\<distro\>-pcl-ros -y

> cd ~/

> git clone https://github.com/tompoek/ws_livox

> cd ~/ws_livox/

> mkdir src && cd src/

> git clone https://github.com/tompoek/ground_filter

> git clone https://github.com/Livox-SDK/livox_ros_driver2

> git clone https://github.com/tompoek/livox_lidar_converter

> git clone https://github.com/tompoek/FAST_LIO

> cd ~/ws_livox/

> chmod +x *.sh

> ./livox_build.sh

### Download logged data

Download this whole folder under ~/ws_livox: https://drive.google.com/drive/folders/15mHAeUO3cy36uBCQtKdujL-i5Evk9zhP?usp=sharing

### Run

> ./livox_launch_fast_lio_with_ground_filter.sh

Wait and see the replay in Rviz2. ENJOY!
