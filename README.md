# Simple ground filtering demonstration in ROS and RVIZ

In this demo using RVIZ, a vehicle is approaching an obstacle. We preprocess the LiDAR point clouds by distinguishing non-ground obstacles (shown in pink) from ground (shown in orange).

Check the youtube videos for rviz recordings:

* https://www.youtube.com/watch?v=1cPNjVveXvY
* https://www.youtube.com/watch?v=AfEXqDKbUt0


### Environment

Linux Ubuntu 18.04 ROS Melodic, you are free to use other Ubuntu and ROS distributions, however they are not yet tested here.

For all terminals you open later, assuming you have sourced ros melodic by default, otherwise:
> echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

### Build

> mkdir -p ~/catkin_ws/src
> cd ~/
> git clone https://github.com/tompoek/ground_filter
> cp -r ~/ground_filter ~/catkin_ws/src/
> cd ~/catkin_ws/
> catkin_make

### Run

open terminal 1:
> roscore

open terminal 2:
> rosrun rviz rviz
in rviz UI, import the provided rviz config: rviz.rviz

open terminal 3:
> source ~/catkin_ws/devel/setup.bash
> rosrun ground_filter_pkg ground_filter

open terminal 4:
> curl -o lidar_bagfile.bag https://drive.google.com/uc?id=1I2Eg-6Dl1GyrdVGBubHunHwM2Rh9ea3n
> rosbag play lidar_bagfile.bag

switch to rviz UI and see the play (/ground and /nonground in different colours).
you should be able to reproduce everything as in the recordings.
ENJOY!


#### List of Assumptions:

- Any debris (not large rocks) is considered part of ground.
- No big objects on both sides (lidar points are noisy at |Y| > 1m).

I have been able to fast implement ground filter algorithms by only using the Z coordinates info and comparing to a threshold.
