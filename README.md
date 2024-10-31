# Simple ground filtering demonstration in ROS2 and RViz2

In this demo using RViz2, a vehicle is approaching an obstacle. I preprocess the LiDAR point clouds by distinguishing non-ground obstacles (shown in pink) from ground (shown in orange).

![image](https://github.com/user-attachments/assets/56063519-433f-498a-a950-3ef27c93baf6)

Check the youtube videos for rviz recordings: (videos below were ROS1 Melodic implementation)

* https://www.youtube.com/watch?v=1cPNjVveXvY
* https://www.youtube.com/watch?v=AfEXqDKbUt0

### Environment

Linux Ubuntu 22.04 ROS2 Humble, you are free to use other Ubuntu and ROS2 distributions, be sure to source the correct setups.

For all terminals you open later, assuming you have sourced ros2 by default, otherwise:

> echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

### Build

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

> ros2 run ground_filter_pkg ground_filter

In terminal 3:

Download the ROS2 bag file from this link and rename it e.g. to "lidar_bagfile.db3": https://drive.google.com/uc?id=12DoGBU6A0A8leMyhqwXASGQqoOyRcbTR

> ros2 bag play lidar_bagfile.bag

Switch to rviz2 UI and see the replay (/ground and /nonground in different colours). You should be able to reproduce everything as in the recordings. ENJOY!


### List of Assumptions:

- Any debris (not large rocks) is considered part of ground.
- No big objects on both sides (lidar points are noisy at |Y| > 1m).

### Ongoing works:

- DONE: Use height (Z) thresholding to filter out ground.
- TODO: Make the z threshold tunable.
- TODO: Apply more advanced techniques / Use third party libs.
