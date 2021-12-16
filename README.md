# ELEC3210-Group7
#ELEC3210 project Group7 -HUI
#CopyRight 2021
#Built on ROS Noetic; Ubuntu 20.04; CoppeliaSim 4.2.0
# Copy the env.ttt to the folder

# Create a work space
http://wiki.ros.org/catkin/Tutorials/create_a_workspace
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
#To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in. 
$ echo $ROS_PACKAGE_PATH

# Create a pkg
$ catkin_create_pkg [pkg_name] []std_msgs rospy roscpp sensor_msgs visualization_msgs cv_bridge geometry_msgs
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash

##steps：
# roscore
$ roscore

# launch project
$ cd ~/catkin_ws
$ catkin_make
$ . ~/catkin_ws/devel/setup.bash
$ roslaunch begin.launch

# launch coppeliasim
$ cd ~/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04
$ ./coppeliaSim.sh

# print tf frams
$ rosrun tf view_frames
$ evince frames.pdf
