# HFSD
## Overview
This package is capable of proposing appropriate directions for exploration by utilizing a sliding-window history of the robot’s pose estimates and the depth measurements of the environment to identify the directions of probable unobservable free space in enclosed environments. More specifically, the method finds areas of sparse sensor returns near the end of the robot’s perception in the vicinity of areas with no sensor returns and determines the directions to these areas to be the probable directions of free space due to the consistency of the lack of sensor readings with the shape of the environment. In tunnel like environments, the probable direction of free space is likely to be the center of the tunnel, while in other cases it may be the center of an open room or closer to a wall that is more heavily observed by the robot’s sensors this is because we cannot assume that merely the lack of sensor readings is the same as free space. This method can be used to assist a path planner by determining the directions of probable free space for efficient exploration.
## Installation
Depends on a full installation of ROS Kinetic. Installation instructions for ROS Kinetic can be found here: http://wiki.ros.org/kinetic/Installation

This package is built using catkin tools.
## Basic Usage

### Subscribers
- sub (pcl::PointCloud<<pcl::PointXYZ>>)
- subod (nav_msgs::Odometry)
### Publishers
### Paramaters
