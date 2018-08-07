# HFSD
History-Aware Free Space Detection for Efficient Autonomous Exploration using Aerial Robots in ROS
## Overview
This package is capable of proposing appropriate directions for exploration by utilizing a sliding-window history of the robot’s pose estimates and the depth measurements of the environment to identify the directions of probable unobservable free space in enclosed environments. More specifically, the method finds areas of sparse sensor returns near the end of the robot’s perception in the vicinity of areas with no sensor returns and determines the directions to these areas to be the probable directions of free space due to the consistency of the lack of sensor readings with the shape of the environment. In tunnel like environments, the probable direction of free space is likely to be the center of the tunnel, while in other cases it may be the center of an open room or closer to a wall that is more heavily observed by the robot’s sensors this is because we cannot assume that merely the lack of sensor readings is the same as free space. This method can be used to assist a path planner by determining the directions of probable free space for efficient exploration.
## Installation
Depends on a full installation of ROS Kinetic.

* Install and initialize ROS Kinetic desktop full, additional ROS packages, catkin-tools:

```sh
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-joy ros-kinetic-octomap-ros python-wstool python-catkin-tools
  $ sudo rosdep init
  $ rosdep update
  $ source /opt/ros/kinetic/setup.bash
```
* Initialize catkin workspace:
```sh
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws
  $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
  $ catkin init  # initialize your catkin workspace
```
* Get the Package
```sh
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/unr-arl/HFSD.git
```
* Build the workspace
```sh
  $ cd ~/catkin_ws
  $ catkin build 
```
### Test Installation
* Open a terminal and type the following:
```sh
  $ cd ~/catkin_ws
  $ source devel/setup.bash
  $ roslaunch hfsd hfsd_test.launch
``` 
* In a second terminal type the following:
```sh
  $ cd ~/catkin_ws/src/hfsd
  $ rosbag play test.bag
```
* Rviz should show something similar to this:

## Basic Usage
### Launching the Package
To launch the program:
* Modify the launch file to the desired parameters.
* Run the following:
```sh
  $ cd ~/catkin_ws
  $ source devel/setup.bash
  $ roslaunch hfsd hfsd.launch
``` 
### Subscribed Topics
- **`/cloudTransformer/inputcloud`** of type `sensor_msgs/PointCloud2`. This should be remapped to a topic that publishes point clouds. It transforms `PointCloud2` messages to `PCL::PointCloud<<PCL::PointXYZ>>` data structures internally.
- **`/hfsd/odometry`** of type `nav_msgs/Odometry`. The odometry should be in the world frame. This is used to create the sliding window of `PointCloud2` messages.
### Published Topics
- **`/hfsd/OdomOut`** of type `nav_msgs/Odometry`. This simply shows the odometry being used by the algorithm.
- **`/hfsd/open/contours`** of type `sensor_msgs/Image` This publishes a 2D image of color coded contours extracted by the algorithm. The contours also have dots on them representing their centroids.
- **`/hfsd/open/image`** of type `sensor_msgs/Image`. This publishes a 2D image of the grayscale matrix that is used to determine directions of free space.
- **`/hfsd/visualization_marker`** of type `visualization_msgs/MarkerArray`. This publishes the visualization of the directions of free space produced by the algorithm.
- **`/hfsd/window_points`** of type `sensor_msgs/PointCloud2`. This publishes the sliding window of point clouds.
### Paramaters
| Parameter             | Description                                                                     |
| --------------------- | ------------------------------------------------------------------------------- |
| `sensor_frame`        | The internal reference frame associated with the sensor                         |
| `base_frame`          | The chosen base_frame typically `base_link`                                     |
| `markerSkip`          | Number of frames before next publishing on `/hfsd/visualization_marker`         |
| `markerMag`           | Length of the visualization markers                                             |
| `markerLifetime`      | Lifetime of the visualization markers                                           |
| `arrowHeadDiameter`   | Diameter of the arrow head for the visualization markers                        |
| `arrowShaftDiameter`  | Diameter of the arrow shaft fot the visualization markers                       |
| `arrowHeadLength`     | Length of the arrow head for the visualization markers                          |
| `HREZ`                | The number of sectors that exist along the azimuth of the spherical matrix      |
| `VREZ`                | The number of sectors that exist along the elevation of the spherical matrix    |
| `queueSize`           | The number of consecutive point clouds aligned in the sliding window            |
| `skipFrames`          | The number of point cloud messages to skip before adding a message to the queue |
| `areaRestricter`      | Filters out all found contours below this number                                |
| `heightSplitter`      | If a contour has more height than this param it will be split into parts        |
| `minDistRejFilterVal` | Sets spherical sectors below this number to 0                                   |
| `voxelSize`           | Size of the initial voxel grid filter                                           |
| `uniformGrid`*        | Enables/Disables secondary voxel grid                                           |
| `voxelUniformSize`*   | Size of the secondary voxel grid                                                |
| `box`*                | Enables/Disables the box filter                                                 |
| `boxSizeX`*           | Changes the X value of the box filter size. Must be odd or 0                    |
| `boxSizeY`*           | Changes the Y value of the box filter size. Must be odd or 0                    |
| `blur`*               | Enables/Disables the gaussian blur                                              |
| `blurSizeX`*          | Changes the X value of the gaussian blur size. Must be odd or 0                 |
| `blurSizeY`*          | Changes the Y value of the gaussian blur size. Must be odd or 0                 |
| `blurSigmaX`*         | Changes the X value of the gaussian blur sigma. Must be odd or 0                |
| `blurSigmaY`*         | Changes the Y value of the gaussian blur sigma. Must be odd or 0                |
| `median`              | Enables/Disables the gaussian blur                                              |
| `medianSize`          | Changes the X value of the gaussian blur size. Must be odd                      |
| `intensityOffset`     | The number that modifies intensity values for grayscale.                        |
| `dilate`              | Enables/Disables dilation                                                       |
| `dilationIterations`  | Number of dilations                                                             |
| `erode`*              | Enables/Disables erosion                                                        |
| `erosionIterations`*  | Number of erosions                                                              |
| `showTiming`          | Enable/Disable timing messages                                                  |
| `showDebugMessages`   | Enable/Disable debug messages                                                   |

\* These parameters are typically disabled and may cause unknown effects when enabled.

