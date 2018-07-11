//============================================================================
// Name        : pubHandler.h
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================

#ifndef PUBHANDLER_H_
#define PUBHANDLER_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
#include "Eigen/Geometry"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include "trajectory.h"

class pubHandler{
public:
	pubHandler(ros::NodeHandle n, const std::string& s, int num);
	void messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
	void messageReceivedPose(const nav_msgs::Odometry::ConstPtr& msg);
	void publish(pcl::PointCloud<pcl::PointXYZ> msg);
	Eigen::Quaterniond differenceOfQuat(Eigen::Quaterniond start, Eigen::Quaterniond end);
	Eigen::Vector3d differenceOfVec(Eigen::Vector3d start, Eigen::Vector3d end);
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr& getData();

private:
	/* Private Variables*/
	ros::Publisher _pub;
	pcl::PointCloud<pcl::PointXYZ> _data;
	std::deque<pcl::PointCloud<pcl::PointXYZ> > _window;
	std::deque<nav_msgs::Odometry> _odomWindow;
	int _queueSize;
	/*Private Functions*/
	std::map<std::string,std::vector<trajectory> > _vfh3D();
	pcl::PointCloud<pcl::PointXYZ> _preprocessing(std::deque<pcl::PointCloud<pcl::PointXYZ> > window,std::deque<nav_msgs::Odometry> odomWindow);
};

#endif /* PUBHANDLER_H_ */
