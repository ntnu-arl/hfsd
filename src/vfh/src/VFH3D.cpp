//============================================================================
// Name        : VFH3D.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description : Primary Node for the VFH3D algorithm.
//============================================================================


#include "ros/ros.h"
#include "pubHandler.cpp"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"


int main(int argc, char** argv) {
	ros::init(argc,argv,"vfh3d");
	ros::NodeHandle n;
	//ros::NodeHandle nh;
	std::string topic = n.resolveName("velodyne_points");
	uint32_t queue_size = 5;
	pubHandler handler = pubHandler(n,"vfh", 100);
	ros::Subscriber sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic,queue_size,&pubHandler::messageReceivedCloud, &handler);
	ros::spin();
	// TODO Create Call back system for Odometry
	// TODO Write transformations for point Clouds
	// TODO Write Code for Cloud Queuing and Pre-processing Information
	// TODO Use OpenCV to Process Histogram
	// TODO Output Vectors
	return 0;
}

