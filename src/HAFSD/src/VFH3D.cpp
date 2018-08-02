//============================================================================
// Name        : VFH3D.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description : Initializer for the detection algorithm. Sets up the ROS
//               subscribers and sets up the pubHandler object.
//============================================================================


#include "ros/ros.h"
#include "pubHandler.cpp"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv) {
	ros::init(argc,argv,"vfh3d");

	std::string topicOdom;
	std::string topicOut;
	ros::NodeHandle n("vfh3d");

	uint32_t queue_size = 5;
	string topic = n.resolveName("/cloudTransformer/points");
	pubHandler handler = pubHandler(n,"window_points", 100);
	ros::Subscriber subPoints = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic,queue_size,&pubHandler::messageReceivedCloud, &handler);
	ros::Subscriber subOdometry = n.subscribe<nav_msgs::Odometry>("odometry",queue_size,&pubHandler::messageReceivedPose, &handler);
	ros::spin();
	return 0;
}

