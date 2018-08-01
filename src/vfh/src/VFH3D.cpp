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
	ros::NodeHandle n;
	if(n.getParam("CloudOutput", topicOut)){
		ROS_INFO("CLOUD OUTPUT SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: CLOUD OUTPUT SET INCORRECTLY. SETTING TO DEFAULT");
		topicOut = "window_points";
	}
	if(n.getParam("OdometryInput", topicOdom)){
		ROS_INFO("ODOMETRY INPUT SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: ODOMETRY INPUT SET INCORRECTLY. SETTING TO DEFAULT");
		topicOdom = "msf_core/odometry";
	}
	uint32_t queue_size = 5;
	string topic = n.resolveName("points");
	pubHandler handler = pubHandler(n,topicOut, 100);
	ros::Subscriber subPoints = n.subscribe<pcl::PointCloud<pcl::PointXYZ> >(topic,queue_size,&pubHandler::messageReceivedCloud, &handler);
	ros::Subscriber subOdometry = n.subscribe<nav_msgs::Odometry>(topicOdom,queue_size,&pubHandler::messageReceivedPose, &handler);
	ros::spin();
	return 0;
}

