/*
 * listener.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: Ryan Fite
 */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void messageReceived(const sensor_msgs::PointCloud2::ConstPtr& msg){
	ROS_INFO("I heard: A PointCloud");
}

int main(int argc, char** argv){
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("sensor_msgs/PointCloud2",100,messageReceived);
	ros::spin();
	return 0;
}

