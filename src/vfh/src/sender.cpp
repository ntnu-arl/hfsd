/*
 * sender.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: Ryan Fite
 */
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

int main(int argc, char** argv){
	ros::init(argc,argv,"sender");
	ros::NodeHandle n;
	ros::Publisher sender_pub = n.advertise<sensor_msgs::PointCloud2>("sender",100);
}
