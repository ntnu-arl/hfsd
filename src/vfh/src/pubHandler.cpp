//============================================================================
// Name        : pubHandler.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================

#include "pubHandler.h"
//constructor for pubHandler class
pubHandler::pubHandler(ros::NodeHandle n, const std::string& s, int num){
	_pub = n.advertise<sensor_msgs::PointCloud2>(s,num);
	_queueSize = 5;
}

//used to publish data from the publisher and check for errors
void pubHandler::publish(pcl::PointCloud<pcl::PointXYZ> msg){
	try{
		_pub.publish(msg);
		ROS_INFO("Success");
	}catch(...){
		ROS_INFO("An error has occurred");
	}
}

//callback function for the subscriber. performs all of the enqueuing and dequeuing;
void pubHandler::messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	ROS_INFO("Recieving Cloud...");
	_data = *msg;
	//ROS_INFO("Check 2");

	//ROS_INFO("Check 3");
	//ROS_INFO("Check 4");
}

//this recieves the odometry for the program to create the sliding window
void pubHandler::messageReceivedPose(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	ROS_INFO("Receiving Odometry...");
	if(_window.size() == _queueSize){
		_window.pop_back();
	}
	_window.push_front(_data);
	//ROS_INFO("Check 2");
	pcl::PointCloud<pcl::PointXYZ> ptCloudScene = pcl::PointCloud<pcl::PointXYZ>(_preprocessing(_window));
	//ROS_INFO("Check 3");
	this->publish(ptCloudScene);
	//ROS_INFO("Check 4");
}

//this is the pre-processing step that transforms and filters the point cloud queue
pcl::PointCloud<pcl::PointXYZ> pubHandler::_preprocessing(std::deque<pcl::PointCloud<pcl::PointXYZ> > window){
	pcl::PointCloud<pcl::PointXYZ> ptCloudScene = pcl::PointCloud<pcl::PointXYZ>(window[0]);
	for(int i = 1; i<window.size();i++){
		ptCloudScene+=window[i];
	}
	ROS_INFO("SUCCESS");
	return ptCloudScene;
}

//this is the primary algorithm used to determine the best vectors for travel
std::map<std::string,std::vector<trajectory> > pubHandler::_vfh3D(){
	std::map<std::string,std::vector<trajectory> > trajVec;
	return trajVec;

}
