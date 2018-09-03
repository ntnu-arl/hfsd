//============================================================================
// Name        : VFH3D.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description : Initializer for the detection algorithm. Sets up the ROS
//               subscribers and sets up the pubHandler object.
//============================================================================


#include "ros/ros.h"

#include "hfsd/pubHandler.h"

int main(int argc, char** argv) {
	ros::init(argc,argv,"hfsd");

	ros::NodeHandle nh("hfsd");

	pubHandler handler = pubHandler(nh, "window_points", 100);

	ros::spin();
	return 0;
}

