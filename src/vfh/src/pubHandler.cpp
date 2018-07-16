//============================================================================
// Name        : pubHandler.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================

#include "pubHandler.h"
//using namespace std;
//constructor for pubHandler class
pubHandler::pubHandler(ros::NodeHandle n, const std::string& s, int num){
	_pub = n.advertise<sensor_msgs::PointCloud2>(s,num);
	image_transport::ImageTransport it(n);
	_pubImage = it.advertise("open/image", 1);
	_pubContours = it.advertise("open/contours", 1);
	_queueSize = 7;
	_count = 2;
	//_HREZ = 180;
	//_VREZ =90;
}

//used to publish data from the publisher and check for errors
void pubHandler::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr msg){
	try{
		_pub.publish(*msg);
		ROS_INFO("Success");
	}catch(...){
		ROS_INFO("An error has occurred");
	}
}

//callback function for the subscriber. performs all of the enqueuing and dequeuing;
void pubHandler::messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	ROS_INFO("Recieving Cloud...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>(*msg));
	_data = temp;
	//ROS_INFO("Check 2");

	//ROS_INFO("Check 3");
	//ROS_INFO("Check 4");
}

//this recieves the odometry for the program to create the sliding window
void pubHandler::messageReceivedPose(const nav_msgs::Odometry::ConstPtr& msg){
	if(_count == 2){
	ROS_INFO("Receiving Odometry...");
	nav_msgs::Odometry odomData = *msg;
	if(_window.size() == _queueSize){
		_window.pop_back();
		_odomWindow.pop_back();
	}
	_window.push_front(_data);
	_odomWindow.push_front(odomData);
	//ROS_INFO("Check 2");
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudScene(new pcl::PointCloud<pcl::PointXYZ>(_preprocessing(_window, _odomWindow)));
	_vfh3D(ptCloudScene);
	//ROS_INFO("Check 3");
	this->publish(ptCloudScene);
	_count = 0;
	}else{
		ROS_INFO("SKIPPING...");
		_count++;
	}
	//ROS_INFO("Check 4");
}

//takes in two quaternions and gives back the quaternion that rotates from the starting quaternion to the next quaternion
Eigen::Quaterniond pubHandler::differenceOfQuat(Eigen::Quaterniond start, Eigen::Quaterniond end){
	Eigen::Quaterniond difference =  end * start.inverse();
	return difference;
}
Eigen::Vector3d pubHandler::differenceOfVec(Eigen::Vector3d start, Eigen::Vector3d end){
	Eigen::Vector3d difference = start-end;
	return difference;
}


//this is the pre-processing step that transforms and filters the point cloud queue
pcl::PointCloud<pcl::PointXYZ> pubHandler::_preprocessing(std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr > window, std::deque<nav_msgs::Odometry> odomWindow){
	ROS_INFO("Preprocessing...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudScene(new pcl::PointCloud<pcl::PointXYZ>(*window[0]));
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudSceneFiltered(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Vector3d endV;
	Eigen::Vector3d startV;
	Eigen::Quaterniond endQ;
	Eigen::Quaterniond startQ;

	tf2::fromMsg(odomWindow[0].pose.pose.position, endV);
	tf2::fromMsg(odomWindow[0].pose.pose.orientation,endQ);
	//endV = endV.transpose() * endQ.toRotationMatrix();
	for(int i = 1; i<window.size();i++){
		tf2::fromMsg(odomWindow[i].pose.pose.position, startV);
		tf2::fromMsg(odomWindow[i].pose.pose.orientation,startQ);
		Eigen::Quaterniond differenceQ = differenceOfQuat(startQ,endQ);
		//startV = startV.transpose() * endQ.toRotationMatrix();
		Eigen::Vector3d differenceV = differenceOfVec(startV,endV);
		differenceV = differenceV.transpose() * endQ.toRotationMatrix();
		Eigen::Affine3d affine = Eigen::Affine3d::Identity();;
		affine.translation() << differenceV[0],differenceV[1],differenceV[2];
		affine.rotate(differenceQ.toRotationMatrix());
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*window[i],*transformedCloud,affine);
		*ptCloudScene += *transformedCloud;
	}
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(ptCloudScene);
	sor.setLeafSize (0.075f, 0.075f, 0.075f);
	sor.filter(*ptCloudSceneFiltered);
	//*ptCloudSceneFiltered->points[1].data;
	ROS_INFO("Preprocessing Complete");
	return *ptCloudSceneFiltered;
}

cv::Mat pubHandler::_radmatrix(std::vector<pubHandler::sector> points){
	ROS_INFO("Creating RadMatrix...");
	cv::Mat radmat(2*_HREZ, _VREZ,CV_8UC1, cv::Scalar(0));
	double tempMat[_HREZ][_VREZ] = {0.0};

	for(int row = 0; row<points.size();row++){
		//std_msgs::String s;
		string s = "Azimuth: " + to_string(points.at(row).a) + "Elevation: "+ to_string(points.at(row).e) + "Radius: "+ to_string(points.at(row).r);
		//ROS_INFO_STREAM(s);//<<points[row];
		if(points.at(row).e >= 0 && points.at(row).e < _VREZ){
			if((tempMat[points.at(row).a][points.at(row).e] > points.at(row).r || tempMat[points.at(row).a][points.at(row).e]<=0.4)){
				tempMat[points.at(row).a][points.at(row).e] = points.at(row).r;
			}
		}
	}
	ROS_INFO("Check 1");
	double max = 0;
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			if(val>max){
				max=val;
			}
		}
	}

	ROS_INFO("Check 2");
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			tempMat[row][col] = 255.0*(val/max);
		}
	}
	double wrappedMat[2*_HREZ][_VREZ]= {0.0};
	for(int row = _HREZ/2; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			wrappedMat[row - _HREZ/2][col] = val;
		}
	}
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			wrappedMat[row + _HREZ/2][col] = val;
		}
	}
	for(int row = 0; row<_HREZ/2; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			wrappedMat[row + 3*_HREZ/2][col] = val;
		}
	}
	ROS_INFO("Check 3");
	for(int row = 0; row<2*_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			radmat.at<uchar>(row,col) = (uchar)(wrappedMat[row][col]);
		}
	}


	//TODO: Write the sectorization Methods
	//TODO: Write Intensity value matrix in mat form
	ROS_INFO("RadMatrix Complete");
	return radmat;
}
//this is the primary algorithm used to determine the best vectors for travel
std::map<std::string,std::vector<trajectory> > pubHandler::_vfh3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	ROS_INFO("Beginning VFH...");
	cv::RNG _rng(12345);
	std::map<std::string,std::vector<trajectory> > trajVec;
	std::vector<std::vector<double> > points = _extractPointsFromCloud(cloud);
	points = _convertToSpherical(points);
	std::vector<pubHandler::sector> sectors = _sectorize(points);
	cv::Mat grayMat(_radmatrix(sectors));
	cv::GaussianBlur(grayMat,grayMat,cv::Size_<int>(3,5),3,5);
	ROS_INFO("Check 1");
	cv::Mat binaryImage(2*_HREZ, _VREZ,CV_8UC1, cv::Scalar(0));
	cv::threshold(grayMat, binaryImage, 0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
	cv::Mat cont(binaryImage);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cv::findContours( cont, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > contoursfiltered;
	for(int i = 0; i<contours.size(); i++){
		if(cv::contourArea(contours[i])>15.0){
			contoursfiltered.push_back(contours[i]);
		}
	}
	Mat drawing = Mat::zeros( binaryImage.size(), CV_8UC3 );
	for( size_t i = 0; i< contoursfiltered.size(); i++ ){
		Scalar color = Scalar( _rng.uniform(0, 255), _rng.uniform(0,255), _rng.uniform(0,255) );
		drawContours( drawing, contoursfiltered, (int)i, color, FILLED, 8, hierarchy, 0, Point() );
	}

	//cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
	ROS_INFO("Check 2");
	sensor_msgs::ImagePtr msgC = cv_bridge::CvImage(std_msgs::Header(), "rgb8", drawing).toImageMsg();
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", binaryImage).toImageMsg();
	_pubImage.publish(msg);
	_pubContours.publish(msgC);
	//cv::imshow("Display Window",grayMat.);
	//TODO process radmatrix into binary matrix using opencv's gaussian adaptive threshold
	//TODO Find Centroids of binary matrix regions.
	//TODO Classify the trajectory vectors in the map
	ROS_INFO("VFH Complete");
	return trajVec;

}
std::vector<std::vector<double> > pubHandler::_convertToSpherical(std::vector<std::vector<double> > xyz){
	//TODO::convert vector of n rows and three columns from cartesian to spherical
	ROS_INFO("Converting to Spherical...");
	for(int i = 0; i< xyz.size();i++){
		double x=xyz.at(i).at(0);
		double y=xyz.at(i).at(1);
		double z=xyz.at(i).at(2);
		xyz.at(i).at(2)=sqrt(pow(x,2) + pow(y,2) + pow(z,2));
		xyz.at(i).at(0)=atan2(y,x);
		xyz.at(i).at(1)=acos(z/xyz.at(i).at(2));

	}
	ROS_INFO("Conversion Complete");
	return xyz;
}

std::vector<std::vector<double> > pubHandler::_convertToCartesian(std::vector<std::vector<double> > aer){
		//TODO::convert vector of n rows and three columns from cartesian to spherical
	for(int i = 0; i< aer.size();i++){
		double a=aer.at(i).at(0);
		double e=aer.at(i).at(1);
		double r=aer.at(i).at(2);
		aer.at(i).at(0)=r*sin(e)*cos(a);
		aer.at(i).at(1)=r*sin(e)*sin(a);
		aer.at(i).at(2)=r*cos(e);
	}
	return aer;

}
std::vector<std::vector<double> > pubHandler::_extractPointsFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	//TODO: get vector nx3 vector of XYZ coordinates from point cloud
	ROS_INFO("Extracting Points...");
	std::vector<std::vector<double> > points;

	for(int i = 0; i<cloud->size();i++){
		std::vector<double> temp;
		temp.push_back(cloud->points[i].x);
		temp.push_back(cloud->points[i].y);
		temp.push_back(cloud->points[i].z);
		points.push_back(temp);
	}
	ROS_INFO("Points Extracted");
	return points;
}

std::vector<pubHandler::sector> pubHandler::_sectorize(std::vector<std::vector<double> > aer){
	ROS_INFO("Sectorizing...");
	std::vector<pubHandler::sector> sectors;
	for(int i = 0; i< aer.size();i++){
		sector tempSector;
		double a=aer.at(i).at(0);
		double e=aer.at(i).at(1);
		double r=aer.at(i).at(2);
		if(a<0){
			a += 2*M_PI;
		}
		if(e<0){
			e += M_PI;
		}
		e -= 5*M_PI/12;
		tempSector.a=int(floor(a*(360/(2*M_PI)/(360/_HREZ))));
		tempSector.e=int(floor(e*(360/(2*M_PI)/(30/_VREZ))));
		tempSector.r = r;
		sectors.push_back(tempSector);
	}
	ROS_INFO("Sectorizing Complete...");
	return sectors;
}
