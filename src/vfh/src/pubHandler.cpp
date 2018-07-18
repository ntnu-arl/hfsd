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
	_vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker",0);
	image_transport::ImageTransport it(n);
	_pubImage = it.advertise("open/image", 1);
	_pubContours = it.advertise("open/contours", 1);
	_queueCurrentSize = 0;
	_count = 0;
	_good = 0;
	_loops =0;
	if(n.getParam("queueSize", _queueSize)){
		ROS_INFO("QUEUE SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: QUEUE SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_queueSize = 7;
	}
	if(n.getParam("HREZ", _HREZ)){
		ROS_INFO("HORIZONTAL RESOLUTION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: HORIZONTAL RESOLUTION SET INCORRECTLY. SETTING TO DEFAULT");
		_HREZ = 90;
	}
	if(n.getParam("VREZ", _VREZ)){
		ROS_INFO("VERTICAL RESOLUTION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: VERTICAL RESOLUTION SET INCORRECTLY. SETTING TO DEFAULT");
		_VREZ = 10;
	}
	if(n.getParam("areaRestricter", _areaRestricter)){
		ROS_INFO("CONTOUR SIZE RESTRICTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: CONTOUR SIZE RESTRICTER INCORRECTLY. SETTING TO DEFAULT");
		_areaRestricter = 10.0;
	}
	if(n.getParam("skipFrames", _skipCounter)){
		ROS_INFO("SKIP FRAME COUNTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: SKIP FRAME COUNTER SET INCORRECTLY. SETTING TO DEFAULT");
		_skipCounter = 4;
	}
	if(n.getParam("voxelSize", _voxelSize)){
		ROS_INFO("VOXEL SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: VOXEL SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_voxelSize = 0.05f;
	}
	if(n.getParam("showTiming", _timing)){
		ROS_INFO("TIMING MESSAGE VISIBILITY SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: TIMING MESSAGE VISIBILITY SET INCORRECTLY. SETTING TO DEFAULT");
		_timing = true;
	}
	if(n.getParam("showDebugMessages", _debug)){
		ROS_INFO("DEBUG MESSAGE VISIBILITY SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: DEBUG MESSAGE VISIBILITY SET INCORRECTLY. SETTING TO DEFAULT");
		_debug = true;
	}
	//_queueSize = 7;
	//_HREZ = 90;
	//_VREZ = 10;
	//_areaRestricter = 10.0;
	//_HREZ = 180;
	//_VREZ =90;
}

//used to publish data from the publisher and check for errors
void pubHandler::publish(pcl::PointCloud<pcl::PointXYZ>::Ptr msg){
	try{
		_pub.publish(*msg);
		if(_debug)ROS_INFO("Success");
	}catch(...){
		ROS_INFO("An error has occurred");
	}
}

//callback function for the subscriber. performs all of the enqueuing and dequeuing;
void pubHandler::messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	if(_debug)ROS_INFO("Recieving Cloud...");
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>(*msg));
	_data = temp;
	_good = 1;
	//ROS_INFO("Check 2");

	//ROS_INFO("Check 3");
	//ROS_INFO("Check 4");
}

//this recieves the odometry for the program to create the sliding window
void pubHandler::messageReceivedPose(const nav_msgs::Odometry::ConstPtr& msg){
	if(_count >= _skipCounter-1 && _good == 1){
		std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
		_loops++;
		stringstream lo;
		lo<<"CURRENT LOOP: "<<_loops;
		if(_timing)ROS_INFO_STREAM(lo.str());
		if(_debug)ROS_INFO("Receiving Odometry...");
		std::chrono::high_resolution_clock::time_point t_start2 = std::chrono::high_resolution_clock::now();
		nav_msgs::Odometry odomData = *msg;
		if(_queueCurrentSize >= _queueSize){
			_window.pop_back();
			_odomWindow.pop_back();
			_queueCurrentSize--;
		}
		_window.push_front(_data);
		_odomWindow.push_front(odomData);
		_queueCurrentSize++;
		std::chrono::high_resolution_clock::time_point t_end2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> queueTime(t_end2-t_start2);
		_averageQueueing += queueTime;
		stringstream s1;
		s1<<"QUEUEING TIME: " << queueTime.count() << " milliseconds "<<"AVERAGE QUEUEING TIME: "<<_averageQueueing.count()/_loops<<" milliseconds";
		if(_timing)ROS_INFO_STREAM(s1.str());

		//ROS_INFO("Check 2");
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudScene(new pcl::PointCloud<pcl::PointXYZ>(_preprocessing(_window, _odomWindow)));
		_vfh3D(ptCloudScene);
		//ROS_INFO("Check 3");
		this->publish(ptCloudScene);
		_count = 0;
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
		_averageExecution += executionTime;
		stringstream s;
		s<<"EXECUTION TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE EXECUTION TIME: "<<_averageExecution.count()/_loops<<" milliseconds";
		if(_timing)ROS_INFO_STREAM(s.str());
		s.clear();

		}else{
			if(_debug)ROS_INFO("SKIPPING...");
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
	if(_debug)ROS_INFO("Preprocessing...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudScene(new pcl::PointCloud<pcl::PointXYZ>(*window[0]));
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudSceneFiltered(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Vector3d endV;

	Eigen::Quaterniond endQ;


	tf2::fromMsg(odomWindow[0].pose.pose.position, endV);
	tf2::fromMsg(odomWindow[0].pose.pose.orientation,endQ);

	Eigen::Matrix3d m;
	m(0,0) = 0;
	m(0,1) = 1;
	m(0,2) = 0;
	m(1,0) = 0;
	m(1,1) = 0;
	m(1,2) = 1;
	m(2,0) = 1;
	m(2,1) = 0;
	m(2,2) = 0;

	//endV = m*endV;
	/*m = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()) *
	    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitX());*/
	//endV = endV.transpose() * endQ.toRotationMatrix();
	//string s = "W: " + to_string(endQ.w()) + "X: "+ to_string(endQ.x()) + "Y: "+ to_string(endQ.y())+ "Z: "+ to_string(endQ.z());
	//ROS_INFO_STREAM(s);
	for(int i = 1; i<_queueCurrentSize;i++){
		Eigen::Vector3d startV;
		Eigen::Quaterniond startQ;
		tf2::fromMsg(odomWindow[i].pose.pose.position, startV);
		tf2::fromMsg(odomWindow[i].pose.pose.orientation,startQ);
		//Eigen::Quaterniond differenceQ = differenceOfQuat(startQ,endQ);
		//startV = m* startV;;
		Eigen::Vector3d differenceV = differenceOfVec(startV,endV);
		differenceV = differenceV.transpose() * endQ.toRotationMatrix();
		Eigen::Affine3d affine1 = Eigen::Affine3d::Identity();;
		//Eigen::Affine3d affine2 = Eigen::Affine3d::Identity();;
		affine1.translation() << differenceV[0],differenceV[1],differenceV[2];
		//affine1.rotate(differenceQ.toRotationMatrix());
		pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*window[i],*transformedCloud,affine1);
		//pcl::transformPointCloud(*transformedCloud,*transformedCloud,affine2);

		*ptCloudScene += *transformedCloud;
	}
	//string s1 = "Size: "+ to_string(ptCloudScene->points.size());
	//ROS_INFO_STREAM(s1);
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(ptCloudScene);
	sor.setLeafSize (_voxelSize, _voxelSize, _voxelSize);
	sor.filter(*ptCloudSceneFiltered);
	//*ptCloudSceneFiltered->points[1].data;
	if(_debug)ROS_INFO("Preprocessing Complete");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> preprocessingTime(t_end-t_start);
	stringstream s;
	s<<"PREPROCESSING TIME: " << preprocessingTime.count() << " milliseconds "<<"AVERAGE PREPROCESSING TIME: "<<_averagePreprocessing.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s.str());
	_averagePreprocessing += preprocessingTime;
	return *ptCloudSceneFiltered;
}

cv::Mat pubHandler::_radmatrix(std::vector<pubHandler::sector> points){
	if(_debug)ROS_INFO("Creating RadMatrix...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	cv::Mat radmat(2*_HREZ, _VREZ,CV_8UC1, cv::Scalar(0));
	//double tempMat[_HREZ][_VREZ] = {0.0};
	double** tempMat = new double* [_HREZ];
	for(int i = 0; i<_HREZ;i++){
		tempMat[i] = new double[_VREZ];
	}
	for(int i = 0; i<_HREZ; i++){
		for( int j = 0; j<_VREZ; j++){
			tempMat[i][j]= 0.0;
		}
	}
	/*
	double** wrappedMat = new double* [2*_HREZ];
	for(int i = 0; i<2*_HREZ;i++){
		wrappedMat[i] = new double[_VREZ];
	}
	for(int i = 0; i<_HREZ; i++){
		for( int j = 0; j<_VREZ; j++){
			wrappedMat[i][j]= 0.0;
		}
	}
	*/

	for(int row = 0; row<points.size();row++){
		//std_msgs::String s;
		string s = "Azimuth: " + to_string(points.at(row).a) + "Elevation: "+ to_string(points.at(row).e) + "Radius: "+ to_string(points.at(row).r);
		//ROS_INFO_STREAM(s);//<<points[row];
		if(points.at(row).e >= 0 && points.at(row).e < _VREZ){
			if((tempMat[points.at(row).a][points.at(row).e] > points.at(row).r || tempMat[points.at(row).a][points.at(row).e]<=0.01)){
				tempMat[points.at(row).a][points.at(row).e] = points.at(row).r;
			}
		}
	}
	//ROS_INFO("Check 1");
	double max = 0;
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			if(val>max){
				max=val;
			}
		}
	}

	//ROS_INFO("Check 2");
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			int val = tempMat[row][col];
			tempMat[row][col] = 255.0*(val/max);
		}
	}
	//double wrappedMat[2*_HREZ][_VREZ]= {0.0};
	for(int row = _HREZ/2; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			radmat.at<uchar>(row-_HREZ/2,col) = (uchar)(tempMat[row][col]);
			//int val = tempMat[row][col];
			//wrappedMat[row - _HREZ/2][col] = val;
		}
	}
	for(int row = 0; row<_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			radmat.at<uchar>(row+_HREZ/2,col) = (uchar)(tempMat[row][col]);
			//int val = tempMat[row][col];
			//wrappedMat[row + _HREZ/2][col] = val;
		}
	}
	for(int row = 0; row<_HREZ/2; row++){
		for(int col = 0; col < _VREZ; col++){
			radmat.at<uchar>(row+ 3*_HREZ/2,col) = (uchar)(tempMat[row][col]);
			//int val = tempMat[row][col];
			//wrappedMat[row + 3*_HREZ/2][col] = val;
		}
	}
	//ROS_INFO("Check 3");
	/*
	for(int row = 0; row<2*_HREZ; row++){
		for(int col = 0; col < _VREZ; col++){
			radmat.at<uchar>(row,col) = (uchar)(wrappedMat[row][col]);
		}
	}*/
	//delete[] wrappedMat;
	delete[] tempMat;


	//TODO: Write the sectorization Methods
	//TODO: Write Intensity value matrix in mat form
	if(_debug)ROS_INFO("RadMatrix Complete");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
	_averageRad += executionTime;
	stringstream s;
	s<<"HISTOGRAM TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE HISTOGRAM TIME: "<<_averageRad.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s.str());
	return radmat;
}
//this is the primary algorithm used to determine the best vectors for travel
std::map<std::string,std::vector<pubHandler::trajectory> > pubHandler::_vfh3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	if(_debug)ROS_INFO("Beginning VFH...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	cv::RNG _rng(12345);
	std::map<std::string,std::vector<trajectory> > trajVec;
	std::vector<std::vector<double> > points = _extractPointsFromCloud(cloud);
	//points = _convertToSpherical(points);
	std::vector<pubHandler::sector> sectors = _sectorize(points);
	cv::Mat grayMat(_radmatrix(sectors));

	cv::GaussianBlur(grayMat,grayMat,cv::Size_<int>(3,5),3,5);
	//ROS_INFO("Check 1");
	cv::Mat binaryImage(2*_HREZ, _VREZ,CV_8UC1, cv::Scalar(0));
	cv::threshold(grayMat, binaryImage, 0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
	//cv::Mat cont(binaryImage);
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cv::findContours( binaryImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > contoursfiltered;
	for(int i = 0; i<contours.size(); i++){
		if(cv::contourArea(contours[i])>_areaRestricter){
			contoursfiltered.push_back(contours[i]);
		}
	}
	std::chrono::high_resolution_clock::time_point t_start2 = std::chrono::high_resolution_clock::now();
	std::vector<Scalar> colors;
	double maxMag = 0;
	vector<pubHandler::trajectory> trajectories;
	Mat drawing = Mat::zeros( binaryImage.size(), CV_8UC3 );
	for( size_t i = 0; i< contoursfiltered.size(); i++ ){
		Scalar color = Scalar( _rng.uniform(0, 255), _rng.uniform(0,255), _rng.uniform(0,255) );
		drawContours( drawing, contoursfiltered, (int)i, color, FILLED, 8, hierarchy, 0, Point() );
		vector<Point> cont = contoursfiltered[i];
		Moments m = moments(cont);
		pubHandler::trajectory t;
		t.sectorX = m.m10/m.m00;
		t.sectorY = m.m01/m.m00;
		t.magnitude = contourArea(cont);
		if(t.magnitude>maxMag){
			maxMag = t.magnitude;
		}
		if(t.sectorY>=_HREZ/2 && t.sectorY<3*_HREZ/2){
			circle(drawing,Point_<int>(t.sectorX,t.sectorY),1, Scalar(255,0,0));
			colors.push_back(color);
			trajectories.push_back(t);
		}
	}
	std::chrono::high_resolution_clock::time_point t_end2 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> visTime1(t_end2-t_start2);
	//ROS_INFO("Check 2");

	/*for(int i = 0; i<contoursfiltered.size();i++){
		vector<Point> cont = contoursfiltered[i];
		Moments m = moments(cont);
		pubHandler::trajectory t;
		t.sectorX = m.m10/m.m00;
		t.sectorY = m.m01/m.m00;
		t.magnitude = contourArea(cont);
		if(t.magnitude>maxMag){
			maxMag = t.magnitude;
		}
		if(t.sectorY>=_HREZ/2 && t.sectorY<3*_HREZ/2){
			circle(drawing,Point_<int>(t.sectorX,t.sectorY),1, Scalar(255,0,0));
			trajectories.push_back(t);
		}
	}*/
	//ROS_INFO("Check 3");
	for(int i =0; i<trajectories.size();i++){
		trajectories[i].sectorX = (trajectories[i].sectorX * (30/_VREZ)*(2*M_PI/360)) + 5*M_PI/12;
		trajectories[i].sectorY = ((trajectories[i].sectorY -_HREZ/2) * (360/_HREZ)*(2*M_PI/360));
		trajectories[i].magnitude *=2/maxMag;
		std::vector<double> aer = {trajectories[i].sectorY,trajectories[i].sectorX,trajectories[i].magnitude};
		trajectories[i].xyz = _convertToCartesian(aer);
	}
	//ROS_INFO("Check 4");
	std::chrono::high_resolution_clock::time_point t_start3 = std::chrono::high_resolution_clock::now();
	visualization_msgs::MarkerArray deleter;
	deleter.markers.resize(1);
	deleter.markers[0].header.frame_id = "velodyne";
	deleter.markers[0].header.stamp = ros::Time();
	deleter.markers[0].ns = "my_namespace";
	deleter.markers[0].id = 0;
	deleter.markers[0].type = visualization_msgs::Marker::ARROW;
	deleter.markers[0].action = visualization_msgs::Marker::DELETEALL;
	_vis_pub.publish(deleter);
	visualization_msgs::MarkerArray markArray;
	markArray.markers.resize(trajectories.size());

	for(int i = 0; i <trajectories.size();i++){

		markArray.markers[i].header.frame_id = "velodyne";
		markArray.markers[i].header.stamp = ros::Time();
		markArray.markers[i].ns = "my_namespace";
		markArray.markers[i].id = i;
		markArray.markers[i].type = visualization_msgs::Marker::ARROW;
		markArray.markers[i].action = visualization_msgs::Marker::ADD;
		geometry_msgs::Point start;
		start.x = 0;
		start.y = 0;
		start.z = 0;
		geometry_msgs::Point end;
		end.x =trajectories[i].xyz[0];
		end.y =trajectories[i].xyz[1];
		end.z =trajectories[i].xyz[2];

		markArray.markers[i].points = {start, end};
		markArray.markers[i].scale.x = 0.1;
		markArray.markers[i].scale.y = 0.15;
		markArray.markers[i].scale.z = 0.1;
		markArray.markers[i].color.a = 1.0; // Don't forget to set the alpha!
		markArray.markers[i].color.r = colors[i].val[0]/255.0;
		markArray.markers[i].color.g = colors[i].val[1]/255.0;
		markArray.markers[i].color.b = colors[i].val[2]/255.0;
		//only if using a MESH_RESOURCE marker type:
		markArray.markers[i].mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

	}
	//ROS_INFO("Check 5");
	//cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE);
	//ROS_INFO("Check 2");

	sensor_msgs::ImagePtr msgC = cv_bridge::CvImage(std_msgs::Header(), "rgb8", drawing).toImageMsg();
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayMat).toImageMsg();
	_vis_pub.publish(markArray);
	_pubImage.publish(msg);
	_pubContours.publish(msgC);
	std::chrono::high_resolution_clock::time_point t_end3 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> visTime2(t_end3-t_start3);
	visTime2 += visTime1;
	//cv::imshow("Display Window",grayMat.);
	//TODO process radmatrix into binary matrix using opencv's gaussian adaptive threshold
	//TODO Find Centroids of binary matrix regions.
	//TODO Classify the trajectory vectors in the map
	if(_debug)ROS_INFO("VFH Complete");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> algorithmTime(t_end-t_start);
	_averageAlgorithm += algorithmTime;
	_averageAlgorithm -= visTime2;
	_averageVis += visTime2;
	stringstream s3;
	s3<<"VISUALIZATION TIME: " << visTime2.count() << " milliseconds "<<"AVERAGE VISUALIZATION TIME: "<<_averageVis.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s3.str());
	stringstream s2;
	s2<<"ALGORITHM TIME: " << algorithmTime.count() << " milliseconds "<<"AVERAGE ALGORITHM TIME: "<<_averageAlgorithm.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s2.str());
	return trajVec;

}
std::vector<std::vector<double> > pubHandler::_convertToSpherical(std::vector<std::vector<double> > xyz){
	//TODO::convert vector of n rows and three columns from cartesian to spherical
	if(_debug)ROS_INFO("Converting to Spherical...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	for(int i = 0; i< xyz.size();i++){
		double x=xyz.at(i).at(0);
		double y=xyz.at(i).at(1);
		double z=xyz.at(i).at(2);
		xyz.at(i).at(2)=sqrt(pow(x,2) + pow(y,2) + pow(z,2));
		xyz.at(i).at(0)=atan2(y,x);
		xyz.at(i).at(1)=acos(z/xyz.at(i).at(2));

	}
	if(_debug)ROS_INFO("Conversion Complete");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
	_averageConversion += executionTime;
	stringstream s;
	s<<"SPHERICAL CONVERSION TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE CONVERSION TIME: "<<_averageConversion.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s.str());
	return xyz;
}

std::vector<double> pubHandler::_convertToCartesian(std::vector<double> aer){
		//TODO::convert vector of n rows and three columns from cartesian to spherical
	//for(int i = 0; i< aer.size();i++){
		double a=aer.at(0);
		double e=aer.at(1);
		double r=aer.at(2);
		aer.at(0)=r*sin(e)*cos(a);
		aer.at(1)=r*sin(e)*sin(a);
		aer.at(2)=r*cos(e);
	//}
	return aer;

}
std::vector<std::vector<double> > pubHandler::_extractPointsFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	//TODO: get vector nx3 vector of XYZ coordinates from point cloud
	if(_debug)ROS_INFO("Extracting Points...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::vector<std::vector<double> > points;
	BOOST_FOREACH(const pcl::PointXYZ& pt, cloud->points){
	//for(int i = 0; i<cloud->size();i++){
		std::vector<double> temp;
		double r = sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
		temp.push_back(atan2(pt.y,pt.x));
		temp.push_back(acos(pt.z/r));
		temp.push_back(r);

		points.push_back(temp);
	}
	if(_debug)ROS_INFO("Points Extracted");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
	_averageExtraction += executionTime;
	stringstream s;
	s<<"EXTRACTION TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE EXTRACTION TIME: "<<_averageExtraction.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s.str());
	return points;
}

std::vector<pubHandler::sector> pubHandler::_sectorize(std::vector<std::vector<double> > aer){
	if(_debug)ROS_INFO("Sectorizing...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
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
		tempSector.a=int(floor(a*(360/(2*M_PI)/(360/(_HREZ-0)))));
		tempSector.e=int(floor(e*(360/(2*M_PI)/(30/(_VREZ-0)))));
		tempSector.r = r;
		sectors.push_back(tempSector);
	}
	if(_debug)ROS_INFO("Sectorizing Complete...");
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
	_averageSectorization += executionTime;
	stringstream s;
	s<<"SECTORING TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE SECTORING TIME: "<<_averageSectorization.count()/_loops<<" milliseconds";
	if(_timing)ROS_INFO_STREAM(s.str());
	return sectors;
}
