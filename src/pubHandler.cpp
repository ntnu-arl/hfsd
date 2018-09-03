//============================================================================
// Name        : pubHandler.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description : The bulk of the algorithm each function is commented.
//============================================================================

#include "hfsd/pubHandler.h"

pubHandler::pubHandler(ros::NodeHandle & n, const std::string & s, int bufSize){
	_windowCurrentSize = 0;
	_count = 0;
	_alignmentSwitch = 0;
	_loops =0;
	_sequence = 0;
	_colors.push_back(Scalar(255,30,30));
	_colors.push_back(Scalar(0,168,8));
	_colors.push_back(Scalar(0,208,255));
	_colors.push_back(Scalar(255,93,0));
	_colors.push_back(Scalar(0,0,255));
	_colors.push_back(Scalar(131,0,255));
	n.param("markerLifetime", _markerLifetime, 0.0);
	n.param("arrowShaftDiameter", _arrowShaftDiameter, 0.1);
	n.param("arrowHeadDiameter", _arrowHeadDiameter, 0.15);
	n.param("arrowHeadLength", _arrowHeadLength, 0.1);
	n.param("markerMag", _relativeMagnitude, 2.0);
	n.param("markerSkip", _markerSkip, 0);
        n.param("hfsd_map_frame", _hfsdMapFrame, std::string("hfsd_map"));
	//_tfListener=new tf2_ros::TransformListener(_tfBuffer);
	if(n.getParam("HREZ", _AzRez)){
		ROS_INFO("HORIZONTAL RESOLUTION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: HORIZONTAL RESOLUTION SET INCORRECTLY. SETTING TO DEFAULT");
		_AzRez = 90;
	}
	if(n.getParam("VREZ", _ElRez)){
		ROS_INFO("VERTICAL RESOLUTION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: VERTICAL RESOLUTION SET INCORRECTLY. SETTING TO DEFAULT");
		_ElRez = 10;
	}
	if(n.getParam("queueSize", _queueSize)){
		ROS_INFO("QUEUE SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: QUEUE SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_queueSize = 10;
	}
	if(n.getParam("windowSize", _windowSize)){
		ROS_INFO("WINDOW SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: WINDOW SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_windowSize = 7;
	}
	if(n.getParam("skipFrames", _skipCounter)){
		ROS_INFO("SKIP FRAME COUNTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: SKIP FRAME COUNTER SET INCORRECTLY. SETTING TO DEFAULT");
		_skipCounter = 4;
	}
	if(n.getParam("areaRestricter", _areaRestricter)){
		ROS_INFO("CONTOUR SIZE RESTRICTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: CONTOUR SIZE RESTRICTER INCORRECTLY. SETTING TO DEFAULT");
		_areaRestricter = 10.0;
	}
	if(n.getParam("heightSplitter", _splitter)){
		ROS_INFO("CONTOUR SPLITTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: CONTOUR SPLITTER SET INCORRECTLY. SETTING TO DEFAULT");
		_splitter = 10.0;
	}
	if(n.getParam("minDistRejFilterVal", _rejection)){
		ROS_INFO("REJECTION FILTER SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: REJECTION FILTER SET INCORRECTLY. SETTING TO DEFAULT");
		_rejection =0.0;
	}
	if(n.getParam("voxelSize", _voxelSize)){
		ROS_INFO("VOXEL SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: VOXEL SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_voxelSize = 0.05;
	}
	if(n.getParam("uniformGrid", _uniformGrid)){
		ROS_INFO("UNIFORM GRID SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: UNIFORM GRID SET INCORRECTLY. SETTING TO DEFAULT");
		_uniformGrid = true;
	}
	if(n.getParam("voxelUniformSize", _voxelUniformSize)){
		ROS_INFO("VOXEL UNIFORMITY SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: VOXEL UNIFORMITY SET INCORRECTLY. SETTING TO DEFAULT");
		_voxelUniformSize = 0.05;
	}
	if(n.getParam("box", _box)){
		ROS_INFO("BOX SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BOX SET INCORRECTLY. SETTING TO DEFAULT");
		_box = true;
	}
	if(n.getParam("boxSizeX", _boxSizeX)){
		ROS_INFO("BOX SIZE X SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BOX SIZE X SET INCORRECTLY. SETTING TO DEFAULT");
		_boxSizeX = 0;
	}
	if(n.getParam("boxSizeY", _boxSizeY)){
		ROS_INFO("BOX SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BOX SIZE Y SET INCORRECTLY. SETTING TO DEFAULT");
		_boxSizeY = 0;
	}
	if(n.getParam("blur", _blur)){
		ROS_INFO("BLUR SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BLUR SET INCORRECTLY. SETTING TO DEFAULT");
		_blur = true;
	}
	if(n.getParam("blurSizeX", _blurSizeX)){
		ROS_INFO("BLUR SIZE X SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BLUR SIZE X SET INCORRECTLY. SETTING TO DEFAULT");
		_blurSizeX = 0;
	}
	if(n.getParam("blurSizeY", _blurSizeY)){
		ROS_INFO("BLUR SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: BLUR SIZE Y SET INCORRECTLY. SETTING TO DEFAULT");
		_blurSizeY = 0;
	}
	if(n.getParam("blurSigmaX", _blurSigmaX)){
		ROS_INFO("SIGMA X SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: SIGMA X SET INCORRECTLY. SETTING TO DEFAULT");
		_blurSigmaX = 3;
	}
	if(n.getParam("blurSigmaY", _blurSigmaY)){
		ROS_INFO("SIGMA Y SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: SIGMA Y SET INCORRECTLY. SETTING TO DEFAULT");
		_blurSigmaY = 1.0;
	}
	if(n.getParam("median", _median)){
		ROS_INFO("MEDIAN SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: MEDIAN SET INCORRECTLY. SETTING TO DEFAULT");
		_median = true;
	}
	if(n.getParam("medianSize", _medianSize)){
		ROS_INFO("MEDIAN SIZE SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: MEDIAN SIZE SET INCORRECTLY. SETTING TO DEFAULT");
		_medianSize = 3;
	}
	if(n.getParam("intensityOffset", _iOffset)){
		ROS_INFO("INTENSITY OFFSET SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: INTENSITY OFFSET SET INCORRECTLY. SETTING TO DEFAULT");
		_iOffset = -745;
	}
	if(n.getParam("dilate", _dilate)){
		ROS_INFO("DILATION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: DILATION SET INCORRECTLY. SETTING TO DEFAULT");
		_dilate = true;
	}
	if(n.getParam("dilationIterations", _dIterations)){
		ROS_INFO("DILATION ITERATIONS SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: DILATION ITERATIONS SET INCORRECTLY. SETTING TO DEFAULT");
		_dIterations = 2;
	}
	if(n.getParam("erode", _erode)){
		ROS_INFO("EROSION SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: EROSION SET INCORRECTLY. SETTING TO DEFAULT");
		_erode = true;
	}
	if(n.getParam("erosionIterations", _eIterations)){
		ROS_INFO("EROSION ITERATIONS SET CORRECTLY");
	}else{
		ROS_INFO("ERROR: EROSION ITERATIONS SET INCORRECTLY. SETTING TO DEFAULT");
		_eIterations = 2;
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
	
	// Advertising
	_pubPoints = n.advertise<sensor_msgs::PointCloud2>(s, bufSize);
	_pubOdometry = n.advertise<nav_msgs::Odometry>("OdomOut", bufSize);
	_pubVect = n.advertise<hfsd::Vector3Array>("direction_vectors", bufSize);
	_vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);
	image_transport::ImageTransport it(n);
	_pubImage = it.advertise("open/image", 1);
	_pubContours = it.advertise("open/contours", 1);
	_pubVote = it.advertise("open/vote", 1);
	
	// Subscriptions
	subPoints = n.subscribe("points_input", _queueSize, &pubHandler::messageReceivedCloud, this);
	subOdometry = n.subscribe("odometry_input", _queueSize, &pubHandler::messageReceivedPose, this);
}

//callback function for the subscriber. performs all of the enqueuing and dequeuing;
void pubHandler::messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg){
	if(_debug)ROS_INFO("Receiving Cloud...");
	_data = boost::make_shared<std::remove_const<std::remove_reference<decltype(*msg)>::type>::type >(*msg);
	_alignmentSwitch = 1;
}

//this receives the odometry for the program to create the sliding window
void pubHandler::messageReceivedPose(const nav_msgs::Odometry::ConstPtr & msg){
	if(_count >= _skipCounter && _alignmentSwitch == 1){
		if(_loops == 0){
			tf2::fromMsg(msg->pose.pose.position, _initV);
			tf2::fromMsg(msg->pose.pose.orientation, _initQ);
		}
		_loops++;
		if(_timing)ROS_INFO_STREAM("CURRENT LOOP: "<<_loops);
		if(_debug)ROS_INFO("Receiving Odometry...");
		std::chrono::high_resolution_clock::time_point t_start2 = std::chrono::high_resolution_clock::now();


		//geometry_msgs::TransformStamped transformStamped = _tfBuffer.lookupTransform("/imu","/stereo_link",ros::Time(0));
		//Eigen::Affine3d affine = tf2::transformToEigen(transformStamped);
		nav_msgs::Odometry odomData = *msg;
		//odomData.header.stamp = ros::Time::now();
		if(_pubOdometry.getNumSubscribers()>0) _pubOdometry.publish(odomData);
		if(_windowCurrentSize >= _windowSize){
			_window.pop_back();
			_odomWindow.pop_back();
			_windowCurrentSize--;
		}
		auto ptCloudSceneFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(_data);
		sor.setLeafSize (_voxelSize, _voxelSize, _voxelSize);
		sor.filter(*ptCloudSceneFiltered);
		//pcl::transformPointCloud(*ptCloudSceneFiltered,*ptCloudSceneFiltered,affine);
		_window.push_front(ptCloudSceneFiltered);
		_odomWindow.push_front(odomData);
		_windowCurrentSize++;
		if(_timing){
			std::chrono::high_resolution_clock::time_point t_end2 = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double, std::milli> queueTime(t_end2-t_start2);
			_averageQueueing += queueTime;
			ROS_INFO_STREAM("QUEUEING TIME: " << queueTime.count() << " milliseconds "<<"AVERAGE QUEUEING TIME: "<<_averageQueueing.count()/_loops<<" milliseconds");
		}
		auto ptCloudScene = _preprocessing(_window, _odomWindow);

		  geometry_msgs::TransformStamped transformStamped;

		  transformStamped.header.frame_id = msg->header.frame_id; //"world";
		  transformStamped.child_frame_id = _hfsdMapFrame;
		  transformStamped.transform.translation.x = _CurrentV.x();
		  transformStamped.transform.translation.y = _CurrentV.y();
		  transformStamped.transform.translation.z = _CurrentV.z();
		  transformStamped.transform.rotation.x = _CurrentQ.x();
		  transformStamped.transform.rotation.y = _CurrentQ.y();
		  transformStamped.transform.rotation.z = _CurrentQ.z();
		  transformStamped.transform.rotation.w = _CurrentQ.w();
		  transformStamped.header.stamp = odomData.header.stamp; //ros::Time::now();
		  tfb.sendTransform(transformStamped);

		_freeTrajectories(ptCloudScene);

		if(_pubPoints.getNumSubscribers()>0){
		  ptCloudScene->header.frame_id = _hfsdMapFrame;
		  //pcl_conversions::toPCL(ros::Time::now(), ptCloudScene->header.stamp);
		  _pubPoints.publish(*ptCloudScene);
		}
		_count = 0;
		_alignmentSwitch=0;
		  if(_timing){
		    std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		    std::chrono::duration<double, std::milli> executionTime(t_end-t_start2);
		    _averageExecution += executionTime;
		    ROS_INFO_STREAM("EXECUTION TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE EXECUTION TIME: "<<_averageExecution.count()/_loops<<" milliseconds");
		  }
	}
	else{
		if(_debug)ROS_INFO("SKIPPING...");
		_count++;
	}
}

//takes in two quaternions and gives back the quaternion that rotates from the starting quaternion to the next quaternion
Eigen::Quaterniond pubHandler::differenceOfQuat(const Eigen::Quaterniond & start, const Eigen::Quaterniond & end){
	Eigen::Quaterniond difference = end.inverse() * start;
	return difference;
}
Eigen::Vector3d pubHandler::differenceOfVec(const Eigen::Vector3d & start, const Eigen::Vector3d & end){
	Eigen::Vector3d difference = start-end;
	return difference;
}
//this is the pre-processing step that transforms and filters the point cloud window
pcl::PointCloud<pcl::PointXYZ>::Ptr pubHandler::_preprocessing(std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr > window, const std::deque<nav_msgs::Odometry> & odomWindow){
	if(_debug)ROS_INFO("Preprocessing...");
	std::chrono::high_resolution_clock::time_point t_start1 = std::chrono::high_resolution_clock::now();
	auto ptCloudScene = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(*window[0]);
	Eigen::Vector3d endV;
	Eigen::Quaterniond endQ;
	tf2::fromMsg(odomWindow[0].pose.pose.position, endV);
	tf2::fromMsg(odomWindow[0].pose.pose.orientation, endQ);
	_CurrentQ = differenceOfQuat(endQ, _initQ);
	_CurrentV = differenceOfVec(endV, _initV);
	_CurrentV = _CurrentV.transpose() * _initQ.toRotationMatrix();
	std::chrono::high_resolution_clock::time_point t_start2 = std::chrono::high_resolution_clock::now();
	if(_debug)ROS_INFO_STREAM("CURRENT WINDOW SIZE: "<<_windowCurrentSize);
	for(int i = 1; i<_windowCurrentSize;i++){
		Eigen::Vector3d startV;
		Eigen::Quaterniond startQ;
		tf2::fromMsg(odomWindow[i].pose.pose.position, startV);
		tf2::fromMsg(odomWindow[i].pose.pose.orientation, startQ);
		Eigen::Quaterniond differenceQ = differenceOfQuat(startQ, endQ);
		Eigen::Vector3d differenceV = differenceOfVec(startV, endV);
		differenceV = differenceV.transpose() * endQ.toRotationMatrix();
		Eigen::Affine3d affine1 = Eigen::Affine3d::Identity();;
		affine1.translation() << differenceV[0],differenceV[1],differenceV[2];
		affine1.rotate(differenceQ.toRotationMatrix());
		pcl::PointCloud<pcl::PointXYZ> transformedCloud;
		pcl::transformPointCloud(*window[i],transformedCloud,affine1);
		*ptCloudScene += transformedCloud;
	}
	std::chrono::high_resolution_clock::time_point t_end2 = std::chrono::high_resolution_clock::now();
	auto ptCloudSceneFiltered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	std::chrono::high_resolution_clock::time_point t_start3 = std::chrono::high_resolution_clock::now();

	if(_uniformGrid){
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(ptCloudScene);
		sor.setLeafSize (_voxelUniformSize, _voxelUniformSize, _voxelUniformSize);
		sor.filter(*ptCloudSceneFiltered);
	}
	std::chrono::high_resolution_clock::time_point t_end3 = std::chrono::high_resolution_clock::now();
	if(_debug)ROS_INFO("Preprocessing Complete");
	if(_timing){
		std::chrono::high_resolution_clock::time_point t_end1 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> preprocessingTime(t_end1-t_start1);
		std::chrono::duration<double, std::milli> transformTime(t_end2-t_start2);
		std::chrono::duration<double, std::milli> voxelTime(t_end3-t_start3);
		_averagePreprocessing += preprocessingTime;
		_averageTransform += transformTime;
		_averageVoxel += voxelTime;
		ROS_INFO_STREAM("TRANSFORM TIME: " << transformTime.count() << " milliseconds "<<"AVERAGE TRANSFORM TIME: "<<_averageTransform.count()/_loops<<" milliseconds");
		ROS_INFO_STREAM("VOXEL GRID TIME: " << voxelTime.count() << " milliseconds "<<"AVERAGE VOXEL GRID TIME: "<<_averageVoxel.count()/_loops<<" milliseconds");
		ROS_INFO_STREAM("PREPROCESSING TIME: " << preprocessingTime.count() << " milliseconds "<<"AVERAGE PREPROCESSING TIME: "<<_averagePreprocessing.count()/_loops<<" milliseconds");
	}
	if(_uniformGrid)return ptCloudSceneFiltered;
	return ptCloudScene;
}

cv::Mat pubHandler::_radmatrix(const std::vector<pubHandler::sector> & points){
	if(_debug)ROS_INFO("Creating RadMatrix...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	cv::Mat radmat(2*_AzRez, _ElRez,CV_8UC1, cv::Scalar(0));
	std::vector<std::vector<double> > tempMat(_AzRez, std::vector<double>(_ElRez, 0.0));
	for(int i = 0; i<_AzRez; i++){
		for( int j = 0; j<_ElRez; j++){
			tempMat[i][j]= -1.0;
		}
	}
	for(int row = 0; row< points.size();row++){
		int a = points.at(row).a;
		int e = points.at(row).e;
		double r = points.at(row).r;
		if(e>= 0 && e < _ElRez){
			if(r<0){
				ROS_INFO("NEGATIVE RADIUS!");
			}
			if(r <tempMat[a][e]|| tempMat[a][e]<0.00){
				tempMat[a][e] = r;
			}
		}
	}
	double max = 0;
	for(int row = 0; row<_AzRez; row++){
		for(int col = 0; col < _ElRez; col++){
			int val = tempMat[row][col];
			if(val<_rejection){
				tempMat[row][col]=0;
			}else{

				if(val>max){
					max=val;
				}
			}
		}
	}
	for(int row = _AzRez/2; row<_AzRez; row++){
		for(int col = 0; col < _ElRez; col++){
			radmat.at<uchar>(row-_AzRez/2,col) = (uchar)(std::min((255-_iOffset)*pow((tempMat[row][col]/max),2),255.0));
		}
	}
	for(int row = 0; row<_AzRez; row++){
		for(int col = 0; col < _ElRez; col++){
			radmat.at<uchar>(row+_AzRez/2,col) = (uchar)(std::min((255-_iOffset)*pow((tempMat[row][col]/max),2),255.0));
		}
	}
	for(int row = 0; row<_AzRez/2; row++){
		for(int col = 0; col < _ElRez; col++){
			radmat.at<uchar>(row+ 3*_AzRez/2,col) = (uchar)(std::min((255-_iOffset)*pow((tempMat[row][col]/max),2),255.0));
		}
	}
	if(_debug)ROS_INFO("RadMatrix Complete");
	if(_timing){
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
		_averageRad += executionTime;
		ROS_INFO_STREAM("HISTOGRAM TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE HISTOGRAM TIME: "<<_averageRad.count()/_loops<<" milliseconds");
	}
	return radmat;
}
//this is the primary algorithm used to determine the best vectors for travel
std::map<std::string,std::vector<pubHandler::trajectory> > pubHandler::_freeTrajectories(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud){
	if(_debug)ROS_INFO("Beginning VFH...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	cv::RNG _rng(12345);
	std::map<std::string,std::vector<trajectory> > trajVec;
	std::vector<pubHandler::sector> sectors = _extractPointsFromCloud(cloud);
	cv::Mat grayMat(_radmatrix(sectors));
	cv::Mat binaryImage(2*_AzRez, _ElRez,CV_8UC1, cv::Scalar(0));
	if(_box)cv::blur(grayMat.clone(),grayMat,cv::Size(_boxSizeX,_boxSizeY));
	if(_blur)cv::GaussianBlur(grayMat.clone(),grayMat,cv::Size(_blurSizeX,_blurSizeY),_blurSigmaX,_blurSigmaY);
	if(_median)cv::medianBlur(grayMat.clone(),grayMat,_medianSize);
	if(_erode)cv::erode(grayMat.clone(),grayMat,Mat(),Point(-1,-1),_eIterations);
	if(_dilate)cv::dilate(grayMat.clone(),grayMat,Mat(),Point(-1,-1),_dIterations);
	cv::threshold(grayMat.clone(), binaryImage,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);

	vector<vector<Point> > contours;
	cv::findContours( binaryImage.clone(), contours, RETR_LIST, CHAIN_APPROX_NONE, Point(0, 0) );
	vector<vector<Point> > contoursfiltered;
	vector<Rect> rois;
	for(int i = 0; i<contours.size(); i++){
		vector<Point> cont = contours[i];
		Rect bounding = boundingRect(cont);
		int height = bounding.height;
		int times = height / _splitter;
		if(times != 0){
			int adder = height % times;
			int heights = height/times;
			int currentY = bounding.y;
			for(int i = 0; i < times - 1; i++){
				rois.push_back(Rect(bounding.x, currentY , bounding.width, heights));
				currentY += heights;
			}
			rois.push_back(Rect(bounding.x, currentY , bounding.width, heights+adder));
		}else{
			rois.push_back(bounding);
		}

	}
	vector<vector<Point>> roiCont;
	for(int i = 0; i<rois.size();i++){
		vector<vector<Point>> tempCont;
		Mat tempMat(binaryImage.clone(),rois[i]);
		findContours(tempMat.clone(),tempCont,RETR_LIST, CHAIN_APPROX_NONE, Point(rois[i].x, rois[i].y));
		roiCont.insert(roiCont.end(),tempCont.begin(),tempCont.end());
	}
	for(int i = 0; i<roiCont.size(); i++){
		if(cv::contourArea(roiCont[i])>_areaRestricter){
			contoursfiltered.push_back(roiCont[i]);
		}
	}
	std::chrono::high_resolution_clock::time_point t_start2 = std::chrono::high_resolution_clock::now();
	std::vector<Scalar> colors;
	double maxMag = 0;
	vector<pubHandler::trajectory> trajectories;
	Mat drawing = Mat::zeros( binaryImage.size(), CV_8UC3 );
	for( size_t i = 0; i< contoursfiltered.size(); i++ ){
		vector<Point> cont = contoursfiltered[i];
		Moments m = moments(cont);
		pubHandler::trajectory t;
		t.sectorX = m.m10/m.m00;
		t.sectorY = m.m01/m.m00;
		t.magnitude = contourArea(cont);
		if(t.magnitude>maxMag){
			maxMag = t.magnitude;
		}
		if(t.sectorY>=_AzRez/2 && t.sectorY<3*_AzRez/2){
				Scalar color;
				if(colors.size()<_colors.size()){
					color = _colors[colors.size()];
				}else{
					color = Scalar( _rng.uniform(0, 255), _rng.uniform(0,255), _rng.uniform(0,255) );
				}
				drawContours( drawing, contoursfiltered, (int)i, color, FILLED);
				circle(drawing,Point_<int>(t.sectorX,t.sectorY),1, Scalar(255 - color[0],255 - color[1],255 - color[2]));
				colors.push_back(color);
				trajectories.push_back(t);
		}
	}

		std::chrono::high_resolution_clock::time_point t_end2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> visTime1(t_end2-t_start2);

		std::sort(trajectories.begin(),trajectories.end(), pubHandler::sortingObj);
	for(int i =0; i<trajectories.size();i++){
		trajectories[i].sectorX = (trajectories[i].sectorX * (180/_ElRez)*(2*M_PI/360));
		trajectories[i].sectorY = ((trajectories[i].sectorY -_AzRez/2) * (360/_AzRez)*(2*M_PI/360));
		trajectories[i].magnitude *=_relativeMagnitude/maxMag;
		trajectories[i]._convertToCartesian();
	}
	std::chrono::high_resolution_clock::time_point t_start3 = std::chrono::high_resolution_clock::now();
	visualization_msgs::MarkerArray markArray;
	markArray.markers.resize(trajectories.size());
	hfsd::Vector3Array outVect;
	outVect.vectors.resize(trajectories.size());
	ros::Time stamp = ros::Time::now();
        ++_sequence;
        outVect.header.seq = _sequence;
	outVect.header.stamp = stamp;
	outVect.header.frame_id = _hfsdMapFrame;
	
	for(int i = 0 ; i <trajectories.size();i++){
		markArray.markers[i].header = outVect.header;
		markArray.markers[i].ns = "freespace_vectors_" + to_string(_loops);
		markArray.markers[i].id = i;
		markArray.markers[i].type = visualization_msgs::Marker::ARROW;
		markArray.markers[i].action = visualization_msgs::Marker::ADD;
		markArray.markers[i].lifetime = ros::Duration(_markerLifetime);
	
		geometry_msgs::Point start;
		start.x = 0;
		start.y = 0;
		start.z = 0;
		geometry_msgs::Point end;
		geometry_msgs::Vector3 vect;
		vect.x =trajectories[i].xyz[0];
		vect.y =trajectories[i].xyz[1];
		vect.z =trajectories[i].xyz[2];
		outVect.vectors[i] = vect;
		end.x =trajectories[i].xyz[0];
		end.y =trajectories[i].xyz[1];
		end.z =trajectories[i].xyz[2];

		markArray.markers[i].points = {start, end};
		markArray.markers[i].scale.x = _arrowShaftDiameter;
		markArray.markers[i].scale.y = _arrowHeadDiameter;
		markArray.markers[i].scale.z = _arrowHeadLength;
		markArray.markers[i].color.a = 1.0; // Don't forget to set the alpha!
		markArray.markers[i].color.r = colors[i].val[0]/255.0;
		markArray.markers[i].color.g = colors[i].val[1]/255.0;
		markArray.markers[i].color.b = colors[i].val[2]/255.0;

	}

	if(_vis_pub.getNumSubscribers()>0 && _loops % (_markerSkip+1) == 0)_vis_pub.publish(markArray);
	if(_pubImage.getNumSubscribers()>0){
	  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayMat).toImageMsg();
          _pubImage.publish(msg);
        }
        if(_pubVect.getNumSubscribers()>0)_pubVect.publish(outVect);
	if(_pubContours.getNumSubscribers()>0){
	  sensor_msgs::ImagePtr msgC = cv_bridge::CvImage(std_msgs::Header(), "rgb8", drawing).toImageMsg();
          _pubContours.publish(msgC);
	}
        std::chrono::high_resolution_clock::time_point t_end3 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> visTime2(t_end3-t_start3);
	visTime2 += visTime1;
	if(_debug)ROS_INFO("VFH Complete");
	if(_timing){
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> algorithmTime(t_end-t_start);
		_averageAlgorithm += algorithmTime;
		_averageAlgorithm -= visTime2;
		_averageVis += visTime2;
		ROS_INFO_STREAM("VISUALIZATION TIME: " << visTime2.count() << " milliseconds "<<"AVERAGE VISUALIZATION TIME: "<<_averageVis.count()/_loops<<" milliseconds");
		ROS_INFO_STREAM("ALGORITHM TIME: " << algorithmTime.count() << " milliseconds "<<"AVERAGE ALGORITHM TIME: "<<_averageAlgorithm.count()/_loops<<" milliseconds");
	}
	return trajVec;

}

std::vector<pubHandler::sector> pubHandler::_extractPointsFromCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud){
	if(_debug)ROS_INFO("Extracting Points...");
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::vector<pubHandler::sector> sectors;
	static double aConverter = (360/(2*M_PI)/(360/(_AzRez-0)));
	static double eConverter = (360/(2*M_PI)/(180/(_ElRez-0)));
	sectors.reserve(cloud->points.size());
	sector tempSector;
	for(const pcl::PointXYZ& pt: cloud->points){
		double x = pt.x;
		double y = pt.y;
		double z = pt.z;
		double r = sqrt(x*x + y*y + z*z);
		double a = atan2(y,x);
		double e = acos(z/r);
			if(a<0){
				a += 2*M_PI;
			}
			if(e<0){
				e += M_PI;
			}
			tempSector.a=int(a*aConverter);
			tempSector.e=int(e*eConverter);
			tempSector.r = r;
			sectors.push_back(tempSector);

	}
	if(_debug)ROS_INFO("Points Extracted");
	if(_timing){
		std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> executionTime(t_end-t_start);
		_averageExtraction += executionTime;
		ROS_INFO_STREAM("EXTRACTION TIME: " << executionTime.count() << " milliseconds "<<"AVERAGE EXTRACTION TIME: "<<_averageExtraction.count()/_loops<<" milliseconds");
	}
	return sectors;
}
