//============================================================================
// Name        : pubHandler.h
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 6, 2018
// Description :
//============================================================================

#ifndef PUBHANDLER_H_
#define PUBHANDLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_eigen/tf2_eigen.h"
#include "Eigen/Geometry"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2/LinearMath/Quaternion.h"
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include "opencv2/core/mat.hpp"
#include <iostream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <sstream>
#include <boost/foreach.hpp>
#include <cmath>

using namespace cv;
using namespace std;
class pubHandler{
public:
	/*Public Functions*/
	pubHandler(ros::NodeHandle n, const std::string& s, int num);
	void messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);
	void messageReceivedPose(const nav_msgs::Odometry::ConstPtr& msg);
	Eigen::Quaterniond differenceOfQuat(Eigen::Quaterniond start, Eigen::Quaterniond end);
	Eigen::Vector3d differenceOfVec(Eigen::Vector3d start, Eigen::Vector3d end);
	/* Public Structs*/
	struct sector{
		int a;
		int e;
		double r;
	};
	struct trajectory{
		double sectorX;
		double sectorY;
		double magnitude;
		std::vector<double> xyz;
	};

private:
	/* Private Variables*/
	ros::Publisher _pubPoints;
	ros::Publisher _vis_pub;
	image_transport::Publisher _pubImage;
	image_transport::Publisher _pubContours;
	image_transport::Publisher _pubVote;
	pcl::PointCloud<pcl::PointXYZ>::Ptr _data;
	std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr > _window;
	std::deque<nav_msgs::Odometry> _odomWindow;
	tf2_ros::Buffer _tfBuffer;
	tf2_ros::TransformListener _tfListener;
	int _alignmentSwitch;
	int _queueSize;
	int _queueCurrentSize;
	int _count;
	int _AzRez;
	int _ElRez;
	int _dIterations;
	int _eIterations;
	int _blurSizeY;
	int _blurSizeX;
	int _boxSizeY;
	int _boxSizeX;
	int _medianSize;
	int _skipCounter;
	int _loops;

	double _blurSigmaY;
	double _areaRestricter;
	double _blurSigmaX;
	double _voxelSize;
	double _voxelUniformSize;
	double _splitter;
	double _rejection;
	double _iOffset;

	std::chrono::duration<double, std::milli> _averageExecution;
	std::chrono::duration<double, std::milli> _averagePreprocessing;
	std::chrono::duration<double, std::milli> _averageAlgorithm;
	std::chrono::duration<double, std::milli> _averageQueueing;
	std::chrono::duration<double, std::milli> _averageVis;
	std::chrono::duration<double, std::milli> _averageExtraction;
	std::chrono::duration<double, std::milli> _averageRad;
	std::chrono::duration<double, std::milli> _averageTransform;
	std::chrono::duration<double, std::milli> _averageVoxel;

	std::vector<Scalar> _colors;

	bool _debug;
	bool _timing;
	bool _median;
	bool _blur;
	bool _box;
	bool _dilate;
	bool _erode;
	bool _uniformGrid;


	/*Private Functions*/
	std::vector<double> _convertToCartesian(std::vector<double> aer);
	std::vector<pubHandler::sector> _extractPointsFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	std::map<std::string,std::vector<trajectory> > _freeTrajectories(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	cv::Mat _radmatrix(std::vector<sector> points);
	pcl::PointCloud<pcl::PointXYZ> _preprocessing(std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr > window,std::deque<nav_msgs::Odometry> odomWindow);
	pcl::PointCloud<pcl::PointXYZ> _preprocessingNew(std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr > window,std::deque<nav_msgs::Odometry> odomWindow);
};

#endif /* PUBHANDLER_H_ */
