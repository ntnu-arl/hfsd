//============================================================================
// Name        : cloudTransformer.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 27, 2018
// Description : 
//============================================================================




#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include "Eigen/Geometry"
#include "pcl/common/transforms.h"
#include "tf2_eigen/tf2_eigen.h"
#include <ros/callback_queue.h>
using namespace std;
string base_frame = "base_link";
ros::Publisher pubPoints;
geometry_msgs::TransformStamped transformStamped;
void messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>(*msg));
	Eigen::Affine3d affine = tf2::transformToEigen(transformStamped);
	pcl::transformPointCloud(*temp,*temp,affine);
	temp->header.frame_id = base_frame;
	pubPoints.publish(*temp);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cloudTransformer");
  std::string topicPoints;


  string sensor_frame = "camera";


  ros::NodeHandle node("cloudTransformer");

  node.param("sensor_frame",sensor_frame,sensor_frame);
  node.param("base_link",base_frame,base_frame);
  ros::Subscriber subPoints = node.subscribe<pcl::PointCloud<pcl::PointXYZ> >("inputcloud",10,&messageReceivedCloud);
  pubPoints = node.advertise<sensor_msgs::PointCloud2 >("points", 10);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){

    try{
      transformStamped = tfBuffer.lookupTransform(base_frame, sensor_frame,
                               ros::Time(0));

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
    rate.sleep();
  }
  return 0;
};
