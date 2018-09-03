//============================================================================
// Name        : cloudTransformer.cpp
// Author      : Ryan Fite - ryanfite@live.com
// Version     : 1.0
// Date Created: Jul 27, 2018
// Description : 
//============================================================================




#include <ros/ros.h>
#include <tf/transform_listener.h>
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

std::shared_ptr<tf::TransformListener> tfListener;
ros::Publisher pubPoints;

std::string base_frame = "base_link";
tf::StampedTransform stampedTf;
geometry_msgs::TransformStamped gm_tfStamped;
  
void messageReceivedCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg){
    try{
      tfListener->waitForTransform(base_frame, msg->header.frame_id, pcl_conversions::fromPCL(msg->header.stamp), ros::Duration(0.1));
      tfListener->lookupTransform(base_frame, msg->header.frame_id, pcl_conversions::fromPCL(msg->header.stamp), stampedTf);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    auto temp = boost::make_shared<std::remove_const<std::remove_reference<decltype(*msg)>::type>::type >(*msg);
        
    tf::transformStampedTFToMsg(stampedTf, gm_tfStamped);
	Eigen::Affine3d affine = tf2::transformToEigen(gm_tfStamped);
	pcl::transformPointCloud(*temp, *temp, affine);
	temp->header.frame_id = base_frame;
	pubPoints.publish(*temp);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cloudTransformer");
  
  ros::NodeHandle nh("cloudTransformer");

  nh.param("base_frame", base_frame, base_frame);
  
  tfListener = std::make_shared<tf::TransformListener>();
  pubPoints = nh.advertise<sensor_msgs::PointCloud2>("points", 10);

  ros::Subscriber subPoints = nh.subscribe("/velodyne_points", 10, &messageReceivedCloud);
 
  ros::spin();
  return 0;
};
