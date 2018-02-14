// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Project includes
#include "../include/test_kitti/evaluation.h"

// IMU data
tf::Quaternion ego_orientation;

// Detector
Detection detector;

// Tracker
Tracking tracker;
double time_stamp;

// Evaluator
Evaluation evaluator("/home/simonappel/Coding/RosbagKitti/0005/tracklet_labels.xml");

// Publisher
ros::Publisher pcl_pub;
ros::Publisher dbb_pub;
ros::Publisher tra_pub;
ros::Publisher gt_pub;

void callback_pcl(const sensor_msgs::PointCloud2ConstPtr& input){

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  // Filter the pointcloud
  if(PCL_FILTER_POINTCLOUD){
    
    // Define inliers and indices
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Loop through point cloud
    for(int i = 0; i < cloud->size(); ++i){

      // Read point
      pcl::PointXYZ point;
      point = cloud->at(i);

      // Determine angle of laser point
      float angle = std::abs( std::atan2(point.y, point.x) );

      // Check opening angle
      if(angle < PCL_FILTER_OPENING_ANGLE){
        // Check range
        float range = std::sqrt(point.x * point.x + point.y * point.y);
        if(range > PCL_FILTER_MIN_RANGE && range < PCL_FILTER_MAX_RANGE){
          // Check minimum height
          if(point.z > PCL_FILTER_MIN_HEIGHT){
            inliers->indices.push_back(i);
          }
        }
      }
    }

    // Extract points
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  // Convert to VoxelGrid
  if(PCL_CONVERT2VOX){

    // Define VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(PCL_VOXEL_SIZE, PCL_VOXEL_SIZE, PCL_VOXEL_SIZE);
    sor.filter(*cloud);
  }
  // Publish the filtered point cloud
  ROS_INFO("#PCL points [%d], #Clusters [%d]", int(cloud->size()), int(detector.getClusters().size()));
  pcl_pub.publish(cloud);

  // Detector
  detector.runConnectedComponent(cloud);
  visualization_msgs::MarkerArray detected_bounding_boxes =
    detector.showDetection();
  dbb_pub.publish(detected_bounding_boxes);

  // Tracker
  time_stamp = input->header.stamp.toSec();
  //ROS_INFO("#Received PCL time stamp [%f]", time_stamp);
  tracker.processMeasurements(detector.getClusters(), time_stamp);
  visualization_msgs::MarkerArray tracked_bounding_boxes =
    tracker.showTracks();
  tra_pub.publish(tracked_bounding_boxes);

  // Evaluator
  visualization_msgs::MarkerArray ground_truth_bounding_boxes =
    evaluator.showTracklets();
  gt_pub.publish(ground_truth_bounding_boxes);
  evaluator.showBikeRMSE(tracker.getTracks()[0]);
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){

  // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", 
  //   msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  // ego_orientation = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

void callback_gps_fix(const sensor_msgs::NavSatFix::ConstPtr& msg){

  //time_stamp = msg->header.stamp.toSec();
  //ROS_INFO("#Received GPS time stamp [%f]", time_stamp);
}

void callback_gps_vel(const geometry_msgs::TwistStamped::ConstPtr& msg){

//  std::cout << "Gps vel Seq: " << std::endl;
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", 
//     msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//   ego_orientation = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}


int main (int argc, char** argv){

  // Initialize ROS
  std::cout << "START" << std::endl;
  ros::init (argc, argv, "kitti_pcl");
  ros::NodeHandle nh;

  // SUBSCRIBER
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_pcl = nh.subscribe ("/kitti/velo/pointcloud", 1, callback_pcl);
  // Create a ROS subscriber for the IMU data
  ros::Subscriber sub_imu = nh.subscribe ("/kitti/oxts/imu", 1, callback_imu);
    // Create two ROS subscribers for the GPS data
  ros::Subscriber sub_gps_fix = nh.subscribe ("/kitti/oxts/gps/fix", 1, callback_gps_fix);
  ros::Subscriber sub_gps_vel = nh.subscribe ("/kitti/oxts/gps/vel", 1, callback_gps_vel);

  // PUBLISHER
  // Create a ROS publisher for the output point cloud
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);
  // Create a ROS publisher for the detected bounding boxes
  dbb_pub = nh.advertise<visualization_msgs::MarkerArray>( "detection", DET_BUFFER_SIZE);
    // Create a ROS publisher for the tracked bounding boxes
  tra_pub = nh.advertise<visualization_msgs::MarkerArray>( "tracking", TRA_BUFFER_SIZE);
  // Create a ROS publisher for the ground truth data
  gt_pub = nh.advertise<visualization_msgs::MarkerArray>( "groundtruth", GT_BUFFER_SIZE);

  // Spin
  ros::spin ();
}