// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Point.h>
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

// Evaluator
Evaluation evaluator("/home/simonappel/Coding/RosbagKitti/0005/tracklet_labels.xml");

// Publisher
ros::Publisher pcl_pub;
ros::Publisher dbb_pub;
ros::Publisher gt_pub;

// TODO Remove this later with bug fix detection
int num_last_objects = 0;

void show_detection(const std::vector<Cluster> & clusters){

  visualization_msgs::MarkerArray marker_array;

  // Loop through clusters
  for(int i = 0; i < clusters.size(); ++i){

    // Create marker and fill it
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i;
    marker.text = "OBJECT";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = clusters[i].x;
    marker.pose.position.y = clusters[i].y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = clusters[i].l_x;
    marker.scale.y = clusters[i].l_y;
    marker.scale.z = 0.1;
    marker.color.a = 0.3; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.frame_locked = true;
    marker_array.markers.push_back(marker);

    // std::cout << marker.id << "    " << marker.pose.position.x << " " << marker.pose.position.y 
    //   << " " << marker_array.markers.size() << std::endl;
  }

  if(num_last_objects > clusters.size()){
    std::cout << "Deleted ";
    for(int i = clusters.size(); i < num_last_objects; ++i){
      marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
      marker_array.markers[i].color.a = 0.0;
      std::cout << i << " (" << marker_array.markers[i].pose.position.x << ","
        << marker_array.markers[i].pose.position.y << ") ";
    }
    std::cout << std::endl;
  }
  num_last_objects = clusters.size();

  dbb_pub.publish(marker_array);
}

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

  //std::cout << cloud->size() << std::endl;

  detector.runConnectedComponent(cloud);
  show_detection(detector.getClusters());

  // Tracker
  tracker.processMeasurements(detector.getClusters());

  // Evaluator
  //visualization_msgs::Marker bike_marker = evaluator.plotBike();
  visualization_msgs::MarkerArray groundtruthdata = evaluator.showTracklets();
  gt_pub.publish(groundtruthdata);

  // Publish the data
  ROS_INFO("#PCL points [%d], #Clusters [%d]", int(cloud->size()), int(detector.getClusters().size()));
  pcl_pub.publish(cloud);
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& msg){

  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", 
    msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  ego_orientation = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}


int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "kitti_pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub_pcl = nh.subscribe ("/kitti/velo/pointcloud", 1, callback_pcl);

  // Create a ROS subscriber for the IMU data
  ros::Subscriber sub_imu = nh.subscribe ("/kitti/oxts/imu", 1, callback_imu);

  // Create a ROS publisher for the output point cloud
  pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud", 1);

  // Create a ROS publisher for the detected bounding boxes
  dbb_pub = nh.advertise<visualization_msgs::MarkerArray>( "detection", 100);

  // Create a ROS publisher for the ground truth data
  gt_pub = nh.advertise<visualization_msgs::MarkerArray>( "groundtruth", 100);

  // Spin
  ros::spin ();
}