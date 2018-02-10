// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Publisher
ros::Publisher pub;

// Parameters
const float voxel_size = 0.2;
const float opening_angle = M_PI/3;
const float minimum_height = -1.4;

const bool filter_pointcloud = true;
const bool convert_to_voxelgrid = false;

void callback(const sensor_msgs::PointCloud2ConstPtr& input){

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);

  // Filter the pointcloud
  if(filter_pointcloud){
    
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
      if(angle < opening_angle){
        // Check minimum height
        //if(point.z > minimum_height && angle < opening_angle){
          inliers->indices.push_back(i);
        //}
      }
    }

    // Extract points
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  // Convert to VoxelGrid
  if(convert_to_voxelgrid){

    // Define VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*cloud);
  }

  // Publish the data
  pub.publish(cloud);
}

int main (int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "kitti_pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kitti/velo/pointcloud", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}