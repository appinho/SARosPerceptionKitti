/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <sensor_processing/sensor_fusion.h>

namespace sensor_processing{

/******************************************************************************/

SensorFusion::SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh),
	pcl_in_(new VPointCloud){

	// Read lidar parameters
	private_nh_.param("lidar_height", params_.lidar_height,
		params_.lidar_height);
	private_nh_.param("lidar_opening_angle", params_.lidar_opening_angle,
		params_.lidar_opening_angle);
	private_nh_.param("lidar_min_height", params_.lidar_min_height,
		params_.lidar_min_height);

	// Read grid parameters
	private_nh_.param("grid_min_range", params_.grid_min_range,
		params_.grid_min_range);
	private_nh_.param("grid_max_range", params_.grid_max_range,
		params_.grid_max_range);
	private_nh_.param("grid_cell_size", params_.grid_cell_size,
		params_.grid_cell_size);
	params_.grid_height = params_.grid_max_range / params_.grid_cell_size ;
	params_.grid_width = params_.grid_height * 2;

	// Define Publisher & Subscriber
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
		"/sensor/pointcloud", 10);
	cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
		"/kitti/velo/pointcloud", 2, &SensorFusion::process, this);

}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(
		const sensor_msgs::PointCloud2::ConstPtr & cloud
		//const sensor_msgs::Image::ConstPtr & image
	){

	// Convert input cloud
	pcl::fromROSMsg(*cloud, *pcl_in_);

	// Define point_cloud_inliers and indices
	pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());
	pcl::ExtractIndices<VPoint> pcl_extractor;

	// Loop through input point cloud
	for(int i = 0; i < pcl_in_->size(); ++i){

		// Read current point
		VPoint & point = pcl_in_->at(i);

		// Determine angle of lidar point and check
		float angle = std::abs( std::atan2(point.y, point.x) );
		if(angle < params_.lidar_opening_angle){

			// Determine range of lidar point and check
			float range = std::sqrt(point.x * point.x + point.y * point.y);
			if(range > params_.grid_min_range &&
				range < params_.grid_max_range){

				// Check height of lidar point
				if(point.z > params_.lidar_min_height){

					// Add index for filtered point cloud
					pcl_inliers->indices.push_back(i);

				}
			}
		}
	}

	// Extract points from original point cloud
	pcl_extractor.setInputCloud(pcl_in_);
	pcl_extractor.setIndices(pcl_inliers);
	pcl_extractor.setNegative(false);
	pcl_extractor.filter(*pcl_in_);

	// Publish and print
	pcl_in_->header.frame_id = cloud->header.frame_id;
	pcl_in_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
	cloud_pub_.publish(pcl_in_);
	ROS_INFO("Sensor Fusion: Input received [%d]", int(pcl_in_->size()));
}

} // namespace sensor_processing