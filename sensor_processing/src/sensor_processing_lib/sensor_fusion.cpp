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
	private_nh_.param("lidar_min_height", params_.lidar_min_height,
		params_.lidar_min_height);
	params_.lidar_opening_angle = M_PI / 4;

	// Read grid parameters
	private_nh_.param("grid_min_range", params_.grid_min_range,
		params_.grid_min_range);
	private_nh_.param("grid_max_range", params_.grid_max_range,
		params_.grid_max_range);
	private_nh_.param("grid_cell_size", params_.grid_cell_size,
		params_.grid_cell_size);
	private_nh_.param("grid_segments", params_.grid_segments,
		params_.grid_segments);
	params_.grid_height = params_.grid_max_range / params_.grid_cell_size ;
	params_.grid_width = params_.grid_height * 2;
	params_.grid_bins = (params_.grid_max_range * std::sqrt(2)) /
		params_.grid_cell_size + 1;

	// Define static conversion values
	params_.inv_angular_res = 1.0f / (2 * params_.lidar_opening_angle /
		params_.grid_segments);
	params_.inv_radial_res = 1.0f / params_.grid_cell_size;

	// Define polar grid
	polar_grid_ = std::vector< std::vector<PolarCell> >(params_.grid_segments,
		std::vector<PolarCell>(params_.grid_bins));

	// Define occupancy grid
	occ_grid_ = boost::make_shared<nav_msgs::OccupancyGrid>();
	occ_grid_->data.resize(params_.grid_width * params_.grid_height);

	// Init occupancy grid
	for(int j = 0; j < params_.grid_height; ++j){
		for(int i = 0; i < params_.grid_width; ++i){

			// Never reach this cells because of opening angle
			if(i < j || i >= params_.grid_width - j){
				occ_grid_->data[j * params_.grid_width + i] = 0;
			}
			// Fill unknown
			else{
				occ_grid_->data[j * params_.grid_width + i] = -1;
			}
			
		}
	}

	// Fill in all message members
	occ_grid_->info.width = uint32_t(params_.grid_width);
	occ_grid_->info.height = uint32_t(params_.grid_height);
	occ_grid_->info.resolution = float(params_.grid_cell_size);
	occ_grid_->info.origin.position.x = params_.grid_max_range;
	occ_grid_->info.origin.position.y = params_.grid_max_range;
	occ_grid_->info.origin.position.z = params_.lidar_height;
	occ_grid_->info.origin.orientation.w = 0;
	occ_grid_->info.origin.orientation.x = 0.707;
	occ_grid_->info.origin.orientation.y = -0.707;
	occ_grid_->info.origin.orientation.z = 0;

	// Define Publisher & Subscriber
	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
		"/sensor/pointcloud", 10);
	occ_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
		"/sensor/occ_grid", 10);
	cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
		"/kitti/velo/pointcloud", 2, &SensorFusion::process, this);

	std::cout << params_.grid_segments << " " << params_.grid_bins << std::endl;

}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(
		const sensor_msgs::PointCloud2::ConstPtr & cloud
		//const sensor_msgs::Image::ConstPtr & image
	){

/******************************************************************************
 * 1. Filter point cloud to only consider points in the front that can also be
 * found in image space.
 */

	// Convert input cloud
	pcl::fromROSMsg(*cloud, *pcl_in_);

	// Define point_cloud_inliers and indices
	pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());
	pcl::ExtractIndices<VPoint> pcl_extractor;

	// Buffer variables
	int seg, bin;

	// Reset polar grid
	polar_grid_ = std::vector< std::vector<PolarCell> >(params_.grid_segments,
		std::vector<PolarCell>(params_.grid_bins));

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

					// Convert into polar grid cell
					fromVeloCoordsToPolarCell(point.x, point.y, seg, bin);

					// TODO change to sth correct
					polar_grid_[seg][bin].height = 1;

					// Sanity check
					if(seg >= params_.grid_segments || bin >= params_.grid_bins)
						ROS_WARN("Out of grid [%d][%d]", seg, bin);
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

/******************************************************************************
 * 2. Map polar grid back to cartesian occupancy grid
 */

	// Occupancy grid
	occ_grid_->header.stamp = cloud->header.stamp;
	occ_grid_->header.frame_id = cloud->header.frame_id;
	occ_grid_->info.map_load_time = occ_grid_->header.stamp;

	// Remember occupied cells
	std::vector<int> occ_grid_cell_indices;

	// Go through cartesian grid
	float x = params_.grid_max_range - params_.grid_cell_size / 2;
	for(int j = 0; j < params_.grid_height; ++j, x -= params_.grid_cell_size){
		float y = x;
		for(int i = j; i < params_.grid_width - j; ++i,
			y -= params_.grid_cell_size){

			// Get polar grid cell
			fromVeloCoordsToPolarCell(x, y, seg, bin);

			if(polar_grid_[seg][bin].height == 1){
				int cell_index = j * params_.grid_width + i;
				occ_grid_->data[cell_index] = 100;
				occ_grid_cell_indices.push_back(cell_index);
			}
		}
	}

	// Publish the map
	ROS_INFO("Publishing Sensor Fusion:	# PCL points [%d], "
		"# Occ Grid Cells [%d] ",
		int(pcl_in_->size()),
		int(occ_grid_cell_indices.size()));

	occ_grid_pub_.publish(occ_grid_);

	for(int i = 0; i < occ_grid_cell_indices.size(); ++i){
		occ_grid_->data[ occ_grid_cell_indices[i] ] = -1;
	}
}

void SensorFusion::fromVeloCoordsToPolarCell(const float x, const float y,
		int & seg, int & bin){

	float mag = std::sqrt(x * x + y * y);
	float ang = -std::atan2(y, x);
	seg = int((ang + params_.lidar_opening_angle - 0.01) * params_.inv_angular_res);
	bin = int(mag * params_.inv_radial_res);

}

} // namespace sensor_processing