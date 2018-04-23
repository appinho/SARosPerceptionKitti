/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <sensor_processing_lib/sensor_fusion.h>

namespace sensor_processing{

/******************************************************************************/

SensorFusion::SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh),
	pcl_in_(new VPointCloud),
	pcl_ground_plane_(new VPointCloud),
	pcl_ground_plane_inliers_(new VPointCloud),
	pcl_ground_plane_outliers_(new VPointCloud),
	pcl_ground_(new VPointCloud),
	pcl_elevated_(new VPointCloud),
	pcl_voxel_ground_(new VPointCloud),
	pcl_voxel_elevated_(new VPointCloud),
	pcl_semantic_(new VRGBPointCloud),
	cloud_sub_(nh, "/kitti/velo/pointcloud", 2),
	image_sub_(nh,	"/kitti/camera_color_left/image_raw", 2),
	sync_(MySyncPolicy(10), cloud_sub_, image_sub_){

	// Get scenario parameter
	int scenario;
	if(ros::param::get("~scenario", scenario)){
		std::ostringstream scenario_stream;
		scenario_stream << std::setfill('0') << std::setw(4) << scenario;
		params_.scenario = scenario_stream.str();
	}
	else{
		ROS_ERROR("Failed to read scenario");
	}

	// Define lidar parameters
	private_nh_.param("lidar_height", params_.lidar_height,
		params_.lidar_height);
	private_nh_.param("lidar_min_height", params_.lidar_min_height,
		params_.lidar_min_height);
	params_.lidar_opening_angle = M_PI / 4;

	// Define grid parameters
	private_nh_.param("grid_min_range", params_.grid_min_range,
		params_.grid_min_range);
	private_nh_.param("grid_max_range", params_.grid_max_range,
		params_.grid_max_range);
	private_nh_.param("grid_cell_size", params_.grid_cell_size,
		params_.grid_cell_size);
	private_nh_.param("grid_min_height", params_.grid_min_height,
		params_.grid_min_height);
	private_nh_.param("grid_segments", params_.grid_segments,
		params_.grid_segments);
	params_.grid_height = params_.grid_max_range / params_.grid_cell_size ;
	params_.grid_width = params_.grid_height * 2;
	params_.grid_bins = (params_.grid_max_range * std::sqrt(2)) /
		params_.grid_cell_size + 1;

	// Define ransac ground plane parameters
	private_nh_.param("ransac_tolerance", params_.ransac_tolerance,
		params_.ransac_tolerance);
	private_nh_.param("ransac_iterations", params_.ransac_iterations,
		params_.ransac_iterations);

	// Define static conversion values
	params_.inv_angular_res = 2 * params_.grid_segments / M_PI;
	params_.inv_radial_res = 1.0f / params_.grid_cell_size;

	// Print parameters
	ROS_INFO_STREAM("scenario " << params_.scenario);
	ROS_INFO_STREAM("grid_height " << params_.grid_height);
	ROS_INFO_STREAM("grid_width " << params_.grid_width);
	ROS_INFO_STREAM("grid_cell_size " << params_.grid_cell_size);
	ROS_INFO_STREAM("grid_min_height " << params_.grid_min_height);
	ROS_INFO_STREAM("grid_bins " << params_.grid_bins);
	ROS_INFO_STREAM("grid_segments " << params_.grid_segments);
	ROS_INFO_STREAM("ransac_tolerance " << params_.ransac_tolerance);
	ROS_INFO_STREAM("ransac_iterations " << params_.ransac_iterations);
	ROS_INFO_STREAM("inv_angular_res " << params_.inv_angular_res);
	ROS_INFO_STREAM("inv_radial_res " << params_.inv_radial_res);

	// Define polar grid
	polar_grid_ = std::vector< std::vector<PolarCell> >(params_.grid_segments,
		std::vector<PolarCell>(params_.grid_bins));

	// Define occupancy grid
	occ_grid_ = boost::make_shared<OccupancyGrid>();
	occ_grid_->data.resize(params_.grid_width * params_.grid_height);
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

	// Init occupancy grid
	for(int j = 0; j < params_.grid_height; ++j){
		for(int i = 0; i < params_.grid_width; ++i){

			// Never reach this cells because of opening angle
			if(i < j || i >= params_.grid_width - j){
				occ_grid_->data[j * params_.grid_width + i] = -1;
			}			
		}
	}

	// Define Publisher 
	cloud_filtered_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_filtered", 2);
	cloud_ground_plane_inliers_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_groundplane_inliers", 2);
	cloud_ground_plane_outliers_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_groundplane_outliers", 2);
	cloud_ground_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_ground", 2);
	cloud_elevated_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_elevated", 2);
	voxel_ground_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/voxel_ground", 2);
	voxel_elevated_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/voxel_elevated", 2);
	grid_occupancy_pub_ = nh_.advertise<OccupancyGrid>(
		"/sensor/grid_occupancy", 2);

	image_semantic_pub_ = nh_.advertise<Image>(
		"/sensor/image_semantic", 2);
	cloud_semantic_pub_ = nh_.advertise<PointCloud2>(
		"/sensor/cloud_semantic", 2);

	// Define Subscriber
	sync_.registerCallback(boost::bind(&SensorFusion::process, this, _1, _2));

	// Init counter for publishing
	time_frame_ = 0;
}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(
		const PointCloud2::ConstPtr & cloud,
		const Image::ConstPtr & image
	){

	// Preprocess point cloud
	processPointCloud(cloud);

	// Preprocess image
	processImage(image);

	// Fuse sensors by mapping elevated point cloud into semantic segmentated
	// image
	mapPointCloudIntoImage(pcl_elevated_);

	// Print sensor fusion
	ROS_INFO("Publishing Sensor Fusion [%d]: # PCL points [%d] # Ground [%d]"
		" # Elevated [%d] # Semantic [%d] ", time_frame_, int(pcl_in_->size()), 
		int(pcl_ground_->size()), int(pcl_elevated_->size()), 
		int(pcl_semantic_->size()));

	// Increment time frame
	time_frame_++;

}

void SensorFusion::processPointCloud(const PointCloud2::ConstPtr & cloud){

/******************************************************************************
 * 1. Filter point cloud to only consider points in the front that can also be
 * found in image space.
 */

	// Convert input cloud
	pcl::fromROSMsg(*cloud, *pcl_in_);

	// Define point_cloud_inliers and indices
	pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());
	pcl::ExtractIndices<VPoint> pcl_extractor;

	// Reset polar grid
	polar_grid_ = std::vector< std::vector<PolarCell> >(
		params_.grid_segments, std::vector<PolarCell>(params_.grid_bins));

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

					// Buffer variables
					int seg, bin;

					// Get polar grid cell indices
					fromVeloCoordsToPolarCell(point.x, point.y, seg, bin);

					// Grab cell
					PolarCell & cell = polar_grid_[seg][bin];

					// Increase count
					cell.count++;

					// Update min max
					if(cell.count == 1){
						cell.x_min = point.x;
						cell.y_min = point.y;
						cell.z_min = point.z;
						cell.z_max = point.z;
					}
					else{
						if(point.z < cell.z_min){
							cell.x_min = point.x;
							cell.y_min = point.y;
							cell.z_min = point.z;
						}
						if(point.z > cell.z_max){
							cell.z_max = point.z;
						}
					}
				}
			}
		}
	}

	// Extract points from original point cloud
	pcl_extractor.setInputCloud(pcl_in_);
	pcl_extractor.setIndices(pcl_inliers);
	pcl_extractor.setNegative(false);
	pcl_extractor.filter(*pcl_in_);

	// Publish filtered cloud
	pcl_in_->header.frame_id = cloud->header.frame_id;
	pcl_in_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
	cloud_filtered_pub_.publish(pcl_in_);

/******************************************************************************
 * 2. Ground plane estimation and dividing point cloud in elevated and ground
 */
	// Clear ground plane points
	pcl_ground_plane_->points.clear();

	// Loop over polar grid
	for(int i = 0; i < params_.grid_segments; ++i){
		for(int j = 0; j < params_.grid_bins; ++j){

			// Grab cell
			PolarCell & cell = polar_grid_[i][j];

			// Check if cell can be ground cell
			if(cell.count > 0 &&
				(cell.z_max - cell.z_min < params_.grid_min_height)){

				// Push back cell attributes to ground plane cloud
				pcl_ground_plane_->points.push_back(
					VPoint(cell.x_min, cell.y_min, cell.z_min));
			}
		}
	}

	// Estimate the ground plane using PCL and RANSAC
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<VPoint> segmentation;
	segmentation.setOptimizeCoefficients(true);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(params_.ransac_tolerance);
	segmentation.setMaxIterations(params_.ransac_iterations);
	segmentation.setInputCloud(pcl_ground_plane_->makeShared());
	segmentation.segment(*inliers, *coefficients);

	// Divide ground plane cloud in inlier cloud and outlier cloud
	pcl_extractor.setInputCloud(pcl_ground_plane_);
	pcl_extractor.setIndices(inliers);
	pcl_extractor.setNegative(false);
	pcl_extractor.filter(*pcl_ground_plane_inliers_);

	pcl_extractor.setInputCloud(pcl_ground_plane_);
	pcl_extractor.setIndices(inliers);
	pcl_extractor.setNegative(true);
	pcl_extractor.filter(*pcl_ground_plane_outliers_);

	// Estimate ground height in the center of the polar map
	float ground_height = -coefficients->values[3] / coefficients->values[2];

	// Sanity check
	if(inliers->indices.empty() || ground_height < -2 || ground_height > -1.5){
		ROS_WARN("Bad ground plane estimation! # Ransac Inliers [%d] # Lidar "
			"height [%f]", int(inliers->indices.size()), ground_height);
	}

	// Publish ground plane inliers and outliers point cloud
	pcl_ground_plane_inliers_->header.frame_id = cloud->header.frame_id;
	pcl_ground_plane_inliers_->header.stamp = 
		pcl_conversions::toPCL(cloud->header.stamp);
	cloud_ground_plane_inliers_pub_.publish(pcl_ground_plane_inliers_);

	pcl_ground_plane_outliers_->header.frame_id = cloud->header.frame_id;
	pcl_ground_plane_outliers_->header.stamp = 
		pcl_conversions::toPCL(cloud->header.stamp);
	cloud_ground_plane_outliers_pub_.publish(pcl_ground_plane_outliers_);

	// Print
	ROS_INFO("Ground plane estimation [%d] # Points [%d] # Inliers [%d] Lidar "
		" height [%f], C [%f][%f][%f][%f]",	time_frame_, 
		int(pcl_ground_plane_->size()),	int(pcl_ground_plane_inliers_->size()), 
		ground_height, coefficients->values[0], coefficients->values[1],
		coefficients->values[2], coefficients->values[3]);

/******************************************************************************
 * 3. Evaluate segments of polar grid to fill with unknown, free or occupied
 */
	
	// Loop over segments
	for(int s = 0; s < params_.grid_segments; s++){

		// Set hit to false
		bool hit = false;

		// Loop over bins
		for(int b = 0; b < params_.grid_bins; b++){
		
			// Grab cell
			PolarCell & cell = polar_grid_[s][b];

			// Buffer variables
			float x,y;

			// Get velodyne coodinates
			fromPolarCellToVeloCoords(s, b, x, y);

			// Get ground height
			cell.ground = (-coefficients->values[0] * x -
				coefficients->values[1] * y - coefficients->values[3]) /
				coefficients->values[2];

			// If cell is not filled
			if(cell.count == 0){

				// And has hit sth so far mark as unknown
				if(hit)
					cell.idx = PolarCell::UNKNOWN;

				// And has not hit sth so far mark as free
				else
					cell.idx = PolarCell::FREE;

				continue;
			}
			else{

				// Calculate cell height
				cell.height = polar_grid_[s][b].z_max - cell.ground;

				// If cell height big enough fill cell as occupied
				if(cell.height > params_.grid_min_height){

					cell.idx = PolarCell::OCCUPIED;

					// Mark segment as hit
					hit = true;
				}
				else{
					
					// And has hit sth so far mark as unknown
					if(hit)
						cell.idx = PolarCell::UNKNOWN;

					// And has not hit sth so far mark as free
					else
						cell.idx = PolarCell::FREE;
				}
			}
		}	
	}

	// Divide filtered point cloud in elevated and ground
	pcl_ground_->points.clear();
	pcl_elevated_->points.clear();

	for(int i = 0; i < pcl_in_->size(); ++i){

		// Read current point
		VPoint point = pcl_in_->at(i);

		// Buffer variables
		int seg, bin;

		// Get polar grid cell indices
		fromVeloCoordsToPolarCell(point.x, point.y, seg, bin);

		// Grab cell
		PolarCell & cell = polar_grid_[seg][bin];

		if(point.z > cell.ground && cell.height > params_.grid_min_height){
			pcl_elevated_->points.push_back(point);
		}
		else{
			pcl_ground_->points.push_back(point);
		}
	}

	// Publish ground cloud
	pcl_ground_->header.frame_id = cloud->header.frame_id;
	pcl_ground_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
	cloud_ground_pub_.publish(pcl_ground_);

	// Publish elevated cloud
	pcl_elevated_->header.frame_id = cloud->header.frame_id;
	pcl_elevated_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
	cloud_elevated_pub_.publish(pcl_elevated_);	

/******************************************************************************
 * 4. Map polar grid back to cartesian occupancy grid
 */
	// Clear voxel pcls
	pcl_voxel_elevated_->points.clear();
	pcl_voxel_ground_->points.clear();

	// Go through cartesian grid
	float x = params_.grid_max_range - params_.grid_cell_size / 2;
	for(int j = 0; j < params_.grid_height; ++j, x -= params_.grid_cell_size){
		float y = x;
		for(int i = j; i < params_.grid_width - j; ++i,
			y -= params_.grid_cell_size){

			// Buffer variables
			int seg, bin;

			// Get polar grid cell indices
			fromVeloCoordsToPolarCell(x, y, seg, bin);
			
			// Grab polar cell
			PolarCell & cell = polar_grid_[seg][bin];

			// Fill ground voxel cloud
			pcl_voxel_ground_->points.push_back( VPoint(x, y, cell.ground) );

			// Calculate occupancy grid cell index
			int cell_index = j * params_.grid_width + i;

			// If cell is free
			if(cell.idx == PolarCell::FREE)
				occ_grid_->data[cell_index] = 0;

			// If cell is unknown
			else if(cell.idx == PolarCell::UNKNOWN)
				occ_grid_->data[cell_index] = 50;

			// If cell is occupied
			else{
				occ_grid_->data[cell_index] = 100;

				// Fill elevated voxel cloud
				for(float v = cell.ground; v < cell.z_max; 
					v += params_.grid_cell_size){
						pcl_voxel_elevated_->points.push_back(
							VPoint(x, y, v));
				}
			}
		}
	}

	// Publish voxel ground
	pcl_voxel_ground_->header.frame_id = cloud->header.frame_id;
	pcl_voxel_ground_->header.stamp = 
		pcl_conversions::toPCL(cloud->header.stamp);
	voxel_ground_pub_.publish(pcl_voxel_ground_);

	// Publish voxel elevated
	pcl_voxel_elevated_->header.frame_id = cloud->header.frame_id;
	pcl_voxel_elevated_->header.stamp = 
		pcl_conversions::toPCL(cloud->header.stamp);
	voxel_elevated_pub_.publish(pcl_voxel_elevated_);

	// Publish occupancy grid
	occ_grid_->header.stamp = cloud->header.stamp;
	occ_grid_->header.frame_id = cloud->header.frame_id;
	occ_grid_->info.map_load_time = occ_grid_->header.stamp;
	grid_occupancy_pub_.publish(occ_grid_);
}

void SensorFusion::processImage(const Image::ConstPtr & image){

/******************************************************************************
 * 1. Load precalculated semantic segmentated images to ensure online
 * performance
 */

	// Define path
	std::ostringstream path_name;
	path_name << "/home/simonappel/kitti_data/"
		<< params_.scenario 
		<< "/segmented_semantic_images/"
		<< std::setfill('0') << std::setw(10)	<< time_frame_ << ".png";

	// Load semantic segmentated image
	sem_image_ = cv::imread(path_name.str(), CV_LOAD_IMAGE_COLOR);

	// Sanity check if image is loaded correctly
	if(sem_image_.cols == 0 || sem_image_.rows == 0){
		ROS_WARN("Semantic image not read properly!");
		return;
	}

	// Publish
	cv_bridge::CvImage cv_semantic_image;
	cv_semantic_image.image = sem_image_;
	cv_semantic_image.encoding = "bgr8";
	cv_semantic_image.header.stamp = image->header.stamp;
	image_semantic_pub_.publish(cv_semantic_image.toImageMsg());
}

void SensorFusion::mapPointCloudIntoImage(const VPointCloud::Ptr cloud){

/******************************************************************************
 * 1. Convert velodyne points into image space
 */

	// Define matrix to write velodyne points into
	MatrixXf matrix_velodyne_points = MatrixXf::Zero(4, cloud->size());
	for(int i = 0; i < cloud->size(); ++i){
		matrix_velodyne_points(0,i) = cloud->points[i].x;
		matrix_velodyne_points(1,i) = cloud->points[i].y;
		matrix_velodyne_points(2,i) = cloud->points[i].z;
		matrix_velodyne_points(3,i) = 1;
	}

	// Project the matrix from velodyne coordinates to the image plane
	MatrixXf matrix_image_points = 
		tools_.transformVeloToImage(matrix_velodyne_points);

	// Get image format
	int img_width = sem_image_.cols;
	int img_height = sem_image_.rows;

	// Clear semantic cloud
	pcl_semantic_->points.clear();

	// Loop over image points
	for(int i = 0; i < matrix_image_points.cols(); i++){

		// Check if image point is valid
		const int & img_x = matrix_image_points(0, i);
		const int & img_y = matrix_image_points(1, i);
		const int & img_z = matrix_image_points(2, i);

		if( (img_x >= 0 && img_x < img_width) &&
			(img_y >= 0 && img_y < img_height) &&
			(img_z >= 0)){

			// Get R G B values of semantic image
			uint8_t r = sem_image_.at<cv::Vec3b>(img_y,img_x)[2];
			uint8_t g = sem_image_.at<cv::Vec3b>(img_y,img_x)[1];
			uint8_t b = sem_image_.at<cv::Vec3b>(img_y,img_x)[0];

			// Create new point and fill it
			VRGBPoint point;
			point.x = cloud->points[i].x;
			point.y = cloud->points[i].y;
			point.z = cloud->points[i].z;
			point.r = r;
			point.g = g;
			point.b = b;

			// Push back point
			pcl_semantic_->points.push_back(point);
		}
	}

	// Publish semantic cloud
	pcl_semantic_->header.frame_id = cloud->header.frame_id;
	pcl_semantic_->header.stamp = cloud->header.stamp;
	cloud_semantic_pub_.publish(pcl_semantic_);
}

void SensorFusion::fromVeloCoordsToPolarCell(const float x, const float y,
		int & seg, int & bin){

	float mag = std::sqrt(x * x + y * y);
	float ang = -std::atan2(y, x);
	seg = int((ang + params_.lidar_opening_angle) * params_.inv_angular_res);
	bin = int(mag * params_.inv_radial_res);

	// For last segment
	if(x == -y)
		seg = params_.grid_segments - 1;
}

void SensorFusion::fromPolarCellToVeloCoords(const int seg, const int bin,
		float & x, float & y){

	float mag = bin / params_.inv_radial_res;
	float ang = seg / params_.inv_angular_res - params_.lidar_opening_angle;
	y = - std::sin(ang) * mag;
	x = std::cos(ang) * mag;
}


} // namespace sensor_processing