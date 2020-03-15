#include "sensors_lib/ground_extraction.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace sensors{


GroundExtraction::GroundExtraction(ros::NodeHandle nh,
	ros::NodeHandle pnh):
	nh_(nh),
	pnh_(pnh)
	// pcl_ground_plane_(new VPointCloud),
	// pcl_ground_plane_inliers_(new VPointCloud),
	// pcl_ground_plane_outliers_(new VPointCloud),
	// pcl_ground_(new VPointCloud),
	// pcl_elevated_(new VPointCloud),
	// pcl_voxel_ground_(new VPointCloud),
	// pcl_voxel_elevated_(new VPointCloud),
	// pcl_semantic_(new VRGBPointCloud),
	// pcl_sparse_semantic_(new VRGBPointCloud),
	{

	// Define sensor_frame parameters
	// ros::NodeHandle pnh("~");

	pnh_.param("sensor_frame/z",
		sensor_frame_z_, sensor_frame_z_);
	pnh_.param("sensor_frame/z_min",
		sensor_frame_z_min_, sensor_frame_z_min_);
	sensor_frame_opening_angle_ = M_PI / 4;

	// Define polar grid parameters
	pnh_.param("polar_grid/range/min", 
		polar_grid_range_min_, polar_grid_range_min_);
	pnh_.param("polar_grid/range/max", 
		polar_grid_range_max_, polar_grid_range_max_);
	pnh_.param("polar_grid/cell/size",
		polar_grid_cell_size_, polar_grid_cell_size_);
	pnh_.param("polar_grid/cell/max_height",
		polar_grid_cell_max_height_, polar_grid_cell_max_height_);
	pnh_.param("polar_grid/segments",
		polar_grid_segments_, polar_grid_segments_);
	polar_grid_bins_ = std::ceil(polar_grid_range_max_ / 
		polar_grid_cell_size_);

	// Define cartesian grid parameters
	pnh_.param("cart_grid/cell/size",
		cart_grid_cell_size_, cart_grid_cell_size_);
	cart_grid_height_ = int(polar_grid_range_max_ / cart_grid_cell_size_ / std::sqrt(2));
	cart_grid_width_ = cart_grid_height_ * 2;
	// polar_grid_bins = (params_.polar_grid_range_max * std::sqrt(2)) /
	// 	params_.polar_grid_cell_size + 1;

	// // Define semantic parameters
	// private_nh_.param("semantic/edge_detection/perform", params_.sem_ed,
	// 	params_.sem_ed);
	// private_nh_.param("semantic/edge_detection/min", params_.sem_ed_min,
	// 	params_.sem_ed_min);
	// private_nh_.param("semantic/edge_detection/max", params_.sem_ed_max,
	// 	params_.sem_ed_max);
	// private_nh_.param("semantic/edge_detection/kernel", params_.sem_ed_kernel,
	// 	params_.sem_ed_kernel);
	// Define ransac ground plane parameters
	pnh_.param("ransac/tolerance",
		ransac_tolerance_, ransac_tolerance_);
	pnh_.param("ransac/iterations",
		ransac_iterations_, ransac_iterations_);

	// // Define static conversion values

	// // Print parameters
	// // ROS_INFO_STREAM("scenario " << params_.scenario);
	// ROS_INFO_STREAM("lidar_height " << params_.lidar_height);
	// ROS_INFO_STREAM("lidar_z_min " << params_.lidar_z_min);
	// ROS_INFO_STREAM("grid_range_min " << params_.grid_range_min);
	// ROS_INFO_STREAM("grid_range_max " << params_.grid_range_max);
	// ROS_INFO_STREAM("grid_height " << params_.grid_height);
	// ROS_INFO_STREAM("grid_width " << params_.grid_width);
	// ROS_INFO_STREAM("grid_cell_size " << params_.grid_cell_size);
	// ROS_INFO_STREAM("grid_cell_height " << params_.grid_cell_height);
	// ROS_INFO_STREAM("grid_bins " << params_.grid_bins);
	// ROS_INFO_STREAM("grid_segments " << params_.grid_segments);
	// ROS_INFO_STREAM("ransac_tolerance " << params_.ransac_tolerance);
	// ROS_INFO_STREAM("ransac_iterations " << params_.ransac_iterations);
	// ROS_INFO_STREAM("inv_angular_res " << params_.inv_angular_res);
	// ROS_INFO_STREAM("inv_radial_res " << params_.inv_radial_res);

	// // Define polar grid
	// polar_grid_ = std::vector< std::vector<PolarCell> >(params_.grid_segments,
	// 	std::vector<PolarCell>(params_.grid_bins));

	// Define occupancy grid
	

	// // Define Publisher 
	// cloud_filtered_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/cloud/filtered", 2);
	// cloud_ground_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/cloud/ground", 2);
	// cloud_elevated_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/cloud/elevated", 2);
	// voxel_ground_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/voxel/ground", 2);
	// voxel_elevated_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/voxel/elevated", 2);
	// grid_occupancy_pub_ = nh_.advertise<OccupancyGrid>(
	// 	"/sensor/grid/occupancy", 2);

	// // image_semantic_pub_ = nh_.advertise<Image>(
	// // 	"/sensor/image/semantic", 2);
	// cloud_semantic_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/cloud/semantic", 2);
	// cloud_semantic_sparse_pub_ = nh_.advertise<PointCloud2>(
	// 	"/sensor/cloud/semantic_sparse", 2);
	// image_detection_grid_pub_ = nh_.advertise<Image>(
	// 	"/sensor/image/detection_grid", 2);
	// image_bev_semantic_grid_pub_ = nh_.advertise<Image>(
	// 	"/sensor/image/bev_semantic_grid", 2);


	// initOccupancyGrid();

	// Define Subscriber
	sub_pointcloud_ = nh_.subscribe(
		"kitti/completed_pointcloud", 2, &GroundExtraction::callback, this);
	pub_pointcloud_ground_inliers_ = nh_.advertise<PointCloud2>(
		"/sensors/pointcloud/ground_inliers", 2);
	pub_pointcloud_ground_outliers_ = nh_.advertise<PointCloud2>(
		"/sensors/pointcloud/ground_outliers", 2);
	pub_pointcloud_ground_ = nh_.advertise<PointCloud2>(
		"/sensors/pointcloud/ground", 2);
	pub_pointcloud_elevated_ = nh_.advertise<PointCloud2>(
		"/sensors/pointcloud/elevated", 2);
	pub_voxel_ground_ = nh_.advertise<PointCloud2>(
		"/sensors/voxel/ground", 2);
	pub_voxel_elevated_ = nh_.advertise<PointCloud2>(
		"/sensors/voxel/elevated", 2);
	pub_occupancy_grid_ = nh_.advertise<OccupancyGrid>(
		"/sensors/occupancy_grid", 2);

	// Init counter for publishing
	time_frame_ = 0;
}

void GroundExtraction::initOccupancyGrid(){

	occ_grid_ = boost::make_shared<OccupancyGrid>();
	occ_grid_->data.resize(cart_grid_width_ * cart_grid_height_);
	occ_grid_->info.width = uint32_t(cart_grid_width_);
	occ_grid_->info.height = uint32_t(cart_grid_height_);
	occ_grid_->info.resolution = float(cart_grid_cell_size_);
	occ_grid_->info.origin.position.x = polar_grid_range_max_;
	occ_grid_->info.origin.position.y = polar_grid_range_max_;
	occ_grid_->info.origin.position.z = sensor_frame_z_;
	occ_grid_->info.origin.orientation.w = 0;
	occ_grid_->info.origin.orientation.x = 0.707;
	occ_grid_->info.origin.orientation.y = -0.707;
	occ_grid_->info.origin.orientation.z = 0;

	// Init occupancy grid
	for(int j = 0; j < cart_grid_height_; ++j){
		for(int i = 0; i < cart_grid_width_; ++i){

			// Never reach this cells because of opening angle
			if(i < j || i >= cart_grid_width_ - j){
				occ_grid_->data[j * cart_grid_width_ + i] = -1;
			}			
		}
	}
}

void GroundExtraction::callback(
	const PointCloud2ConstPtr & msg_pointcloud)
{

	if(true){
		semanticBasedGroundExtraction(msg_pointcloud);
	}
	else{
		geometricBasedGroundExtraction(msg_pointcloud);
	}

}

void GroundExtraction::semanticBasedGroundExtraction(
	const PointCloud2ConstPtr & msg_pointcloud)
{

	// Convert input cloud
	VRGBPointCloud::Ptr pcl_cloud(new VRGBPointCloud);
	pcl::fromROSMsg(*msg_pointcloud, *pcl_cloud);

	pcl::PointIndices::Ptr pcl_outliers(new pcl::PointIndices());
	pcl::ExtractIndices<VRGBPoint> pcl_extractor;


	// Loop through input point cloud
	for(int i = 0; i < pcl_cloud->size(); ++i){

		// Read current point
		VRGBPoint & point = pcl_cloud->at(i);

		if(isSidewalkOrRoad(point)){
			pcl_outliers->indices.push_back(i);
		}
	}

	pcl_extractor.setInputCloud(pcl_cloud);
	pcl_extractor.setIndices(pcl_outliers);
	pcl_extractor.setNegative(true);
	pcl_extractor.filter(*pcl_cloud);

	PointCloud2 msg_pointcloud_elevated;
	pcl::toROSMsg(*pcl_cloud, msg_pointcloud_elevated);
	msg_pointcloud_elevated.header.stamp.nsec =
		msg_pointcloud->header.stamp.nsec;
	pub_pointcloud_elevated_.publish(msg_pointcloud_elevated);
}

void GroundExtraction::geometricBasedGroundExtraction(
	const PointCloud2ConstPtr & msg_pointcloud)
{
	
	// Convert input cloud
	VRGBPointCloud::Ptr pcl_cloud(new VRGBPointCloud);
	pcl::fromROSMsg(*msg_pointcloud, *pcl_cloud);

	pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());
	pcl::ExtractIndices<VRGBPoint> pcl_extractor;

	ROS_INFO("Converted");

	// Reset polar grid
	polar_grid_ = std::vector< std::vector<PolarCell> >(
		polar_grid_segments_, std::vector<PolarCell>(polar_grid_bins_));

	// Loop through input point cloud
	for(int i = 0; i < pcl_cloud->size(); ++i){

		// Read current point
		VRGBPoint & point = pcl_cloud->at(i);

		// Determine range of lidar point and check
		float range = std::sqrt(point.x * point.x + point.y * point.y);
		if(range > polar_grid_range_min_ &&
		   range < polar_grid_range_max_){

		   	// Add index for filtered point cloud
			pcl_inliers->indices.push_back(i);

			// Buffer variables
			int seg, bin;

			// Get polar grid cell indices
			fromVeloCoordsToPolarCell(point.x, point.y, seg, bin);

			// Grab cell
			PolarCell & cell = polar_grid_[seg][bin];

			// Update min max
			if(cell.count == 0){
				cell.min_point = point;
				cell.max_point = point;
			}
			else{
				if(point.z < cell.min_point.z){
					cell.min_point = point;
				}
				if(point.z > cell.max_point.z){
					cell.max_point = point;
				}
			}
			// Increase count
			cell.count++;
		}
	}

	// Extract points from original point cloud
	pcl_extractor.setInputCloud(pcl_cloud);
	pcl_extractor.setIndices(pcl_inliers);
	pcl_extractor.setNegative(false);
	pcl_extractor.filter(*pcl_cloud);


	VRGBPointCloud::Ptr pcl_ground_plane(new VRGBPointCloud);
	// Loop over polar grid
	for(int i = 0; i < polar_grid_segments_; ++i){
		for(int j = 0; j < polar_grid_bins_; ++j){

			// Grab cell
			PolarCell & cell = polar_grid_[i][j];

			// Check if cell can be ground cell
			if(cell.count > 0 &&
			  (cell.max_point.z - cell.min_point.z < polar_grid_cell_max_height_)
			  && isSidewalkOrRoad(cell.min_point)
			){

				// Push back cell attributes to ground plane cloud
				pcl_ground_plane->points.push_back(cell.min_point);
				// std::cout << cell.min_point << std::endl;
			}
		}
	}

	// Estimate the ground plane using PCL and RANSAC
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<VRGBPoint> segmentation;
	segmentation.setOptimizeCoefficients(true);
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(ransac_tolerance_);
	segmentation.setMaxIterations(ransac_iterations_);
	segmentation.setInputCloud(pcl_ground_plane->makeShared());
	segmentation.segment(*inliers, *coefficients);

	// Divide ground plane cloud in inlier cloud and outlier cloud
	pcl_extractor.setInputCloud(pcl_ground_plane);
	pcl_extractor.setIndices(inliers);
	pcl_extractor.setNegative(false);
	VRGBPointCloud::Ptr pcl_ground_plane_inliers(new VRGBPointCloud);
	pcl_extractor.filter(*pcl_ground_plane_inliers);

	pcl_extractor.setInputCloud(pcl_ground_plane);
	pcl_extractor.setIndices(inliers);
	pcl_extractor.setNegative(true);
	VRGBPointCloud::Ptr pcl_ground_plane_outliers(new VRGBPointCloud);
	pcl_extractor.filter(*pcl_ground_plane_outliers);

	// Sanity check
	if(inliers->indices.empty() || coefficients->values[3] > 2 || 
		coefficients->values[3] < 1.5){
		ROS_WARN("Bad ground plane estimation! # Ransac Inliers [%d] # Lidar "
			"height [%f]", int(inliers->indices.size()),
			coefficients->values[3]);
	}

	// Publish ground plane inliers and outliers point cloud
	pcl_ground_plane_inliers->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_ground_plane_inliers->header.stamp = 
		pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_pointcloud_ground_inliers_.publish(pcl_ground_plane_inliers);

	pcl_ground_plane_outliers->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_ground_plane_outliers->header.stamp = 
		pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_pointcloud_ground_outliers_.publish(pcl_ground_plane_outliers);

	// Print
	ROS_INFO("Ground plane estimation [%d] # Points [%d] # Inliers [%d] "
		" C [%f][%f][%f][%f]",	time_frame_, 
		int(pcl_ground_plane->size()),	int(pcl_ground_plane_inliers->size()), 
		coefficients->values[0], coefficients->values[1],
		coefficients->values[2], coefficients->values[3]);

	// Loop over segments
	for(int s = 0; s < polar_grid_segments_; s++){

		// Set hit to false
		bool hit = false;

		// Loop over bins
		for(int b = 0; b < polar_grid_bins_; b++){
		
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
				cell.height = polar_grid_[s][b].max_point.z - cell.ground;

				// If cell height big enough fill cell as occupied
				if(cell.height > polar_grid_cell_max_height_){

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
	VRGBPointCloud::Ptr pcl_ground(new VRGBPointCloud);
	VRGBPointCloud::Ptr pcl_elevated(new VRGBPointCloud);

	for(int i = 0; i < pcl_cloud->size(); ++i){

		// Read current point
		VRGBPoint point = pcl_cloud->at(i);

		// Buffer variables
		int seg, bin;

		// Get polar grid cell indices
		fromVeloCoordsToPolarCell(point.x, point.y, seg, bin);

		// Grab cell
		PolarCell & cell = polar_grid_[seg][bin];

		if(point.z > cell.ground && cell.height > polar_grid_cell_max_height_){
			pcl_elevated->points.push_back(point);
		}
		else{
			pcl_ground->points.push_back(point);
		}
	}

	// Publish ground cloud
	pcl_ground->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_ground->header.stamp = pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_pointcloud_ground_.publish(pcl_ground);

	// Publish elevated cloud
	pcl_elevated->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_elevated->header.stamp = pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_pointcloud_elevated_.publish(pcl_elevated);	

	
	// Clear voxel pcls
	VRGBPointCloud::Ptr pcl_voxel_elevated(new VRGBPointCloud);
	VRGBPointCloud::Ptr pcl_voxel_ground(new VRGBPointCloud);

	// Init detection image and fill free space grid cells
	cv::Mat detection_grid = cv::Mat(cart_grid_height_, cart_grid_width_, CV_32FC3,
		cv::Scalar(-100.0, 0.0, 0.0));
	// Init detection image and fill free space grid cells
	// bev_semantic_grid_ = cv::Mat(cart_grid_height_, cart_grid_width_, CV_8UC3,
	// 	cv::Scalar(128, 128, 128));

	// Go through cartesian grid
	float x = polar_grid_range_max_ - cart_grid_cell_size_ / 2;
	for(int j = 0; j < cart_grid_height_; ++j, x -= cart_grid_cell_size_){
		float y = x;
		for(int i = j; i < cart_grid_width_ - j; ++i, y -= cart_grid_cell_size_){

			// Buffer variables
			int seg, bin;
			float rx, ry;

			// Get polar grid cell indices
			// ROS_INFO("%f %f", x, y);
			fromVeloCoordsToPolarCell(x, y, seg, bin);
			fromPolarCellToVeloCoords(seg, bin, rx, ry);
			float dx = std::sqrt( (x-rx)*(x-rx) + (y-ry)*(y-ry) );
			// if(dx / 2 * std::sqrt(2) > cart_grid_cell_size_ )
				
			ROS_WARN("%f %f %d %d %f %f", x, y, seg, bin, rx, ry);
			// ROS_INFO("%d %d", seg, bin);
			
			// Grab polar cell
			PolarCell & cell = polar_grid_[seg][bin];

			// Fill ground voxel cloud
			VRGBPoint ground_point;
			ground_point.x = x;
			ground_point.y = y;
			ground_point.z = cell.ground;
			pcl_voxel_ground->points.push_back(ground_point);

			// Calculate occupancy grid cell index
			int cell_index = j * cart_grid_width_ + i;
			// ROS_INFO("%d", cell_index);
			// ROS_INFO("%d %d", i, j);

			// If cell is free
			if(cell.idx == PolarCell::FREE){
				// occ_grid_->data[cell_index] = 0;
				// detection_grid.at<cv::Vec3f>(j, i)[0] = -50.0;
				// bev_semantic_grid_.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 255, 255);
			}
			// If cell is unknown
			// else if(cell.idx == PolarCell::UNKNOWN)
			// 	occ_grid_->data[cell_index] = 50;

			// If cell is occupied
			else{
				// occ_grid_->data[cell_index] = 100;
				if(cell.count == 0){
					ROS_WARN("OCCUPIED BUT NO CELL DATA");
				}
				// Fill elevated voxel cloud
				// ROS_INFO("%f", cell.max_point.z);
				// for(float v = cell.ground; v < cell.max_point.z; v += cart_grid_cell_size_){
				// 	VRGBPoint voxel;
				// 	voxel.x = x;
				// 	voxel.x = y;
				// 	voxel.z = v;
				// 	pcl_voxel_elevated->points.push_back(voxel);

				// }
			}
		}
	}
	
	// Publish voxel ground
	pcl_voxel_ground->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_voxel_ground->header.stamp = 
		pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_voxel_ground_.publish(pcl_voxel_ground);

	// Publish voxel elevated
	pcl_voxel_elevated->header.frame_id = msg_pointcloud->header.frame_id;
	pcl_voxel_elevated->header.stamp = 
		pcl_conversions::toPCL(msg_pointcloud->header.stamp);
	pub_voxel_elevated_.publish(pcl_voxel_elevated);
	/*
	// Publish occupancy grid
	occ_grid_->header.stamp = msg_pointcloud->header.stamp;
	occ_grid_->header.frame_id = msg_pointcloud->header.frame_id;
	occ_grid_->info.map_load_time = occ_grid_->header.stamp;
	pub_occupancy_grid_.publish(occ_grid_);
	*/

	// Increment time frame
	time_frame_++;
}

// // void GroundExtraction::processImage(const Image::ConstPtr & image){

// /******************************************************************************
//  * 1. Load precalculated semantic segmentated images to ensure online
//  * performance
//  */
// 	// Canny edge detection
// 	/*
// 	cv::Mat sem_edge_img, sem_dil_img, sem_output;
// 	if(params_.sem_ed){
// 		cv::Canny(sem_image_, sem_edge_img, params_.sem_ed_min,
// 			params_.sem_ed_max, params_.sem_ed_kernel);
// 		cv::dilate(sem_edge_img, sem_dil_img, cv::Mat(), 
// 			cv::Point(-1, -1), 1, 1, 1);
// 		sem_image_.copyTo(sem_output, sem_dil_img);
// 	}
// 	*/

// // 	// Publish
// // 	cv_bridge::CvImage cv_semantic_image;
// // 	cv_semantic_image.image = sem_image_;
// // 	cv_semantic_image.encoding = "bgr8";
// // 	cv_semantic_image.header.stamp = image->header.stamp;
// // 	image_semantic_pub_.publish(cv_semantic_image.toImageMsg());
// // }

// /******************************************************************************
//  * 1. Convert velodyne points into image space
//  */

// 	// Define matrix to write velodyne points into
// 	MatrixXf matrix_velodyne_points = MatrixXf::Zero(4, cloud->size());
// 	for(int i = 0; i < cloud->size(); ++i){
// 		matrix_velodyne_points(0,i) = cloud->points[i].x;
// 		matrix_velodyne_points(1,i) = cloud->points[i].y;
// 		matrix_velodyne_points(2,i) = cloud->points[i].z;
// 		matrix_velodyne_points(3,i) = 1;
// 	}

// 	// Project the matrix from velodyne coordinates to the image plane
// 	MatrixXf matrix_image_points = 
// 		tools_.transformVeloToImage(matrix_velodyne_points);

// 	// Get image format
// 	const cv::Mat_<uint8_t> cv_image = cv_bridge::toCvShare(sem_image, image_encodings::BGR8)->image;
// 	int img_width = cv_image.cols;
// 	int img_height = cv_image.rows;

// 	// Clear semantic cloud
// 	pcl_semantic_->points.clear();

// 	// Loop over image points
// 	for(int i = 0; i < matrix_image_points.cols(); i++){

// 		// Check if image point is valid
// 		const int & img_x = matrix_image_points(0, i);
// 		const int & img_y = matrix_image_points(1, i);
// 		const int & img_z = matrix_image_points(2, i);

// 		// TODO change to method
// 		if( (img_x >= 0 && img_x < img_width) &&
// 			(img_y >= 0 && img_y < img_height) &&
// 			(img_z >= 0)){

// 			// Get R G B values of semantic image
// 			uint8_t r = cv_image.at<cv::Vec3b>(img_y,img_x)[2];
// 			uint8_t g = cv_image.at<cv::Vec3b>(img_y,img_x)[1];
// 			uint8_t b = cv_image.at<cv::Vec3b>(img_y,img_x)[0];

// 			// Create new point and fill it
// 			VRGBPoint point;
// 			point.x = cloud->points[i].x;
// 			point.y = cloud->points[i].y;
// 			point.z = cloud->points[i].z;
// 			point.r = r;
// 			point.g = g;
// 			point.b = b;

// 			// Push back point
// 			pcl_semantic_->points.push_back(point);
// 		}
// 	}

// 	// Sanity check
// 	if(pcl_semantic_->empty()){
// 		ROS_WARN("Empty semantic point cloud!");
// 		return;
// 	}

// 	// Publish semantic cloud
// 	pcl_semantic_->header.frame_id = cloud->header.frame_id;
// 	pcl_semantic_->header.stamp = cloud->header.stamp;
// 	cloud_semantic_pub_.publish(pcl_semantic_);

// /******************************************************************************
//  * 2. Gather in each cartesian grid cell the semantic labels
//  */	

// void GroundExtraction::produceDetectionGrid(
// 	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pcl_semantic)
// {

// 	// Define hash table to remember points of semantic point cloud in each cell
// 	std::map<int, std::map<int,int> > cell_hash_table;

// 	// Loop through semantic point cloud
// 	for(int i = 0; i < pcl_semantic->size(); ++i){

// 		// Read current point
// 		VRGBPoint & point = pcl_semantic_->at(i);

// 		// Buffer variables
// 		int grid_x, grid_y;

// 		// Get cartesian grid indices
// 		fromVeloCoordsToCartesianCell(point.x, point.y, grid_x, grid_y);
// 		int grid_occ = grid_y * params_.grid_width + grid_x;

// 		// Get semantic class
// 		int semantic_class = tools_.SEMANTIC_COLOR_TO_CLASS[
// 			point.r + point.g + point.b];

// 		// Increment hash table counter for this grid cell
// 		cell_hash_table[grid_occ][semantic_class]++;
// 	}

// /******************************************************************************
//  * 3. Fill detection grid image and sparse semantic point cloud
//  */	

// 	// Init  sparse semantic point cloud
// 	pcl_sparse_semantic_->points.clear();

// 	// Loop over hash table to find most dominant semantic label
// 	std::map<int, std::map<int,int> >::iterator it;
// 	for(it = cell_hash_table.begin(); it != cell_hash_table.end(); it++ ){

// 		// Loop through all hits in cell and determine best semantic label
// 		std::map<int,int>::iterator it2;
// 		int max_semantic_counter = -1;
// 		int max_class;
// 		for(it2 = it->second.begin(); it2 != it->second.end(); it2++ ){

// 			if(it2->second > max_semantic_counter){
// 				max_semantic_counter = it2->second;
// 				max_class = it2->first;
// 			}
// 		}

// 		// Determine cartesian grid indices
// 		int grid_x = it->first % params_.grid_width;
// 		int grid_y = it->first / params_.grid_width;

// 		// Buffer variables
// 		float x, y;
// 		int seg, bin;

// 		// Calculate velodyne coordinates
// 		fromCartesianCellToVeloCoords(grid_x, grid_y, x, y);

// 		// Calculate polar grid indices to grab polar cell
// 		fromVeloCoordsToPolarCell(x, y, seg, bin);
// 		PolarCell & cell = polar_grid_[seg][bin];

// 		// Write point to sparse point cloud
// 		VRGBPoint point;
// 		point.x = x;
// 		point.y = y;
// 		point.z = cell.ground;
// 		point.r = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,0);
// 		point.g = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,1);
// 		point.b = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,2);
// 		pcl_sparse_semantic_->points.push_back(point);

// 		// Fill detection grid with semantic class
// 		detection_grid_.at<cv::Vec3f>(grid_y, grid_x)[0] = max_class;
// 		detection_grid_.at<cv::Vec3f>(grid_y, grid_x)[1] = cell.ground;
// 		detection_grid_.at<cv::Vec3f>(grid_y, grid_x)[2] = cell.ground + cell.height;
// 		// BGR assignment
// 		bev_semantic_grid_.at<cv::Vec3b>(grid_y, grid_x)[0] = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,2);
// 		bev_semantic_grid_.at<cv::Vec3b>(grid_y, grid_x)[1] = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,1);
// 		bev_semantic_grid_.at<cv::Vec3b>(grid_y, grid_x)[2] = tools_.SEMANTIC_CLASS_TO_COLOR(max_class,0);
// 	}

// 	// Publish sparse semantic cloud
// 	pcl_sparse_semantic_->header.frame_id = cloud->header.frame_id;
// 	pcl_sparse_semantic_->header.stamp = cloud->header.stamp;
// 	cloud_semantic_sparse_pub_.publish(pcl_sparse_semantic_);

// 	// Publish detection grid
// 	cv_bridge::CvImage cv_detection_grid_image;
// 	cv_detection_grid_image.image = detection_grid_;
// 	cv_detection_grid_image.encoding = image_encodings::TYPE_32FC3;
// 	cv_detection_grid_image.header.stamp = sem_image->header.stamp;
// 	image_detection_grid_pub_.publish(cv_detection_grid_image.toImageMsg());
// 	cv_bridge::CvImage cv_bev_semantic_grid_image;
// 	cv_bev_semantic_grid_image.image = bev_semantic_grid_;
// 	cv_bev_semantic_grid_image.encoding = image_encodings::TYPE_8UC3;
// 	cv_bev_semantic_grid_image.header.stamp = sem_image->header.stamp;
// 	image_bev_semantic_grid_pub_.publish(cv_bev_semantic_grid_image.toImageMsg());
// }

void GroundExtraction::fromVeloCoordsToPolarCell(
	const float x, const float y, int & seg, int & bin){

	float mag = std::sqrt(x * x + y * y);
	float ang = -std::atan2(y, x);
	seg = int((ang + sensor_frame_opening_angle_) * (2 * polar_grid_segments_/ M_PI));
	bin = int(mag / polar_grid_cell_size_);

	// For last segment
	if(x == -y)
		seg = polar_grid_segments_ - 1;
}

void GroundExtraction::fromPolarCellToVeloCoords(
	const int seg, const int bin, float & x, float & y){

	float mag = bin * polar_grid_cell_size_;
	float ang = seg * M_PI / 2 / polar_grid_segments_ - sensor_frame_opening_angle_;
	y = - std::sin(ang) * mag;
	x = std::cos(ang) * mag;
}

void GroundExtraction::fromVeloCoordsToCartesianCell(const float x, const float y,
		int & grid_x, int & grid_y){

	grid_y = cart_grid_height_ - x / cart_grid_cell_size_;
	grid_x = - y / cart_grid_cell_size_ + cart_grid_height_;
}

void GroundExtraction::fromCartesianCellToVeloCoords(const int grid_x,
	const int grid_y, float & x, float & y){

	x = (cart_grid_height_ - grid_y) * cart_grid_cell_size_ - 
		cart_grid_cell_size_ / 2;
	y = (cart_grid_height_ - grid_x) * cart_grid_cell_size_ -
		cart_grid_cell_size_ / 2;
}

bool GroundExtraction::isSidewalkOrRoad(const VRGBPoint & point)
{
	return point.g == 64 || point.g == 35;
}

} // namespace sensors


/*
if(true){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_(
			new pcl::PointCloud<pcl::PointXYZRGB>);
		fromROSMsg(*msg_pointcloud_velo, *pcl_velo);
		pcl_extractor.setInputCloud(pcl_ground_plane_);
		pcl_extractor.setIndices(inliers);
		pcl_extractor.setNegative(false);
		pcl_extractor.filter(*pcl_ground_plane_inliers_);
	}
	else{
		// Has to be in camera frame
		// PointCloud2Iterator<float> iter_x(&(*msg_pointcloud), "x");
	 	// PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
	 	// PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");
		PointCloud2ConstIterator<float> it(*msg_pointcloud, "x");
		ROS_INFO("%s", msg_pointcloud->header.frame_id.c_str());
		int i = 0;
	 	// Buffer variables
		int seg, bin;
		
		cv::Mat polar_image_min = cv::Mat(
			polar_grid_bins_, polar_grid_segments_, CV_32FC1, cv::Scalar(100));
		cv::Mat polar_image_max = cv::Mat(
			polar_grid_bins_, polar_grid_segments_, CV_32FC1, cv::Scalar(-100));

		for (; it != it.end(); ++it) {
	  		float x = it[0];
	  		float y = it[1];
	  		float z = it[2];
	  		float range = std::sqrt(x * x + y * y);
	  		if(range > polar_grid_range_min_ &&
			   range < polar_grid_range_max_)
	  		{
				fromSensorCoordinatesToPolarCell(x, y, seg, bin);
	    		std::cout << it[0] << ", " << it[1] << ", " << it[2] << 
	    		  "->" << range << " " << seg << " " << bin << '\n';
				std::cout << z << " " << polar_image_min.at<float>(bin, seg) << 
					" " << std::min(z, polar_image_min.at<float>(bin, seg))	 << std::endl;
				polar_image_min.at<float>(bin, seg) =
					std::min(z, polar_image_min.at<float>(bin, seg));
				polar_image_max.at<float>(bin, seg) =
					std::max(z, polar_image_max.at<float>(bin, seg));
	  		}
	    }
	    std::cout << polar_image_min << std::endl;

	    for(int s=0; s<polar_grid_segments_; s++)
	    {
	    	for(int b=0; b<polar_grid_bins_; b++)
	    	{
	    	
	    	}
	    }
	    std::cout << polar_image_max << std::endl;
*/