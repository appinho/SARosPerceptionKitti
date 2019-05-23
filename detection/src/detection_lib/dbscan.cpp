/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <detection_lib/dbscan.h>

namespace detection{

/******************************************************************************/

DbScan::DbScan(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	// Get parameter
	private_nh_.param("grid/range/max", params_.grid_range_max,
		params_.grid_range_max);
	private_nh_.param("grid/cell/size", params_.grid_cell_size,
		params_.grid_cell_size);
	private_nh_.param("pedestrian/spawn/side/min", params_.ped_spawn.side_min,
		params_.ped_spawn.side_min);
	private_nh_.param("pedestrian/spawn/side/max", params_.ped_spawn.side_max,
		params_.ped_spawn.side_max);
	private_nh_.param("pedestrian/spawn/height/min", params_.ped_spawn.height_min,
		params_.ped_spawn.height_min);
	private_nh_.param("pedestrian/spawn/height/max", params_.ped_spawn.height_max,
		params_.ped_spawn.height_max);
	private_nh_.param("pedestrian/spawn/semantic/min", params_.ped_spawn.semantic_min,
		params_.ped_spawn.semantic_min);
	private_nh_.param("car/spawn/side/min", params_.car_spawn.side_min,
		params_.car_spawn.side_min);
	private_nh_.param("car/spawn/side/max", params_.car_spawn.side_max,
		params_.car_spawn.side_max);
	private_nh_.param("car/spawn/height/min", params_.car_spawn.height_min,
		params_.car_spawn.height_min);
	private_nh_.param("car/spawn/height/max", params_.car_spawn.height_max,
		params_.car_spawn.height_max);
	private_nh_.param("car/spawn/semantic/min", params_.car_spawn.semantic_min,
		params_.car_spawn.semantic_min);
	private_nh_.param("pedestrian/update/side/min", params_.ped_update.side_min,
		params_.ped_update.side_min);
	private_nh_.param("pedestrian/update/side/max", params_.ped_update.side_max,
		params_.ped_update.side_max);
	private_nh_.param("pedestrian/update/height/min", params_.ped_update.height_min,
		params_.ped_update.height_min);
	private_nh_.param("pedestrian/update/height/max", params_.ped_update.height_max,
		params_.ped_update.height_max);
	private_nh_.param("pedestrian/update/semantic/min", params_.ped_update.semantic_min,
		params_.ped_update.semantic_min);
	private_nh_.param("car/update/side/min", params_.car_update.side_min,
		params_.car_update.side_min);
	private_nh_.param("car/update/side/max", params_.car_update.side_max,
		params_.car_update.side_max);
	private_nh_.param("car/update/height/min", params_.car_update.height_min,
		params_.car_update.height_min);
	private_nh_.param("car/update/height/max", params_.car_update.height_max,
		params_.car_update.height_max);
	private_nh_.param("car/update/semantic/min", params_.car_update.semantic_min,
		params_.car_update.semantic_min);

	// Print parameters
	ROS_INFO_STREAM("ped_spawn.side_min " << params_.ped_spawn.side_min);
	ROS_INFO_STREAM("ped_spawn.side_max " << params_.ped_spawn.side_max);
	ROS_INFO_STREAM("ped_spawn.height_min " << params_.ped_spawn.height_min);
	ROS_INFO_STREAM("ped_spawn.height_max " << params_.ped_spawn.height_max);
	ROS_INFO_STREAM("ped_spawn.semantic_min " << params_.ped_spawn.semantic_min);
	ROS_INFO_STREAM("car_spawn.side_min " << params_.car_spawn.side_min);
	ROS_INFO_STREAM("car_spawn.side_max " << params_.car_spawn.side_max);
	ROS_INFO_STREAM("car_spawn.height_min " << params_.car_spawn.height_min);
	ROS_INFO_STREAM("car_spawn.height_max " << params_.car_spawn.height_max);
	ROS_INFO_STREAM("car_spawn.semantic_min " << params_.car_spawn.semantic_min);
	ROS_INFO_STREAM("ped_update.side_min " << params_.ped_update.side_min);
	ROS_INFO_STREAM("ped_update.side_max " << params_.ped_update.side_max);
	ROS_INFO_STREAM("ped_update.height_min " << params_.ped_update.height_min);
	ROS_INFO_STREAM("ped_update.height_max " << params_.ped_update.height_max);
	ROS_INFO_STREAM("ped_update.semantic_min " << params_.ped_update.semantic_min);
	ROS_INFO_STREAM("car_update.side_min " << params_.car_update.side_min);
	ROS_INFO_STREAM("car_update.side_max " << params_.car_update.side_max);
	ROS_INFO_STREAM("car_update.height_min " << params_.car_update.height_min);
	ROS_INFO_STREAM("car_update.height_max " << params_.car_update.height_max);
	ROS_INFO_STREAM("car_update.semantic_min " << params_.car_update.semantic_min);

	// Init counter for publishing
	time_frame_ = 0;

	// Define Subscriber
	image_detection_grid_sub_ = nh.subscribe(
		"/sensor/image/detection_grid", 2, &DbScan::process, this);

	// Define Publisher
	object_array_pub_ = nh_.advertise<ObjectArray>(
		"/detection/objects", 2);
}

DbScan::~DbScan(){

}

void DbScan::process(const Image::ConstPtr & image_detection_grid){

	// Convert image detection grid to cv mat detection grid
	cv_bridge::CvImagePtr cv_det_grid_ptr;
	try{
		cv_det_grid_ptr = cv_bridge::toCvCopy(image_detection_grid, 
			image_encodings::TYPE_32FC3);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Run DbScan algorithm
	cv::Mat grid = cv_det_grid_ptr->image.clone(); 
	runDbScan(grid);

	// Determine cluster information
	filterClusters(cv_det_grid_ptr->image);

	// Publish object list
	fillObjectList();
	object_array_.header = image_detection_grid->header;
	object_array_pub_.publish(object_array_);

	// Print cluster info
	for(int i = 0; i < number_of_clusters_; ++i){
		printCluster(clusters_[i]);
	}
	// Print object info
	for(int i = 0; i < object_array_.list.size(); ++i){
		printObject(object_array_.list[i]);
	}
	// Print sensor fusion
	ROS_INFO("Publishing Detection [%d]: # Clusters[%d]", time_frame_,
		number_of_clusters_);

	// Increment time frame
	time_frame_++;
}

void DbScan::runDbScan(cv::Mat grid){

	// Clear previous Clusters
	clusters_.clear();

	// Loop through image
	for(int y = 0; y < grid.rows; y++){
		for(int x = y; y < grid.cols - x; x++){

			// Get semantic
			int semantic_class = grid.at<cv::Vec3f>(y,x)[0];

			// If not valid semantic continue
			if(!isKittiValidSemantic(semantic_class)){
				continue;
			}
				
			// Flag cell as visited
			grid.at<cv::Vec3f>(y,x)[0] = -100.0;

			// New cluster
			Cluster c = Cluster();
			c.kernel = tools_.getClusterKernel(semantic_class);

			// Init neighbor queue and add cell
			std::queue<cv::Point> neighbor_queue;
			neighbor_queue.push(cv::Point(x,y));

			// Init non neighbor list
			std::vector<cv::Point> non_neighbor_list;

			// Search for neighbor cells with same semantic until queue is empty
			while(!neighbor_queue.empty()){

				// Store cell
				c.geometric.cells.push_back(neighbor_queue.front());
				int c_x = neighbor_queue.front().x;
				int c_y = neighbor_queue.front().y;
				neighbor_queue.pop();

				// Look for equal semantic information within kernel
				for(int k = -c.kernel; k <= c.kernel; ++k){
					for(int l = -c.kernel; l <= c.kernel; ++l){

						// Dont check the center cell itself
						if(k == 0 && l == 0)
							continue;

						// Calculate neighbor cell indices
						int n_x = c_x + k;
						int n_y = c_y + l;

						// Check if neighbor cell is out of bounce
						if(n_x >= 0 && n_x < grid.cols &&
							n_y >= 0 && n_y < grid.rows){

							// Get semantic of neighbor cell
							int n_semantic_class = 
								grid.at<cv::Vec3f>(n_y,n_x)[0];

							// If this semantic matches with cluster semantic
							if(n_semantic_class == semantic_class){

								// Flag neighbor cell as visited
								grid.at<cv::Vec3f>(n_y,n_x)[0] -= 20;

								// Add neighbor cell to neighbor queue
								neighbor_queue.push(cv::Point(n_x,n_y));

								// Check free space attachment
								if(!c.has_adjacent_free_space){
									// If no free space cell next to it continue
									int fs_kernel = 1;
									for(int k = -fs_kernel; k <= fs_kernel; ++k){
										for(int l = -fs_kernel; l <= fs_kernel; ++l){
											if(grid.at<cv::Vec3f>(n_y + k,n_x + l)[0] == -50){
												c.has_adjacent_free_space = true;
												break;
											}
										}
									}
								}

							}
							// If non aimed semantic has hit
							else if(!isKittiValidSemantic(n_semantic_class) &&
								n_semantic_class >= 0){

								// Flag cell temporarily as visited
								grid.at<cv::Vec3f>(n_y,n_x)[0] -= 20;
								non_neighbor_list.push_back(cv::Point(n_x,n_y));
								c.semantic.diff_counter++;
							}
						}
					}
				}
			}

			// Write non neighbor list back to unvisited
			for(int nn = 0; nn < non_neighbor_list.size(); ++nn){

				int n_x = non_neighbor_list[nn].x;
				int n_y = non_neighbor_list[nn].y;
				grid.at<cv::Vec3f>(n_y,n_x)[0] += 20;
			}

			// Add semantic information
			c.semantic.id = semantic_class;
			c.geometric.num_cells = c.geometric.cells.size();
			c.semantic.confidence = float(c.geometric.num_cells) / 
				(c.geometric.num_cells + c.semantic.diff_counter);
			c.semantic.name = tools_.SEMANTIC_NAMES[c.semantic.id];

			// Push back cluster
			clusters_.push_back(c);
		}
	}

	// Determine number of clusters
	number_of_clusters_ = clusters_.size();
}

void DbScan::filterClusters(const cv::Mat grid){

	// Clear buffer
	object_array_.list.clear();

	// Loop through clusters
	for(int i = 0; i < clusters_.size(); i++){

		// Grab cluster
		Cluster & c = clusters_[i];

		// Fill id
		c.id = i;
		
		// Fit minimum areal rectangular around cluster cells
		cv::RotatedRect rect = cv::minAreaRect(cv::Mat(c.geometric.cells));

		// Get center point of bounding box/cube
		c.geometric.x = params_.grid_range_max -
			(rect.center.y * params_.grid_cell_size)
			- params_.grid_cell_size / 2;
		c.geometric.y = params_.grid_range_max -
			(rect.center.x * params_.grid_cell_size)
			- params_.grid_cell_size / 2;

		// Get width and length of cluster
		c.geometric.width = (rect.size.width + 1) * params_.grid_cell_size;
		c.geometric.length = (rect.size.height + 1) * params_.grid_cell_size;

		// Find minimum and maximum in z coordinates
		float min_low_z = grid.at<cv::Vec3f>(
			c.geometric.cells[0].y,c.geometric.cells[0].x)[1];
		float max_high_z = grid.at<cv::Vec3f>(
			c.geometric.cells[0].y,c.geometric.cells[0].x)[2];
		for(int j = 1; j < c.geometric.cells.size(); ++j){
			float low_z = grid.at<cv::Vec3f>(
				c.geometric.cells[j].y,c.geometric.cells[j].x)[1];
			float high_z = grid.at<cv::Vec3f>(
				c.geometric.cells[j].y,c.geometric.cells[j].x)[2];
			min_low_z = (min_low_z < low_z) ? min_low_z : low_z;
			max_high_z = (max_high_z > high_z) ? max_high_z : high_z;
		}

		// Get ground level and height of cluster
		c.geometric.z = min_low_z;
		c.geometric.height = max_high_z - min_low_z;

		// Get orientation of bounding box
		// Minus since opencv y,x is the opposite of the velodyne frame
		c.geometric.orientation = -rect.angle;

		// Store rect as back up
		c.rect = rect;

		// Get color BGR
		int r = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 0);
		int g = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 1);
		int b = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 2);
		c.color = cv::Scalar(r, g, b);

		// Determine if cluster can be a new track
		// Car
		if(c.semantic.id == 13){
			if(updateCar(c)){
				if(spawnCar(c)){
					c.is_new_track = true;
				}
				addObject(c);
			}
		}
		// Pedestrian
		else if(c.semantic.id == 11){
			if(updatePed(c)){
				if(spawnPed(c)){
					c.is_new_track = true;
				}
				addObject(c);
			}
		}
	}
}

void DbScan::fillObjectList(){

	// Transform objects in camera and world frame
	try{
		for(int i = 0; i < object_array_.list.size(); ++i){

			listener_.transformPoint("world",
				object_array_.list[i].velo_pose,
				object_array_.list[i].world_pose);

			listener_.transformPoint("camera_color_left",
				object_array_.list[i].velo_pose,
				object_array_.list[i].cam_pose);
		}
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("Received an exception trying to transform a point from"
			"\"velo_link\" to \"world\": %s", ex.what());
	}
}

void DbScan::addObject(const Cluster & c){

	// Create object
	Object object;
	object.id = c.id;

	// Pose in velo frame
	object.velo_pose.header.frame_id = "velo_link";
	object.velo_pose.point.x = c.geometric.x;
	object.velo_pose.point.y = c.geometric.y;
	object.velo_pose.point.z = c.geometric.z;

	// Geometry
	object.width = c.geometric.width;
	object.length = c.geometric.length;
	object.height = c.geometric.height;
	object.orientation = c.geometric.orientation;

	// Semantic
	object.semantic_id = c.semantic.id;
	object.semantic_confidence = c.semantic.confidence;
	object.semantic_name = c.semantic.name;

	// Color with BGR encoding
	object.r = c.color[0];
	object.g = c.color[1];
	object.b = c.color[2];
	if(c.is_new_track){
		object.a = 0.75f;
	}
	else{
		object.a = 0.35f;
	}

	// Tracking
	object.is_new_track = c.is_new_track;

	// Push back object to list
	object_array_.list.push_back(object);
}

bool DbScan::spawnPed(const Cluster & c){
	return (c.geometric.width > params_.ped_spawn.side_min ||
		c.geometric.length > params_.ped_spawn.side_min)
		&&
		(c.geometric.width < params_.ped_spawn.side_max &&
		c.geometric.length < params_.ped_spawn.side_max)
		&&
		(c.geometric.height > params_.ped_spawn.height_min && 
		c.geometric.height < params_.ped_spawn.height_max)
		&&
		(c.semantic.confidence > params_.ped_spawn.semantic_min)
		&&
		c.has_adjacent_free_space;
}

bool DbScan::spawnCar(const Cluster & c){
	return (c.geometric.width > params_.car_spawn.side_min ||
		c.geometric.length > params_.car_spawn.side_min)
		&&
		(c.geometric.width < params_.car_spawn.side_max &&
		c.geometric.length < params_.car_spawn.side_max)
		&&
		(c.geometric.height > params_.car_spawn.height_min && 
		c.geometric.height < params_.car_spawn.height_max)
		&&
		(c.semantic.confidence > params_.car_spawn.semantic_min)
		&&
		c.has_adjacent_free_space;
}

bool DbScan::updatePed(const Cluster & c){
	return (c.geometric.width > params_.ped_update.side_min ||
		c.geometric.length > params_.ped_update.side_min)
		&&
		(c.geometric.width < params_.ped_update.side_max &&
		c.geometric.length < params_.ped_update.side_max)
		&&
		(c.geometric.height > params_.ped_update.height_min && 
		c.geometric.height < params_.ped_update.height_max)
		&&
		(c.semantic.confidence > params_.ped_update.semantic_min);
}

bool DbScan::updateCar(const Cluster & c){
	return (c.geometric.width > params_.car_update.side_min ||
		c.geometric.length > params_.car_update.side_min)
		&&
		(c.geometric.width < params_.car_update.side_max &&
		c.geometric.length < params_.car_update.side_max)
		&&
		(c.geometric.height > params_.car_update.height_min && 
		c.geometric.height < params_.car_update.height_max)
		&&
		(c.semantic.confidence > params_.car_update.semantic_min);
}

bool DbScan::isValidSemantic(const int semantic_class){
	return semantic_class > 10;
}

bool DbScan::isKittiValidSemantic(const int semantic_class){

	// Only allow Cars and Pedestrians
	return (semantic_class == 11 || semantic_class == 13);
}

void DbScan::printCluster(const Cluster & c){

	ROS_INFO("Cluster %d Label %s Freespace %d New Track %d with to [%f,%d,%d],"
		" pos[x,y,z] [%f,%f,%f]"
		" form[w,l,h,o] [%f,%f,%f,%f]",
		c.id, c.semantic.name.c_str(), c.has_adjacent_free_space, c.is_new_track ,
		c.semantic.confidence, c.geometric.num_cells, c.semantic.diff_counter,
		c.geometric.x, c.geometric.y, c.geometric.z,
		c.geometric.width, c.geometric.length,
		c.geometric.height, c.geometric.orientation);
}

void DbScan::printObject(const Object & o){

	ROS_INFO("Object %d Label %s New Track %d with to [%f],"
		" pos[x,y,z] [%f,%f,%f]"
		" form[w,l,h,o] [%f,%f,%f,%f]",
		o.id, o.semantic_name.c_str(), o.is_new_track ,o.semantic_confidence,
		o.velo_pose.point.x, o.velo_pose.point.y, o.velo_pose.point.z,
		o.width, o.length,
		o.height, o.orientation);
}

} // namespace detection