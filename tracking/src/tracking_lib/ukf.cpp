/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <tracking_lib/ukf.h>

namespace tracking{

/******************************************************************************/

UnscentedKF::UnscentedKF(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	private_nh_.param("data_association/ped/dist/position",
		params_.da_ped_dist_pos, params_.da_ped_dist_pos);
	private_nh_.param("data_association/ped/dist/form",
		params_.da_ped_dist_form, params_.da_ped_dist_form);
	private_nh_.param("data_association/car/dist/position",
		params_.da_car_dist_pos, params_.da_car_dist_pos);
	private_nh_.param("data_association/car/dist/form",
		params_.da_car_dist_form, params_.da_car_dist_form);

	private_nh_.param("tracking/dim/z", params_.tra_dim_z,
		params_.tra_dim_z);
	private_nh_.param("tracking/dim/x", params_.tra_dim_x,
		params_.tra_dim_x);
	private_nh_.param("tracking/dim/x_aug", params_.tra_dim_x_aug,
		params_.tra_dim_x_aug);

	private_nh_.param("tracking/std/lidar/x", params_.tra_std_lidar_x,
		params_.tra_std_lidar_x);
	private_nh_.param("tracking/std/lidar/y", params_.tra_std_lidar_y,
		params_.tra_std_lidar_y);
	private_nh_.param("tracking/std/acc", params_.tra_std_acc,
		params_.tra_std_acc);
	private_nh_.param("tracking/std/yaw_rate", params_.tra_std_yaw_rate,
		params_.tra_std_yaw_rate);
	private_nh_.param("tracking/lambda", params_.tra_lambda,
		params_.tra_lambda);
	private_nh_.param("tracking/aging/bad", params_.tra_aging_bad,
		params_.tra_aging_bad);

	// Print parameters
	ROS_INFO_STREAM("da_ped_dist_pos " << params_.da_ped_dist_pos);
	ROS_INFO_STREAM("da_ped_dist_form " << params_.da_ped_dist_form);
	ROS_INFO_STREAM("da_car_dist_pos " << params_.da_car_dist_pos);
	ROS_INFO_STREAM("da_car_dist_form " << params_.da_car_dist_form);
	ROS_INFO_STREAM("tra_dim_z " << params_.tra_dim_z);
	ROS_INFO_STREAM("tra_dim_x " << params_.tra_dim_x);
	ROS_INFO_STREAM("tra_dim_x_aug " << params_.tra_dim_x_aug);
	ROS_INFO_STREAM("tra_std_lidar_x " << params_.tra_std_lidar_x);
	ROS_INFO_STREAM("tra_std_lidar_y " << params_.tra_std_lidar_y);
	ROS_INFO_STREAM("tra_std_acc " << params_.tra_std_acc);
	ROS_INFO_STREAM("tra_std_yaw_rate " << params_.tra_std_yaw_rate);
	ROS_INFO_STREAM("tra_lambda " << params_.tra_lambda);
	ROS_INFO_STREAM("tra_aging_bad " << params_.tra_aging_bad);

	// Define Subscriber
	list_detected_objects_sub_ = nh.subscribe("/detection/objects", 2,
		&UnscentedKF::process, this);

	// Define Publisher
	list_tracked_objects_pub_ = nh_.advertise<ObjectArray>(
		"/tracking/objects", 2);
		
	// Init counter for publishing
	time_frame_ = 0;
}

UnscentedKF::~UnscentedKF(){

}

void UnscentedKF::process(const ObjectArrayConstPtr & detected_objects){

	// Print sensor fusion
	ROS_INFO("Publishing Tracking [%d]: # Tracks", time_frame_);

	// Increment time frame
	time_frame_++;
}

/*
void UnscentedKF::runUnscentedKF(cv::Mat grid){

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

							// Get semantic of neihbor cell
							int n_semantic_class = 
								grid.at<cv::Vec3f>(n_y,n_x)[0];

							// If this semantic matches with cluster semantic
							if(n_semantic_class == semantic_class){

								// Flag neighbor cell as visited
								grid.at<cv::Vec3f>(n_y,n_x)[0] -= 20;

								// Add neighbor cell to neighbor queue
								neighbor_queue.push(cv::Point(n_x,n_y));

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

void UnscentedKF::getClusterDetails(const cv::Mat grid){

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
		c.geometric.ground_level = min_low_z;
		c.geometric.height = max_high_z - min_low_z;

		// Get orientation of bounding box
		// Minus since opencv y,x is the opposite of the velodyne frame
		c.geometric.orientation = - rect.angle;

		// Store rect as back up
		c.rect = rect;

		// Get color
		int r = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 0);
		int g = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 1);
		int b = tools_.SEMANTIC_CLASS_TO_COLOR(c.semantic.id, 2);
		c.color = cv::Scalar(r, g, b);

		// Determine if cluster can be a new track
		// Car
		if(c.semantic.id == 13){
			c.is_new_track = hasShapeOfCar(c);
		}
		// Pedestrian
		else if(c.semantic.id == 11){
			c.is_new_track = hasShapeOfPed(c);
		}
		else{
			c.is_new_track = false;
		}
	}
}

void UnscentedKF::createObjectList(){

	// Clear buffer
	object_array_.list.clear();

	// Loop through clusters to obtain object information
	for(int i = 0; i < number_of_clusters_; ++i){

		addObject(clusters_[i]);
	}

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

void UnscentedKF::addObject(const Cluster & c){

	// Create object
	Object object;
	object.id = c.id;

	// Pose in velo frame
	object.velo_pose.header.frame_id = "velo_link";
	object.velo_pose.point.x = c.geometric.x;
	object.velo_pose.point.y = c.geometric.y;
	object.velo_pose.point.z = c.geometric.ground_level;

	// Geometry
	object.width = c.geometric.width;
	object.length = c.geometric.length;
	object.height = c.geometric.height;
	object.orientation = c.geometric.orientation;

	// Semantic
	object.semantic_id = c.semantic.id;
	object.semantic_confidence = c.semantic.confidence;
	object.semantic_name = c.semantic.name;

	// Color
	object.r = c.color[2];
	object.g = c.color[1];
	object.b = c.color[0];

	// Tracking
	object.is_new_track = c.is_new_track;

	// Push back object to list
	object_array_.list.push_back(object);
}

void UnscentedKF::createDetectionImage(cv::Mat image_raw_left){

	// OpenCV Viz parameters
	int linewidth = 5; // negative for filled
	int fontface =  cv::FONT_HERSHEY_SIMPLEX;
	double fontscale = 0.7;
	int thickness = 3;

	// Loop through clusters
	for(int i = 0; i < object_array_.list.size(); ++i){

		// Grab object
		Object & o = object_array_.list[i];

		if(!o.is_new_track)
			continue;
		
		MatrixXf image_points = tools_.getImage2DBoundingBox(o);

		// Draw box
		cv::Point top_left = cv::Point(image_points(0,0), image_points(1,0));
		cv::Point bot_right = cv::Point(image_points(0,1), image_points(1,1));

		cv::rectangle(image_raw_left, top_left, bot_right,
			clusters_[o.id].color, linewidth, 8);

		// Draw text
		std::stringstream ss;
		ss 	<< o.id << "," 
			<< std::setprecision(1) << o.width << ","
			<< std::setprecision(1) << o.length << ","
			<< std::setprecision(1) << o.height << ","
			<< std::setprecision(3) << o.orientation;
		std::string text = ss.str();
		top_left.y -= 10;
		top_left.x -= 50;
		cv::putText(image_raw_left, text, top_left, fontface, fontscale,
			clusters_[o.id].color,	thickness,	8);
	}

	/*
	if(save_){
		std::ostringstream filename;
		filename << "/home/simonappel/kitti_data/" << scenario_name_ 
		<< "/detection/00000" << setfill('0') << setw(5) << msg_counter_ << ".png";
		cv::imwrite(filename.str(), raw_image);
	}

}

bool UnscentedKF::hasShapeOfPed(const Cluster & c){
	return (c.geometric.width > params_.ped_side_min ||
		c.geometric.length > params_.ped_side_min)
		&&
		(c.geometric.width < params_.ped_side_max &&
		c.geometric.length < params_.ped_side_max)
		&&
		(c.geometric.height > params_.ped_height_min && 
		c.geometric.height < params_.ped_height_max)
		&&
		(c.semantic.confidence > params_.ped_semantic_min);
}

bool UnscentedKF::hasShapeOfCar(const Cluster & c){
	return (c.geometric.width > params_.car_side_min ||
		c.geometric.length > params_.car_side_min)
		&&
		(c.geometric.width < params_.car_side_max &&
		c.geometric.length < params_.car_side_max)
		&&
		(c.geometric.height > params_.car_height_min && 
		c.geometric.height < params_.car_height_max)
		&&
		(c.semantic.confidence > params_.car_semantic_min);
}

bool UnscentedKF::isValidSemantic(const int semantic_class){
	return semantic_class > 10;
}

bool UnscentedKF::isKittiValidSemantic(const int semantic_class){

	// Only allow Cars and Pedestrians
	return (semantic_class == 11 || semantic_class == 13);
}

void UnscentedKF::printCluster(const Cluster & c){

	std::cout << std::fixed;
	std::cout << "C " << std::setw(2)
		<< c.id << " "
		<< c.semantic.name
		<< " % " << c.semantic.confidence
		<< "(" << c.geometric.cells.size()
		<< "," << c.semantic.diff_counter
		<< ") GEO"
		<< " at (x,y,z) " << c.geometric.x 
		<< "," << c.geometric.y
		<< "," << c.geometric.ground_level
		<< " ori " << c.geometric.orientation
		<< " w/along " << c.geometric.width
		<< " l/ortho " << c.geometric.length
		<< " h " << c.geometric.height
		<< std::endl;
}
*/

} // namespace tracking