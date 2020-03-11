/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <visualization_lib/visualization.h>

namespace visualization{

/******************************************************************************/

Visualization::Visualization(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh),
	image_raw_left_sub_(nh,	"/kitti/camera_color_left/image_raw", 2),
	list_detected_objects_sub_(nh, "/detection/objects", 2),
	list_tracked_objects_sub_(nh, "/tracking/objects", 2),
	list_ground_truth_objects_sub_(nh, "/ground_truth/objects", 2),
	sync_det_(MySyncPolicy(10), image_raw_left_sub_, list_detected_objects_sub_),
	sync_tra_(MySyncPolicy(10), image_raw_left_sub_, list_tracked_objects_sub_),
	sync_gt_(MySyncPolicy(10), image_raw_left_sub_, list_ground_truth_objects_sub_)
	{

	// Get scenario parameter
	int scenario;
	if(ros::param::get("~scenario", scenario)){
		std::ostringstream scenario_stream;
		scenario_stream << std::setfill('0') << std::setw(4) << scenario;
		scenario_ = scenario_stream.str();
	}
	else{
		ROS_ERROR("Failed to read scenario");
	}

	// Define Subscriber
	sync_det_.registerCallback(boost::bind(&Visualization::processDetection, this, _1, _2));
	sync_tra_.registerCallback(boost::bind(&Visualization::processTracking, this, _1, _2));
	sync_gt_.registerCallback(boost::bind(&Visualization::processGroundTruth, this, _1, _2));

	// Define class members;
	linewidth_ = 5; // negative for filled
	fontface_ =  cv::FONT_HERSHEY_SIMPLEX;
	fontscale_ = 0.7;
	thickness_ = 3;

	// Init viz buffers
	viz_buffer_ = 60;
	for(int i = 0; i < viz_buffer_; i++){
		detection_.push_back(initVizObject(i));
	}
	for(int i = 0; i < viz_buffer_; i++){
		tracking_.push_back(initVizObject(i));
	}
	for(int i = 0; i < viz_buffer_; i++){
		ground_truth_.push_back(initVizObject(i));
	}

	// Define Publisher
	image_detection_pub_ = nh_.advertise<Image>(
		"/viz/detection/image", viz_buffer_);
	image_tracking_pub_ = nh_.advertise<Image>(
		"/viz/tracking/image", viz_buffer_);
	image_ground_truth_pub_ = nh_.advertise<Image>(
		"/viz/ground_truth/image", viz_buffer_);
	cube_detection_pub_ = nh_.advertise<Marker>(
		"/viz/detection/cubes", viz_buffer_);
	cube_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/cubes", viz_buffer_);
	cube_ground_truth_pub_ = nh_.advertise<Marker>(
		"/viz/ground_truth/cubes", viz_buffer_);
	text_detection_pub_= nh_.advertise<Marker>(
		"/viz/detection/texts", viz_buffer_);
	text_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/texts", viz_buffer_);
	text_ground_truth_pub_ = nh_.advertise<Marker>(
		"/viz/ground_truth/texts", viz_buffer_);
	arrow_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/arrows", viz_buffer_);

	// Store images externally
	save_ = false;
		
	// Init counter for publishing
	det_time_frame_ = 0;
	tra_time_frame_ = 0;
	gt_time_frame_ = 0;
}

Visualization::~Visualization(){

}

void Visualization::processDetection(const Image::ConstPtr & image_raw_left,
	const ObjectArrayConstPtr & detected_objects){

	// Show detection image
	showFirstPersonImage("detection", detected_objects, image_raw_left);

	// Show rviz markers
	showRVizMarkers("detection", detected_objects);

	// Print sensor fusion
	ROS_INFO("Publishing Visualization for Detection [%d]", det_time_frame_);

	// Increment time frame
	det_time_frame_++;
}

void Visualization::processTracking(const Image::ConstPtr & image_raw_left,
	const ObjectArrayConstPtr & tracked_objects){

	// Show tracking image
	showFirstPersonImage("tracking", tracked_objects, image_raw_left);

	// Show rviz markers
	showRVizMarkers("tracking", tracked_objects);

	// Print sensor fusion
	ROS_INFO("Publishing Visualization for Tracking [%d]", tra_time_frame_);

	// Increment time frame
	tra_time_frame_++;
}

void Visualization::processGroundTruth(const Image::ConstPtr & image_raw_left,
	const ObjectArrayConstPtr & ground_truth_objects){

	// Show tracking image
	//showFirstPersonImage("ground_truth", ground_truth_objects, image_raw_left);

	// Show rviz markers
	showRVizMarkers("ground_truth", ground_truth_objects);

	// Print sensor fusion
	ROS_INFO("Publishing Visualization for Ground Truth [%d]", gt_time_frame_);

	// Increment time frame
	gt_time_frame_++;
}

void Visualization::showFirstPersonImage(
	const std::string & node_name,
	const ObjectArrayConstPtr & objects,
	const Image::ConstPtr & image_raw_left){

	// Convert sensor image of raw left image to cv mat
	cv_bridge::CvImagePtr cv_raw_left_ptr;
	try{
		cv_raw_left_ptr = cv_bridge::toCvCopy(image_raw_left, 
			"rgb8");
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Copy image for drawing
	cv::Mat image = cv_raw_left_ptr->image;	

	// Loop through clusters
	for(int i = 0; i < objects->list.size(); ++i){

		// Grab object
		const Object & o = objects->list[i];

		//if(!o.is_new_track)
			//continue;
		
		MatrixXf image_points = tools_.getImage2DBoundingBox(o);

		// Draw box
		cv::Point top_left = cv::Point(image_points(0,0), image_points(1,0));
		cv::Point bot_right = cv::Point(image_points(0,1), image_points(1,1));

		// Get color
		cv::Scalar color = cv::Scalar(o.r, o.g, o.b);

		cv::rectangle(image, top_left, bot_right, color, linewidth_, 4);

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
		//cv::putText(image, text, top_left, fontface_, fontscale_, color,
		//	thickness_,	8);
	}

	if(save_){
		std::ostringstream filename;

		if(node_name == "detection"){
			filename << "~/kitti_data/" << scenario_  << "/" 
				<< node_name << "/00000" << std::setfill('0') << std::setw(5) 
				<< det_time_frame_ << ".png";			
		}
		else if(node_name == "tracking"){
			filename << "~/kitti_data/" << scenario_  << "/" 
				<< node_name << "/00000" << std::setfill('0') << std::setw(5) 
				<< tra_time_frame_ << ".png";
		}

		cv::imwrite(filename.str(), image);
	}

	// Publish image
	cv_bridge::CvImage cv_image;
	cv_image.image = image;
	cv_image.encoding = "rgb8";
	cv_image.header.stamp = image_raw_left->header.stamp;

	// Print and publish
	if(node_name == "detection"){
		ROS_INFO("Publishing First Person Detection Image!");
		image_detection_pub_.publish(cv_image.toImageMsg());
	}
	else if(node_name == "tracking"){
		ROS_INFO("Publishing First Person Tracking Image!");
		image_tracking_pub_.publish(cv_image.toImageMsg());
	}
}

void Visualization::showRVizMarkers(
	const std::string & node_name,
	const ObjectArrayConstPtr & objects){

	// Update Markers
	for(int i = 0; i < objects->list.size(); ++i){

		if(node_name == "detection"){
			VizObject & viz_obj = detection_[i];
			updateBoundingBox(viz_obj, i, objects->list[i]);
			updateText(viz_obj, i, objects->list[i]);
			cube_detection_pub_.publish(viz_obj.bb);
			text_detection_pub_.publish(viz_obj.txt);
		}
		else if(node_name == "tracking"){
			VizObject & viz_obj = tracking_[i];
			updateBoundingBox(viz_obj, i, objects->list[i]);
			updateArrow(viz_obj, i, objects->list[i]);
			updateText(viz_obj, i, objects->list[i]);
			cube_tracking_pub_.publish(viz_obj.bb);
			arrow_tracking_pub_.publish(viz_obj.arr);
			text_tracking_pub_.publish(viz_obj.txt);
		}
		else if(node_name == "ground_truth"){
			VizObject & viz_obj = tracking_[i];
			updateBoundingBox(viz_obj, i, objects->list[i]);
			// updateArrow(viz_obj, i, objects->list[i]);
			updateText(viz_obj, i, objects->list[i]);
			cube_ground_truth_pub_.publish(viz_obj.bb);
			// arrow_ground_truth_pub_.publish(viz_obj.arr);
			text_ground_truth_pub_.publish(viz_obj.txt);
		}
	}
	// Hide markers
	for(int i = objects->list.size(); i < viz_buffer_; ++i){

		if(node_name == "detection"){
			VizObject & viz_obj = detection_[i];
			hide(viz_obj);
			cube_detection_pub_.publish(viz_obj.bb);
			text_detection_pub_.publish(viz_obj.txt);
		}
		else if(node_name == "tracking"){
			VizObject & viz_obj = tracking_[i];
			hide(viz_obj);
			cube_tracking_pub_.publish(viz_obj.bb);
			arrow_tracking_pub_.publish(viz_obj.arr);
			text_tracking_pub_.publish(viz_obj.txt);
		}
		else if(node_name == "ground_truth"){
			VizObject & viz_obj = ground_truth_[i];
			hide(viz_obj);
			cube_ground_truth_pub_.publish(viz_obj.bb);
			// arrow_ground_truth_pub_.publish(viz_obj.arr);
			text_ground_truth_pub_.publish(viz_obj.txt);
		}
	}
}

VizObject Visualization::initVizObject(
	const int i){

	// Create object
	VizObject viz_obj;

	// Init Bounding Box
	viz_obj.bb.id = i;
	viz_obj.bb.header.frame_id = "velo_link";
	viz_obj.bb.ns = "my_namespace";
	viz_obj.bb.type = Marker::CUBE;
	viz_obj.bb.action = Marker::ADD;

	// Init Arrow
	viz_obj.arr.id = i;
	viz_obj.arr.header.frame_id = "velo_link";
	viz_obj.arr.ns = "my_namespace";
	viz_obj.arr.type = Marker::ARROW;
	viz_obj.arr.action = Marker::ADD;

	// Init Text
	viz_obj.txt.id = i;
	viz_obj.txt.header.frame_id = "velo_link";
	viz_obj.txt.ns = "my_namespace";
	viz_obj.txt.type = Marker::TEXT_VIEW_FACING;
	viz_obj.txt.action = Marker::ADD;

	return viz_obj;
}

void Visualization::updateBoundingBox(VizObject & viz_obj, const int i, const Object & obj){

	// Fill in bounding box information
	viz_obj.bb.action = Marker::ADD;
	viz_obj.bb.ns = "my_namespace";
	viz_obj.bb.type = Marker::CUBE;
	viz_obj.bb.header.frame_id = "velo_link";
	viz_obj.bb.id = i;

	viz_obj.bb.pose.position.x = obj.velo_pose.point.x;
	viz_obj.bb.pose.position.y = obj.velo_pose.point.y;
	viz_obj.bb.pose.position.z = obj.velo_pose.point.z + obj.height/2;

	tf::Quaternion quat = tf::createQuaternionFromRPY(
		obj.orientation/180*M_PI, 0, 0);
	viz_obj.bb.pose.orientation.w = quat[3];
	viz_obj.bb.pose.orientation.x = quat[2];
	viz_obj.bb.pose.orientation.y = quat[1];
	viz_obj.bb.pose.orientation.z = quat[0];

	viz_obj.bb.scale.x = obj.length;
	viz_obj.bb.scale.y = obj.width;
	viz_obj.bb.scale.z = obj.height;

	viz_obj.bb.color.a = obj.a;
	viz_obj.bb.color.r = float(obj.r)/255;
	viz_obj.bb.color.g = float(obj.g)/255;
	viz_obj.bb.color.b = float(obj.b)/255;

}

void Visualization::updateText(VizObject & viz_obj, const int i, const Object & obj){

	// Fill in text information
	viz_obj.txt.action = Marker::ADD;
	viz_obj.txt.ns = "my_namespace";
	viz_obj.txt.type = Marker::TEXT_VIEW_FACING;
	viz_obj.txt.header.frame_id = "velo_link";
	viz_obj.txt.id = i;

	viz_obj.txt.pose.position.x = obj.velo_pose.point.x;
	viz_obj.txt.pose.position.y = obj.velo_pose.point.y;
	viz_obj.txt.pose.position.z = obj.velo_pose.point.z + obj.height * 1.5;

	viz_obj.txt.pose.orientation.w = 0;
	viz_obj.txt.pose.orientation.x = 0;
	viz_obj.txt.pose.orientation.y = 0;
	viz_obj.txt.pose.orientation.z = 1;

	viz_obj.txt.scale.x = 2.0;
	viz_obj.txt.scale.y = 2.0;
	viz_obj.txt.scale.z = 2.0;

	viz_obj.txt.color.a = obj.a;
	viz_obj.txt.color.r = float(obj.r)/255;
	viz_obj.txt.color.g = float(obj.g)/255;
	viz_obj.txt.color.b = float(obj.b)/255;

	viz_obj.txt.text = std::to_string(obj.id);
}

void Visualization::updateArrow(VizObject & viz_obj, const int i, const Object & obj){

	// Fill in arrow information
	viz_obj.arr.action = Marker::ADD;
	viz_obj.arr.ns = "my_namespace";
	viz_obj.arr.type = Marker::ARROW;
	viz_obj.arr.header.frame_id = "world";
	viz_obj.arr.id = i;

	viz_obj.arr.pose.position.x = obj.world_pose.point.x;
	viz_obj.arr.pose.position.y = obj.world_pose.point.y;
	viz_obj.arr.pose.position.z = obj.world_pose.point.z + obj.height * 0.5;

	tf::Quaternion quat = tf::createQuaternionFromRPY(
		obj.heading, 0, 0);
	viz_obj.arr.pose.orientation.w = quat[3];
	viz_obj.arr.pose.orientation.x = quat[2];
	viz_obj.arr.pose.orientation.y = quat[1];
	viz_obj.arr.pose.orientation.z = quat[0];

	if(abs(obj.velocity) < 0.1){
		viz_obj.arr.scale.x = 0.1;
		viz_obj.arr.scale.y = 0.1;
		viz_obj.arr.scale.z = 0.1;
	}
	else{
		viz_obj.arr.scale.x = obj.velocity;
		viz_obj.arr.scale.y = 0.5;
		viz_obj.arr.scale.z = 0.5;
	}

	viz_obj.arr.color.a = obj.a;
	viz_obj.arr.color.r = float(obj.r)/255;
	viz_obj.arr.color.g = float(obj.g)/255;
	viz_obj.arr.color.b = float(obj.b)/255;
}

void Visualization::hide(VizObject & viz_obj){

	viz_obj.bb.action = Marker::DELETE;
	viz_obj.arr.action = Marker::DELETE;
	viz_obj.txt.action = Marker::DELETE;
}

} // namespace visualization
