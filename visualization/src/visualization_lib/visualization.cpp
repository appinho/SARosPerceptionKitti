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
	sync_(MySyncPolicy(10), image_raw_left_sub_, list_detected_objects_sub_,
		list_tracked_objects_sub_)
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
	sync_.registerCallback(boost::bind(&Visualization::process, this, _1, _2, _3));

	// Define Publisher
	image_detection_pub_ = nh_.advertise<Image>(
		"/viz/detection/image", 2);
	image_tracking_pub_ = nh_.advertise<Image>(
		"/viz/tracking/image", 2);
	cube_detection_pub_ = nh_.advertise<Marker>(
		"/viz/detection/cubes", 2);;
	cube_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/cubes", 2);;
	text_detection_pub_= nh_.advertise<Marker>(
		"/viz/detection/texts", 2);;
	text_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/texts", 2);;
	arrow_tracking_pub_ = nh_.advertise<Marker>(
		"/viz/tracking/arrows", 2);;

	// Define class members;
	linewidth_ = 5; // negative for filled
	fontface_ =  cv::FONT_HERSHEY_SIMPLEX;
	fontscale_ = 0.7;
	thickness_ = 3;

	// Store images externally
	save_ = true;
		
	// Init counter for publishing
	time_frame_ = 0;
}

Visualization::~Visualization(){

}

void Visualization::process(const Image::ConstPtr & image_raw_left,
	const ObjectArrayConstPtr & detected_objects,
	const ObjectArrayConstPtr & tracked_objects){

	// Show detection image
	showFirstPersonImage("detection", detected_objects, image_raw_left);

	// Show tracking image
	showFirstPersonImage("tracking", tracked_objects, image_raw_left);

	// Print sensor fusion
	ROS_INFO("Publishing Visualization [%d]", time_frame_);

	// Increment time frame
	time_frame_++;
}

void Visualization::showFirstPersonImage(
	const std::string & node_name,
	const ObjectArrayConstPtr & objects,
	const Image::ConstPtr & image_raw_left){

	// Convert sensor image of raw left image to cv mat
	cv_bridge::CvImagePtr cv_raw_left_ptr;
	try{
		cv_raw_left_ptr = cv_bridge::toCvCopy(image_raw_left, 
			"bgr8");
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Copy image for drawing
	cv::Mat image = cv_raw_left_ptr->image.clone();	

	// Loop through clusters
	for(int i = 0; i < objects->list.size(); ++i){

		// Grab object
		const Object & o = objects->list[i];

		if(!o.is_track)
			continue;
		
		MatrixXf image_points = tools_.getImage2DBoundingBox(o);

		// Draw box
		cv::Point top_left = cv::Point(image_points(0,0), image_points(1,0));
		cv::Point bot_right = cv::Point(image_points(0,1), image_points(1,1));

		// Get color
		cv::Scalar color = cv::Scalar(o.r, o.g, o.b);

		cv::rectangle(image, top_left, bot_right, color, linewidth_, 8);

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
		cv::putText(image, text, top_left, fontface_, fontscale_, color,
			thickness_,	8);
	}

	if(save_){
		std::ostringstream filename;
		filename << "/home/simonappel/kitti_data/" << scenario_  << "/" 
		<< node_name << "/00000" << std::setfill('0') << std::setw(5) 
		<< time_frame_ << ".png";
		cv::imwrite(filename.str(), image);
	}

	// Publish image
	cv_bridge::CvImage cv_image;
	cv_image.image = image;
	cv_image.encoding = "bgr8";
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

} // namespace visualization