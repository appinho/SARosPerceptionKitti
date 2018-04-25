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
	//list_tracked_objects_sub_(nh, "/detection/objects", 2),
	sync_(MySyncPolicy(10), image_raw_left_sub_, list_detected_objects_sub_)
	{

	// Define Subscriber
	sync_.registerCallback(boost::bind(&Visualization::process, this, _1, _2));

	// Define Publisher
	image_detection_pub_ = nh_.advertise<Image>(
		"/viz/image/detection", 2);
	image_tracking_pub_ = nh_.advertise<Image>(
		"/viz/image/tracking", 2);
		
	// Init counter for publishing
	time_frame_ = 0;
}

Visualization::~Visualization(){

}

void Visualization::process(const Image::ConstPtr & image_raw_left,
	const ObjectArrayConstPtr & detected_objects){

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

	// Show detection image
	cv::Mat detection_image = 
		showFirstPersonImage(detected_objects, cv_raw_left_ptr->image);
	// Publish image
	cv_bridge::CvImage cv_det_image;
	cv_det_image.image = detection_image;
	cv_det_image.encoding = "bgr8";
	cv_det_image.header.stamp = image_raw_left->header.stamp;
	image_detection_pub_.publish(cv_det_image.toImageMsg());

	// Show tracking image
	//showFirstPersonImage(tracked_objects, cv_raw_left_ptr->image);

	// Print sensor fusion
	ROS_INFO("Publishing Visualization [%d]", time_frame_);

	// Increment time frame
	time_frame_++;
}

cv::Mat Visualization::showFirstPersonImage(
	const ObjectArrayConstPtr & objects,
	cv::Mat image_raw_left){

	cv::Mat image = image_raw_left.clone();	

	// OpenCV Viz parameters
	int linewidth = 5; // negative for filled
	int fontface =  cv::FONT_HERSHEY_SIMPLEX;
	double fontscale = 0.7;
	int thickness = 3;

	// Loop through clusters
	for(int i = 0; i < objects->list.size(); ++i){

		// Grab object
		const Object & o = objects->list[i];

		if(!o.is_new_track)
			continue;
		
		MatrixXf image_points = tools_.getImage2DBoundingBox(o);

		// Draw box
		cv::Point top_left = cv::Point(image_points(0,0), image_points(1,0));
		cv::Point bot_right = cv::Point(image_points(0,1), image_points(1,1));
		cv::Scalar color = cv::Scalar(o.r, o.g, o.b);

		cv::rectangle(image, top_left, bot_right, color, linewidth, 8);

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
		cv::putText(image, text, top_left, fontface, fontscale, color,
			thickness,	8);
	}

	return image;

	/*
	if(save_){
		std::ostringstream filename;
		filename << "/home/simonappel/kitti_data/" << scenario_name_ 
		<< "/detection/00000" << setfill('0') << setw(5) << msg_counter_ << ".png";
		cv::imwrite(filename.str(), raw_image);
	}
	*/
}

} // namespace visualization