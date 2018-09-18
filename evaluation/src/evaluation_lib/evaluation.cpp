/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <evaluation_lib/evaluation.h>

namespace evaluation{

/******************************************************************************/

Evaluation::Evaluation(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	// Get scenario parameter
	int scenario;
	std::string scenario_name;
	if(ros::param::get("~scenario", scenario)){
		std::ostringstream scenario_stream;
		scenario_stream << std::setfill('0') << std::setw(4) << scenario;
		scenario_name = scenario_stream.str();
		ROS_INFO("Read scenario : %s ", scenario_name.c_str());
	}
	else{
		ROS_ERROR("Failed to read scenario");
	}

	// Delete content in file if there is one
	filename_ = 
		"~/kitti_results/"
		+ scenario_name + ".txt";
	tracking_results_.open(filename_.c_str(),
		std::ofstream::out | std::ofstream::trunc);

	tracking_results_.close();
	// Subscriber
	list_tracked_objects_sub_ = 
		nh.subscribe("/tracking/objects", 1, &Evaluation::process, this);
		
	// Init counter for publishing
	time_frame_ = 0;
}

Evaluation::~Evaluation(){

}

void Evaluation::process(const ObjectArray& tracks){

	// Write results to file
	tracking_results_.open(filename_.c_str(),
		std::ofstream::ate | std::fstream::app);
	if (tracking_results_.is_open()){
		
		for(int i = 0; i < tracks.list.size(); ++i){

			// Grab track and get additional information
			Object o = tracks.list[i];

			// Transform
			try{
				geometry_msgs::PointStamped cam_pose;
				cam_pose.header.frame_id = "camera_color_left";
				o.world_pose.header.frame_id = "world";
				listener_.transformPoint("camera_color_left",
					o.world_pose,
					cam_pose);
				listener_.transformPoint("velo_link",
					o.world_pose,
					o.velo_pose);

				// Width
				MatrixXf bounding_box = tools_.getImage2DBoundingBox(o);

				// Write information
				tracking_results_ << time_frame_ << " " << o.id << " "
					<< o.semantic_name << " 0 0 0 "
					<< bounding_box(0,0) << " " << bounding_box(1,0) << " "
					<< bounding_box(0,1) << " " << bounding_box(1,1) << " "
					<< o.height << " " << o.width << " " << o.length << " "
					<< cam_pose.point.x << " " << cam_pose.point.y << " "
					<< cam_pose.point.z << " "
					<< o.orientation << " " << o.semantic_confidence
					<< "\n";
			}
			catch(tf::TransformException& ex){
				ROS_ERROR("Received an exception trying to transform a point from"
					"\"world\" to \"cam\": %s", ex.what());
			}
		}
		tracking_results_.close();
	}
	else{
		ROS_WARN("Error opening file");
	}

	// Print sensor fusion
	ROS_INFO("Publishing Evaluation [%d]", time_frame_);

	// Increment time frame
	time_frame_++;
}

} // namespace evaluation
