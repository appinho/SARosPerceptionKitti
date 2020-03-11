/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 11/03/2020
 *
 */

#include <ground_truth_lib/ground_truth.h>

namespace ground_truth{

/******************************************************************************/

GroundTruth::GroundTruth(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	if(ros::param::get("~filename", ground_truth_filename_)){
		ROS_INFO("Reading ground truth from %s", ground_truth_filename_.c_str());
	}
	else{
		ROS_ERROR("Could not find benchmark package");
	}

	image_raw_left_sub_ = nh.subscribe("/kitti/camera_color_left/image_raw", 1, &GroundTruth::process, this);
	// Publisher
	list_ground_truth_objects_pub_ = nh_.advertise<ObjectArray>(
		"/ground_truth/objects", 2);
		
	// Init counter for publishing
	readGroundTruth();
	time_frame_ = 0;
}

GroundTruth::~GroundTruth(){

}

void GroundTruth::process(const Image::ConstPtr& msg){
	

	ObjectArray gt_list;
	gt_list.header = msg->header;
	for(int i = 0; i < 2; i++){
		Object o_msg;
		o_msg.id = i;
		o_msg.world_pose.header.frame_id = "velo_link";
		o_msg.world_pose.point.x = 4 + 3*i;
		o_msg.world_pose.point.y = 3;
		o_msg.world_pose.point.z = 0.5;

		o_msg.heading = 1 + i;
		o_msg.velocity = 2 + 3*i;
		o_msg.width = 2;
		o_msg.length = 3;
		o_msg.height = 2;
		o_msg.orientation = 0.0;
		// o_msg.semantic_name = track.sem.name;
		// o_msg.semantic_id = track.sem.id;
		// o_msg.semantic_confidence = track.sem.confidence;
		o_msg.r = 255;
		o_msg.g = 123;
		o_msg.b = 0;
		o_msg.a = 1;

		gt_list.list.push_back(o_msg);
	}

	ROS_INFO("Publishing Ground Truth [%d]", time_frame_);
	list_ground_truth_objects_pub_.publish(gt_list);
	// Increment time frame
	time_frame_++;
}

void GroundTruth::readGroundTruth()
{
	ground_truth_results_.open(ground_truth_filename_);
	if (ground_truth_results_.is_open()) {
		std::string line;
		while(std::getline(ground_truth_results_, line))
	    {
	     	std::istringstream iss(line);
			std::vector<std::string> words(
				std::istream_iterator<std::string>{iss},
		    	std::istream_iterator<std::string>());

			if(words[2] == "Car" or words[2] == "Pedestrian"){
	    		std::cout << line << std::endl;
			}
	    }
	    std::cout << "Done" << std::endl;
	}
	else{
		ROS_WARN("Error opening ground truth file %s", ground_truth_filename_.c_str());
	}
	ground_truth_results_.close();
}

} // namespace ground_truth
