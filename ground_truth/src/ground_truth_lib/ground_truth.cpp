/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 11/03/2020
 *
 */

#include <ground_truth_lib/ground_truth.h>
#include <math.h>

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
	
	if(ground_truth_objects_.find(time_frame_) != ground_truth_objects_.end())
	{
		ObjectArray & gt_list = ground_truth_objects_[time_frame_];
		gt_list.header = msg->header;
		for(auto & obj: gt_list.list){
			try{
				listener_.transformPoint("velo_link",
					obj.cam_pose,
					obj.velo_pose);
			}
			catch(tf::TransformException& ex){
				ROS_WARN("Received an exception trying to transform a point from "
					"\"camera_color_left\" to \"velo_link\": %s", ex.what());
			}
		}
		list_ground_truth_objects_pub_.publish(gt_list);
	}
	ROS_INFO("Publishing Ground Truth [%d]", time_frame_);
	// Increment time frame
	time_frame_++;
}

void GroundTruth::readGroundTruth()
{
	ground_truth_file_.open(ground_truth_filename_);
	if (ground_truth_file_.is_open())
	{
		std::string line;
		while(std::getline(ground_truth_file_, line))
	    {
	     	std::istringstream iss(line);
			std::vector<std::string> words(
				std::istream_iterator<std::string>{iss},
		    	std::istream_iterator<std::string>());

			if(words[2] == "Car" or words[2] == "Pedestrian")
			{
	    		int timestamp = std::stoi(words[0]);
	    		Object obj;
	    		getObjectFromLine(words, obj);
 	    		if(ground_truth_objects_.find(timestamp) == ground_truth_objects_.end())
	    		{
	    			ground_truth_objects_.insert({timestamp, ObjectArray()});
	    		}
	    		ground_truth_objects_[timestamp].list.push_back(obj);
			}
	    }
	    std::cout << "Done" << std::endl;
	}
	else
	{
		ROS_WARN("Error opening ground truth file %s", ground_truth_filename_.c_str());
	}
	ground_truth_file_.close();
}

void GroundTruth::getObjectFromLine(const std::vector<std::string> & words,
									Object & obj)
{
	obj.id = std::stoi(words[1]);
	obj.cam_pose.header.frame_id = "camera_color_left";
	obj.cam_pose.point.x = std::stof(words[13]);
	obj.cam_pose.point.y = std::stof(words[14]);
	obj.cam_pose.point.z = std::stof(words[15]);
	obj.heading = std::stof(words[5]);
	// obj.velocity = 0;
	obj.height = std::stof(words[10]);
	obj.width = std::stof(words[11]);
	obj.length = std::stof(words[12]);
	obj.semantic_name = words[2];
	float orientation = - std::stof(words[16]) / M_PI * 180 - 90;
	obj.orientation = orientation;
	if(obj.semantic_name == "Car"){
		obj.r = 0;
		obj.g = 0;
		obj.b = 255;
	}
	else{
		obj.r = 255;
		obj.g = 0;
		obj.b = 0;
	}
	obj.a = 0.3;
}

} // namespace ground_truth
