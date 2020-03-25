/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 11/03/2020
 *
 */

// Include guard
#ifndef ground_truth_H
#define ground_truth_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include "helper/ObjectArray.h"
#include "helper/tools.h"

// Namespaces
namespace ground_truth{

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace helper;

class GroundTruth{

public:

	// Default constructor
	GroundTruth(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~GroundTruth();

	void process(const Image::ConstPtr& msg);


private:

	void readGroundTruth();
	void getObjectFromLine(const std::vector<std::string> & words,
						   Object & o_msg);

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	std::fstream ground_truth_file_;
	std::string ground_truth_filename_;
	std::map<int, ObjectArray> ground_truth_objects_;
	int time_frame_;
	tf::TransformListener listener_;
	Tools tools_;

	// Subscriber
	ros::Subscriber image_raw_left_sub_;

	// Publisher
	ros::Publisher list_ground_truth_objects_pub_;

};

} // namespace ground_truth

#endif // ground_truth_H
