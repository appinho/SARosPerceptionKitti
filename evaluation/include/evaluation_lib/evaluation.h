/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

// Include guard
#ifndef evaluation_H
#define evaluation_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <helper/ObjectArray.h>
#include <helper/tools.h>

// Namespaces
namespace evaluation{

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace helper;

class Evaluation{

public:

	// Default constructor
	Evaluation(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~Evaluation();

	void process(const ObjectArray& tracks);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	std::ofstream tracking_results_;
	std::string filename_;
	int time_frame_;
	tf::TransformListener listener_;
	Tools tools_;

	// Subscriber
	ros::Subscriber list_tracked_objects_sub_;

};

} // namespace evaluation

#endif // evaluation_H
