/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

// Include guard
#ifndef visualization_H
#define visualization_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <helper/ObjectArray.h>
#include <helper/tools.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <visualization_msgs/MarkerArray.h>

// Namespaces
namespace visualization{

using namespace helper;
using namespace sensor_msgs;
using namespace message_filters;
using namespace visualization_msgs;

class Visualization{

public:

	// Default constructor
	Visualization(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~Visualization();

	virtual void process(const Image::ConstPtr & image_raw_left,
		const ObjectArrayConstPtr & detected_objects,
		const ObjectArrayConstPtr & tracked_objects);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	Tools tools_;
	std::string scenario_;
	int time_frame_;
	int linewidth_;
	int fontface_;
	double fontscale_;
	int thickness_;
	bool save_;

	// Subscriber
	Subscriber<Image> image_raw_left_sub_;
	Subscriber<ObjectArray> list_detected_objects_sub_;
	Subscriber<ObjectArray> list_tracked_objects_sub_;
	typedef sync_policies::ExactTime
		<Image, ObjectArray, ObjectArray> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync_;

	// Publisher
	ros::Publisher image_detection_pub_;
	ros::Publisher image_tracking_pub_;
	ros::Publisher cube_detection_pub_;
	ros::Publisher cube_tracking_pub_;
	ros::Publisher text_detection_pub_;
	ros::Publisher text_tracking_pub_;
	ros::Publisher arrow_tracking_pub_;

	// Class functions
	void showFirstPersonImage(
		const std::string & node_name,
		const ObjectArrayConstPtr & objects,
		const Image::ConstPtr & image_raw_left);

};

} // namespace visualization

#endif // visualization_H
