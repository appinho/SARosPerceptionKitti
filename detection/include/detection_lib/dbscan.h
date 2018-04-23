/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 23/04/2018
 *
 */

// Include guard
#ifndef dbscan_H
#define dbscan_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Namespaces
namespace detection{

using namespace sensor_msgs;

class DbScan{

public:

	// Default constructor
	DbScan(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~DbScan();

	virtual void process(const Image::ConstPtr & detection_grid);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	int time_frame_;

	// Subscriber
	ros::Subscriber image_detection_grid_sub_;
};

} // namespace detection

#endif // dbscan_H
