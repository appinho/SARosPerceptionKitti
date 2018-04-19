/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

// Include guard
#ifndef sensor_processing_H
#define sensor_processing_H

#include <ros/ros.h>

namespace sensor_processing{

class SensorFusion{

public:

	// Default constructor
	SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~SensorFusion();

private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

};

} // namespace sensor_processing

#endif // sensor_processing_H
