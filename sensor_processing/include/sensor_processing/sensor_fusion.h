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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

namespace sensor_processing{

class SensorFusion{

public:

	// Default constructor
	SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~SensorFusion();

	// Processes 3D Velodyne point cloud together with raw camera image
	// and publishes the output grid message
	virtual void process(
		const sensor_msgs::PointCloud2::ConstPtr & cloud
		//const sensor_msgs::Image::ConstPtr & image
	);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Publisher & Subscriber
	ros::Subscriber cloud_sub_;
	//ros::Subscriber image_sub_;
	ros::Publisher grid_pub_;

};

} // namespace sensor_processing

#endif // sensor_processing_H
