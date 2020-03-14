/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <ros/ros.h>
#include <sensors_lib/sensor_fusion.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "sensor_setup_node");
	sensors::SensorFusion sensor_setup(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
