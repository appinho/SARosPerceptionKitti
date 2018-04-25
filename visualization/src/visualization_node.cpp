/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <ros/ros.h>
#include <visualization_lib/visualization.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "visualization_node");
	visualization::Visualization visualizer(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
