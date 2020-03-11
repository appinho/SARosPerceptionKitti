/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 11/03/2020
 *
 */

#include <ros/ros.h>
#include <ground_truth_lib/ground_truth.h>

int main(int argc, char **argv){
	
	ros::init(argc, argv, "ground_truth_node");
	ground_truth::GroundTruth ground_truth_publisher(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
