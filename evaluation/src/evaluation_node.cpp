/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 26/04/2018
 *
 */

#include <ros/ros.h>
#include <evaluation_lib/evaluation.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "evaluation_node");
	evaluation::Evaluation evaluator(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
