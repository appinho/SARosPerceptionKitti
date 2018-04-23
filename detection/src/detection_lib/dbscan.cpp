/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <detection_lib/dbscan.h>

namespace detection{

/******************************************************************************/

DbScan::DbScan(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	// Init counter for publishing
	time_frame_ = 0;

	image_detection_grid_sub_ = nh.subscribe("/sensor/image_detection_grid", 2,
		&DbScan::process, this);
}

DbScan::~DbScan(){

}

void DbScan::process(const Image::ConstPtr & detection_grid){

	// Print sensor fusion
	ROS_INFO("Detection [%d]: ", time_frame_);

	// Increment time frame
	time_frame_++;
}

} // namespace detection