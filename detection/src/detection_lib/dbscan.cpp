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
	private_nh_(private_nh){

	// Init counter for publishing
	time_frame_ = 0;
}

DbScan::~DbScan(){

}

void DbScan::process(){

}

} // namespace detection