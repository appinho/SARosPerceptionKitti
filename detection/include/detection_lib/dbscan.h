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

// Namespaces
namespace detection{

class DbScan{

public:

	// Default constructor
	DbScan(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~DbScan();

	virtual void process();


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	int time_frame_;
};

} // namespace detection

#endif // dbscan_H
