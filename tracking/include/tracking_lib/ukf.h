/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

// Include guard
#ifndef unscentedkf_H
#define unscentedkf_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <helper/ObjectArray.h>
#include <tf/transform_listener.h>

// Namespaces
namespace tracking{

using namespace helper;

struct Parameter{

	float da_ped_dist_pos;
	float da_ped_dist_form;
	float da_car_dist_pos;
	float da_car_dist_form;

	int tra_dim_z;
	int tra_dim_x;
	int tra_dim_x_aug;

	float tra_std_lidar_x;
	float tra_std_lidar_y;
	float tra_std_acc;
	float tra_std_yaw_rate;
	float tra_lambda;
	int tra_aging_bad;
};


class UnscentedKF{

public:

	// Default constructor
	UnscentedKF(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~UnscentedKF();

	virtual void process(const ObjectArrayConstPtr & detected_objects);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class member
	Parameter params_;
	tf::TransformListener listener_;
	int time_frame_;

	// Subscriber
	ros::Subscriber list_detected_objects_sub_;

	// Publisher
	ros::Publisher list_tracked_objects_pub_;

};

} // namespace tracking

#endif // unscentedkf_H
