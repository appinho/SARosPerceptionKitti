/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <sensor_processing/sensor_fusion.h>

namespace sensor_processing{

/******************************************************************************/

SensorFusion::SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh){

	cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
		"/kitti/velo/pointcloud", 2, &SensorFusion::process, this);

}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(
		const sensor_msgs::PointCloud2::ConstPtr & cloud
		//const sensor_msgs::Image::ConstPtr & image
	){

	ROS_INFO("Sensor Fusion: Input received");
}

} // namespace sensor_processing