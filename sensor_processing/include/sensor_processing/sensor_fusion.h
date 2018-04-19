/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

// Include guard
#ifndef sensor_processing_H
#define sensor_processing_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

// Types of point and cloud to work with
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

namespace sensor_processing{

struct Parameters{

	float grid_min_range;
	float grid_max_range;
	float grid_cell_size;
	int grid_width;
	int grid_height;

	float lidar_height;
	float lidar_opening_angle;
	float lidar_min_height;

};

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

	// Class members
	Parameters params_;
	VPointCloud::Ptr pcl_in_;

	// Publisher & Subscriber
	ros::Subscriber cloud_sub_;
	//ros::Subscriber image_sub_;
	ros::Publisher cloud_pub_;
	ros::Publisher grid_pub_;

};

} // namespace sensor_processing

#endif // sensor_processing_H
