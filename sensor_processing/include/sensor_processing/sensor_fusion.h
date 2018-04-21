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
#include <nav_msgs/OccupancyGrid.h>

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
	int grid_segments;
	int grid_bins;

	float inv_angular_res;
	float inv_radial_res;

	float lidar_height;
	float lidar_opening_angle;
	float lidar_min_height;

};

struct PolarCell{

	enum Indexes { NOT_SET = 0, FREE = 1, UNKNOWN = 2, OCCUPIED = 3 };

	float z_min, z_max;
	float height;
	float rad_dist;

	// Default constructor.
	PolarCell():
		z_min(0.0), z_max(0.0), height(0.0), rad_dist(0.0)
	{}
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
	std::vector< std::vector<PolarCell> > polar_grid_;
	nav_msgs::OccupancyGrid::Ptr occ_grid_;

	// Publisher & Subscriber
	ros::Subscriber cloud_sub_;
	//ros::Subscriber image_sub_;
	ros::Publisher cloud_pub_;
	ros::Publisher occ_grid_pub_;

	// Conversion functions
	void fromVeloCoordsToPolarCell(const float x, const float y,
		int & seg, int & bin);
};

} // namespace sensor_processing

#endif // sensor_processing_H
