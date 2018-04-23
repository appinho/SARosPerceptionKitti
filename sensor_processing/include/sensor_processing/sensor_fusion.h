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
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Types of point and cloud to work with
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

// Namespaces
namespace sensor_processing{

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

// Parameter handler
struct Parameters{

	float grid_min_range;
	float grid_max_range;
	float grid_cell_size;
	int grid_width;
	int grid_height;
	int grid_segments;
	int grid_bins;
	float grid_min_height;

	double inv_angular_res;
	double inv_radial_res;

	float lidar_height;
	float lidar_opening_angle;
	float lidar_min_height;

	double ransac_tolerance;
	int ransac_iterations;

};

// Attributes of cell from polar grid
struct PolarCell{

	enum Indexes { NOT_SET = 0, FREE = 1, UNKNOWN = 2, OCCUPIED = 3 };

	float x_min, y_min, z_min;
	float z_max;
	float ground;
	float height;
	float rad_dist;
	int count;
	unsigned idx;

	// Default constructor.
	PolarCell():
		z_min(0.0), z_max(0.0), height(0.0), rad_dist(0.0), count(0)
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
		const sensor_msgs::PointCloud2::ConstPtr & cloud,
		const sensor_msgs::Image::ConstPtr & image
	);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Class members
	Parameters params_;
	VPointCloud::Ptr pcl_in_;
	VPointCloud::Ptr pcl_ground_plane_;
	VPointCloud::Ptr pcl_ground_;
	VPointCloud::Ptr pcl_elevated_;
	VPointCloud::Ptr pcl_voxel_ground_;
	VPointCloud::Ptr pcl_voxel_elevated_;
	std::vector< std::vector<PolarCell> > polar_grid_;
	OccupancyGrid::Ptr occ_grid_;
	int time_frame_;

	// Publisher
	ros::Publisher cloud_filtered_pub_;
	ros::Publisher cloud_ground_plane_pub_;
	ros::Publisher cloud_ground_pub_;
	ros::Publisher cloud_elevated_pub_;
	ros::Publisher voxel_ground_pub_;
	ros::Publisher voxel_elevated_pub_;
	ros::Publisher grid_occupancy_pub_;

	// Subscriber
	Subscriber<PointCloud2> cloud_sub_;
	Subscriber<Image> image_sub_;
	typedef sync_policies::ExactTime<PointCloud2, Image> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync_;

	// Conversion functions
	void fromVeloCoordsToPolarCell(const float x, const float y,
		int & seg, int & bin);
	void fromPolarCellToVeloCoords(const int seg, const int bin,
		float & x, float & y);
};

} // namespace sensor_processing

#endif // sensor_processing_H
