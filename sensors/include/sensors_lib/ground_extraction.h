#ifndef GROUNDEXTRACTION_H
#define GROUNDEXTRACTION_H

#include "sensors/GroundExtractionParamsConfig.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

#include <nav_msgs/OccupancyGrid.h>
// #include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
// #include <tf/transform_listener.h>
// #include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <helper/tools.h>

typedef pcl::PointXYZRGB VRGBPoint;
typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

namespace sensors{

// using namespace message_filters;
// using namespace sync_policies;
using namespace sensor_msgs;
using namespace nav_msgs;


struct PolarCell{

	enum Indexes { NOT_SET = 0, FREE = 1, UNKNOWN = 2, OCCUPIED = 3 };

	VRGBPoint min_point;
	VRGBPoint max_point;
	float ground;
	float height;
	int count;
	unsigned idx;

	PolarCell():
		height(0.0), count(0)
	{}
};

class GroundExtraction{

public:

	explicit GroundExtraction(ros::NodeHandle nh,
		ros::NodeHandle pnh);

private:

	void callback(const PointCloud2ConstPtr & msg_pointcloud);

	void semanticBasedGroundExtraction(
		const PointCloud2ConstPtr & msg_pointcloud);

	void geometricBasedGroundExtraction(
		const PointCloud2ConstPtr & msg_pointcloud);

	void initOccupancyGrid();

	void fromSensorCoordinatesToPolarCell(
		const float x, const float y, int & seg, int & bin);
	void fromPolarCellToSensorCoordinates(
		const int seg, const int bin, float & x, float & y);
	
	// Node handle
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;

	// Subscribers
	ros::Subscriber sub_pointcloud_;

	// Publisher
	ros::Publisher pub_pointcloud_ground_inliers_;
	ros::Publisher pub_pointcloud_ground_outliers_;
	ros::Publisher pub_pointcloud_ground_;
	ros::Publisher pub_pointcloud_elevated_;
	ros::Publisher pub_voxel_ground_;
	ros::Publisher pub_voxel_elevated_;
	ros::Publisher pub_occupancy_grid_;

	float sensor_frame_z_;
	float sensor_frame_z_min_;
	float sensor_frame_opening_angle_;

	float polar_grid_range_min_;
	float polar_grid_range_max_;
	float polar_grid_cell_size_;
	float polar_grid_cell_max_height_;
	int polar_grid_segments_;
	int polar_grid_bins_;

	float cart_grid_cell_size_;
	int cart_grid_height_;
	int cart_grid_width_;

	float ransac_tolerance_;
	int ransac_iterations_;

	std::vector< std::vector<PolarCell> > polar_grid_;
	OccupancyGrid::Ptr occ_grid_;

	int time_frame_;

	// Conversion functions
	void fromVeloCoordsToPolarCell(const float x, const float y,
		int & seg, int & bin);
	void fromPolarCellToVeloCoords(const int seg, const int bin,
		float & x, float & y);
	void fromVeloCoordsToCartesianCell(const float x, const float y,
		int & grid_x, int & grid_y);
	void fromCartesianCellToVeloCoords(const int grid_x,
		const int grid_y, float & x, float & y);

	bool isSidewalkOrRoad(const VRGBPoint & point);
};

} // namespace sensors

#endif // GROUNDEXTRACTION_H
