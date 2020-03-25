#ifndef sensors_H
#define sensors_H

#include "sensors_lib/stereo_vision.h"
#include "sensors_lib/depth_completion.h"
#include "sensors_lib/sensor_fusion.h"
#include "sensors_lib/ground_extraction.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace sensors{

class SensorSetup{

public:

	SensorSetup(ros::NodeHandle nh, ros::NodeHandle pnh);
	virtual ~SensorSetup();

private:

	void process(const PointCloud2ConstPtr & msg_pointcloud_velo);

	// Modules
	StereoVision stereo_vision_;
	DepthCompletion depth_completion_;
	SensorFusion sensor_fusion_;
	GroundExtraction ground_extraction_;

	ros::Subscriber sub_pointcloud_velo_;
	ros::Publisher pub_pointcloud_velo_;
};

} // namespace sensors

#endif // sensors_H