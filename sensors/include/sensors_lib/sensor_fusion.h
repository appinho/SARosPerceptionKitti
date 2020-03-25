#ifndef sensor_fusion_H
#define sensor_fusion_H

#include "sensors_lib/stereo_vision.h"
#include "sensors_lib/depth_completion.h"
#include "sensors_lib/ground_extraction.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

namespace sensors{

using namespace message_filters;
using namespace sync_policies;
using namespace sensor_msgs;


class SensorFusion{

public:

	SensorFusion(ros::NodeHandle nh, ros::NodeHandle pnh);

	virtual ~SensorFusion();

	virtual void process(
		const PointCloud2ConstPtr & msg_pointcloud_stereo,
		const PointCloud2ConstPtr & msg_pointcloud_velo
	);


private:

	// Node handle
	ros::NodeHandle nh_, pnh_;
	tf::TransformListener listener_;

	Subscriber<PointCloud2> sub_pointcloud_stereo_;
	Subscriber<PointCloud2> sub_pointcloud_velo_;
	
	// Synchronize input
	typedef ExactTime<PointCloud2, PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync_;

	// Publisher
	ros::Publisher pub_pointcloud_velo_;
	ros::Publisher pub_pointcloud_fused_;

	
	int time_frame_;

};

} // namespace sensors

#endif // sensor_fusion_H
