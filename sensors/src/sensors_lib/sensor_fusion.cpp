#include "sensors_lib/sensor_fusion.h"
namespace sensors{


SensorFusion::SensorFusion(ros::NodeHandle nh, ros::NodeHandle pnh):
	nh_(nh),
	pnh_(pnh),
	sub_pointcloud_stereo_(nh, "/sensors/stereo/pointcloud", 2),
	sub_pointcloud_velo_(nh, "/sensors/velo/pointcloud", 2),
	sync_(MySyncPolicy(10), sub_pointcloud_stereo_, sub_pointcloud_velo_){

	// Define Subscriber
	sync_.registerCallback(boost::bind(&SensorFusion::process, this, _1, _2));

	pub_pointcloud_fused_ = nh.advertise<PointCloud2>(
		"/sensors/fusion/pointcloud", 2);

	time_frame_ = 0;
}

SensorFusion::~SensorFusion(){

}

void SensorFusion::process(
	const PointCloud2ConstPtr & msg_pointcloud_stereo,
	const PointCloud2ConstPtr & msg_pointcloud_velo)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_velo(new pcl::PointCloud<pcl::PointXYZRGB>);
	fromROSMsg(*msg_pointcloud_velo, *pcl_velo);

	// Fuse pointclouds
	std::string target_frame = msg_pointcloud_stereo->header.frame_id;
	bool transformed = pcl_ros::transformPointCloud(
		target_frame,
		*pcl_velo,
		*pcl_velo,
		listener_);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_stereo(new pcl::PointCloud<pcl::PointXYZRGB>);
	fromROSMsg(*msg_pointcloud_stereo, *pcl_stereo);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_fused(new pcl::PointCloud<pcl::PointXYZRGB>);
	*pcl_fused = *pcl_stereo + *pcl_velo;

	PointCloud2 msg_pointcloud_fused;
	toROSMsg(*pcl_fused, msg_pointcloud_fused);
	msg_pointcloud_fused.header.stamp.nsec = msg_pointcloud_stereo->header.stamp.nsec;
	pub_pointcloud_fused_.publish(msg_pointcloud_fused);

	ROS_INFO("Sensor Fusion [%d] Stereo %d + Lidar %d = Fusion %d", 
		time_frame_,
		int(pcl_stereo->points.size()), 
		int(pcl_velo->points.size()), 
		int(pcl_fused->points.size()));

	time_frame_++;
}

} // namespace sensors
