#ifndef sensors_H
#define sensors_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <pcl_ros/impl/transforms.hpp>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <helper/tools.h>

// typedef pcl::PointXYZ VPoint;
// typedef pcl::PointCloud<VPoint> VPointCloud;
// typedef pcl::PointXYZRGB VRGBPoint;
// typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

namespace sensors{

using namespace message_filters;
using namespace sync_policies;
using namespace sensor_msgs;
//using namespace nav_msgs;

// struct Parameters{

// 	std::string home_dir;
// 	std::string scenario;

// 	float grid_range_min;
// 	float grid_range_max;
// 	float grid_cell_size;
// 	int grid_width;
// 	int grid_height;
// 	int grid_segments;
// 	int grid_bins;
// 	float grid_cell_height;

// 	double inv_angular_res;
// 	double inv_radial_res;

// 	bool sem_ed;
// 	int sem_ed_min;
// 	int sem_ed_max;
// 	int sem_ed_kernel;

// 	float lidar_height;
// 	float lidar_opening_angle;
// 	float lidar_z_min;

// 	double ransac_tolerance;
// 	int ransac_iterations;

// };

// struct PolarCell{

// 	enum Indexes { NOT_SET = 0, FREE = 1, UNKNOWN = 2, OCCUPIED = 3 };

// 	float x_min, y_min, z_min;
// 	float z_max;
// 	float ground;
// 	float height;
// 	float rad_dist;
// 	int count;
// 	unsigned idx;

// 	PolarCell():
// 		z_min(0.0), z_max(0.0), height(0.0), rad_dist(0.0), count(0)
// 	{}
// };

class SensorFusion{

public:

	SensorFusion(ros::NodeHandle nh, ros::NodeHandle private_nh);

	virtual ~SensorFusion();

	virtual void process(
		const ImageConstPtr& msg_image_color_left,
        const CameraInfoConstPtr& msg_caminfo_color_left,
        const ImageConstPtr& msg_image_color_right,
        const CameraInfoConstPtr& msg_caminfo_color_right,
		const ImageConstPtr & msg_image_semantic,
		const PointCloud2ConstPtr & msg_pointcloud_velo
	);


private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Subscribers
	Subscriber<Image> sub_image_color_left_;
	Subscriber<CameraInfo> sub_caminfo_color_left_;
	Subscriber<Image> sub_image_color_right_;
	Subscriber<CameraInfo> sub_caminfo_color_right_;
	Subscriber<Image> sub_image_semantic_;
	Subscriber<PointCloud2> sub_pointcloud_velo_;
	
	// Synchronize input
	typedef ExactTime<Image, CameraInfo, Image, CameraInfo, Image, PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync_;

	// Publisher
	// ros::Publisher cloud_filtered_pub_;
	// ros::Publisher cloud_ground_plane_inliers_pub_;
	// ros::Publisher cloud_ground_plane_outliers_pub_;
	// ros::Publisher cloud_ground_pub_;
	// ros::Publisher cloud_elevated_pub_;
	// ros::Publisher voxel_ground_pub_;
	// ros::Publisher voxel_elevated_pub_;
	// ros::Publisher grid_occupancy_pub_;

	// // ros::Publisher image_semantic_pub_;
	// ros::Publisher cloud_semantic_pub_;
	// ros::Publisher cloud_semantic_sparse_pub_;
	// ros::Publisher image_detection_grid_pub_;
	// ros::Publisher image_bev_semantic_grid_pub_;


	// // Class members
	// Parameters params_;

	// VPointCloud::Ptr pcl_in_;
	// VPointCloud::Ptr pcl_ground_plane_;
	// VPointCloud::Ptr pcl_ground_plane_inliers_;
	// VPointCloud::Ptr pcl_ground_plane_outliers_;
	// VPointCloud::Ptr pcl_ground_;
	// VPointCloud::Ptr pcl_elevated_;
	// VPointCloud::Ptr pcl_voxel_ground_;
	// VPointCloud::Ptr pcl_voxel_elevated_;
	// std::vector< std::vector<PolarCell> > polar_grid_;
	// OccupancyGrid::Ptr occ_grid_;

	// cv::Mat sem_image_;
	// cv::Mat detection_grid_;
	// cv::Mat bev_semantic_grid_;

	// VRGBPointCloud::Ptr pcl_semantic_;
	// VRGBPointCloud::Ptr pcl_sparse_semantic_;

	// Tools tools_;

	// int time_frame_;



	// // Class functions
	// void processPointCloud(const PointCloud2::ConstPtr & cloud);
	// // void processImage(const Image::ConstPtr & image);
	// void mapPointCloudIntoImage(const VPointCloud::Ptr cloud,
	// 	const Image::ConstPtr & sem_image);

	// // Conversion functions
	// void fromVeloCoordsToPolarCell(const float x, const float y,
	// 	int & seg, int & bin);
	// void fromPolarCellToVeloCoords(const int seg, const int bin,
	// 	float & x, float & y);
	// void fromVeloCoordsToCartesianCell(const float x, const float y,
	// 	int & grid_x, int & grid_y);
	// void fromCartesianCellToVeloCoords(const int grid_x,
	// 	const int grid_y, float & x, float & y);
};

} // namespace sensors

#endif // sensors_H
