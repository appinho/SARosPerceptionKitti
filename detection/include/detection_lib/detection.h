// Include guard
#ifndef DETECTION_H
#define DETECTION_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include "helper/tools.h"
#include "helper/ObjectArray.h"
#include <tf/transform_listener.h>

// Namespaces
namespace detection{

using namespace sensor_msgs;
using namespace helper;

struct ObjectAttributes{
	float side_min;
	float side_max;
	float height_min;
	float height_max;
	float semantic_min;
};

struct Parameter{

	float grid_range_max;
	float grid_cell_size;

	ObjectAttributes car_spawn;
	ObjectAttributes car_update;
	ObjectAttributes ped_spawn;
	ObjectAttributes ped_update;
};

// Semantic information of a cluster
struct Semantic{

	std::map<int,int> classes;
	float confidence;
	int id;
	std::string name;
	int diff_counter;
};

// Geometric information of a cluster
struct Geometric{

	float x;
	float y;
	float z;
	float width;
	float length;
	float height;
	float orientation;

	std::vector<cv::Point> cells;
	int num_cells;
};

// Information of a cluster
struct Cluster{

	// ID
	int id;

	// Kernel size
	int kernel;

	// Semantic information
	Semantic semantic;

	// Geometric information
	Geometric geometric;

	// Misc stuff
	cv::RotatedRect rect;
	cv::Scalar color;
	bool is_new_track;
};

typedef pcl::PointXYZRGB VRGBPoint;
typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

class Detection{

public:

	// Default constructor
	Detection(ros::NodeHandle nh, ros::NodeHandle pnh);

	// Virtual destructor
	virtual ~Detection();

	virtual void process(
		const PointCloud2ConstPtr & msg_pointcloud_elevated);


private:

	void produce2Dgrid(const VRGBPointCloud::Ptr & pcl_cloud);

	bool isKittiValidSemantic(const int semantic_class);

	void fromVeloCoordsToCartesianCell(
		const float x, const float y, int & grid_x, int & grid_y);

	void fromCartesianCellToVeloCoords(
		const int grid_x, const int grid_y, float & x, float & y);

	void fromRectangleCoordsToVeloCoords(
		const float grid_x, const float grid_y, float & x, float & y);
	// Node handle
	ros::NodeHandle nh_, pnh_;

	// // Class member
	std::vector<Cluster> clusters_;
	ObjectArray object_array_;
	// int number_of_clusters_;
	// int time_frame_;
	Parameter params_;
	Tools tools_;
	tf::TransformListener listener_;

	// Subscriber
	ros::Subscriber sub_pointcloud_elevated_;

	// Publisher
	ros::Publisher pub_detected_objects_;
	ros::Publisher pub_pointcloud_per_cell_;

	// Params
	float sensor_frame_z_;
	int cart_grid_height_;
	int cart_grid_width_;
	float cart_grid_cell_size_;

	int time_frame_;

	// // Class functions
	// void DBSCAN(cv::Mat grid);
	// void filterClusters(const cv::Mat grid);
	void fillObjectList();
	void addObject(const Cluster & c);
	bool spawnPed(const Cluster & c);
	bool spawnCar(const Cluster & c);
	bool updatePed(const Cluster & c);
	bool updateCar(const Cluster & c);
	// bool isValidSemantic(const int semantic_class);
	// bool isKittiValidSemantic(const int semantic_class);
	void printCluster(const Cluster & c);
	void printObject(const Object & o);

};

} // namespace detection

#endif // DETECTION_H
