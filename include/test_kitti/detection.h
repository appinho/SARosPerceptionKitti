#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl_ros/point_cloud.h>
#include <vector>
#include "parameter.h"

struct Cluster{
	float x;
	float y;
	float l_x;
	float l_y;
};

class Detection{

public:
	Detection();
	~Detection();
	
	void runConnectedComponent(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	std::vector<Cluster> & getClusters();

private:
	void transformWorldToImage(const float x, const float y, int * img_x, int * img_y);

	std::vector<Cluster> clusters_;
};