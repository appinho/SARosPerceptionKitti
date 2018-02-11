#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl_ros/point_cloud.h>
#include <vector>

struct Cluster{
	float x;
	float y;
	float l_x;
	float l_y;
};

class Detection{

public:
	Detection(float range);
	~Detection();
	
	void runConnectedComponent(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	std::vector<Cluster> & getClusters();

private:
	void transformWorldToImage(const float x, const float y, int * img_x, int * img_y);

	std::vector<Cluster> clusters_;

	float range_;
	int cells_per_edge_;
	float edge_size_;
};