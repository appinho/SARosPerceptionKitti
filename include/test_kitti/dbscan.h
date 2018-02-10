#include <vector>
#include <pcl_ros/point_cloud.h>

struct Cluster{
	float x;
	float y;
	float w;
	float h;
};

class Dbscan{

  public:
  	Dbscan();
  	Dbscan(int minimum_neighbors, float epsilon);
  	~Dbscan();

    void run_dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<Cluster> get_clusters();

  private:
  	int minimum_neighbors_;
  	float epsilon_;
  	std::vector<Cluster> clusters_;
};