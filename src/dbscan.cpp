#include "../include/test_kitti/dbscan.h"
// DEBUG
#include <iostream>

Dbscan::Dbscan(){
	minimum_neighbors_ = 5;
	epsilon_ = 3.0;
}

Dbscan::Dbscan(int minimum_neighbors, float epsilon){
	minimum_neighbors_ = minimum_neighbors;
	epsilon_ = epsilon;
}

Dbscan::~Dbscan(){

}

void Dbscan::run_dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	std::cout << "Detection starts with " << cloud->size() << " Points." << std::endl;
	
}