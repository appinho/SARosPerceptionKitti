#include "../include/test_kitti/tracking.h"

Tracking::Tracking(){

	is_initialized_ = false;

	n_z_laser_ = 2;
}

Tracking::~Tracking(){
	
}

void Tracking::processMeasurements(const std::vector<Cluster> & detected_clusters){
	std::cout << detected_clusters[0].x << std::endl;
}

std::vector<Track> & Tracking::getTracks(){
	return tracks_;
}
