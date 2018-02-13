#include "../include/test_kitti/tracking.h"

Tracking::Tracking(){

	is_initialized_ = false;

	n_z_laser_ = 2;
}

Tracking::~Tracking(){
	
}

void Tracking::processMeasurements(const std::vector<Cluster> & detected_clusters, const double time_stamp){

	// All other frames
	if(is_initialized_){

		std::cout << "Time stamp " << time_stamp - last_time_stamp_ << std::endl;
	}
	// First frame
	else{


		// Set initialized to true
		is_initialized_ = true;
	}

	last_time_stamp_ = time_stamp;
}

std::vector<Track> & Tracking::getTracks(){
	return tracks_;
}
