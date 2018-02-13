#include "detection.h"
#include <vector>
#include "Eigen/Dense"

const int n_x = 5;
const int n_aug = 7;

struct Track{
	Eigen::VectorXd x;
	Eigen::MatrixXd P;

	Track(int n_x){
		x = Eigen::VectorXd::Zero(n_x);
		P = Eigen::MatrixXd::Zero(n_x,n_x);
	}
};

class Tracking{

public:
	Tracking();
	~Tracking();

	void processMeasurements(const std::vector<Cluster> & detected_clusters, const double time_stamp);
	std::vector<Track> & getTracks();

private:
	std::vector<Track> tracks_;

	// Members
	bool is_initialized_;	///* True after first processMeasurements call
	int n_z_laser_;  		///* Laser measurement dimension
	double last_time_stamp_;
};