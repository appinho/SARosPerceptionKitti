#include "detection.h"
#include <vector>
#include "Eigen/Dense"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;

struct Track{
	VectorXd x;
	MatrixXd P;

	Track(const Cluster & cluster){
		x = VectorXd::Zero(TRA_N_X);
		x[0] = cluster.x;
		x[1] = cluster.y;
		P = MatrixXd::Zero(TRA_N_X, TRA_N_X);
		P <<  	1,  0,  0,  0,  0,
           		0,  1,  0,  0,  0,
           		0,  0, 10,  0,  0,
           		0,  0,  0,  1,  0,
           		0,  0,  0,  0,  1;
	}
};

class Tracking{

public:
	Tracking();
	~Tracking();

	void processMeasurements(const std::vector<Cluster> & detected_clusters, const double time_stamp);
	visualization_msgs::MarkerArray & showTracks();
	std::vector<Track> & getTracks();

private:
	// Functions
	void Prediction(const double delta_t);
	void Update(const std::vector<Cluster> & detected_clusters);

	// Members
	std::vector<Track> tracks_;		///* List of tracks
	bool is_initialized_;			///* True after first processMeasurements call
	int n_z_laser_;  				///* Laser measurement dimension
	double last_time_stamp_;		///* Buffer for last time stamp
	VectorXd x_aug_;				///* Buffer state vector for prediction
	MatrixXd P_aug_;				///* Buffer state covariance for prediction
	MatrixXd Xsig_aug_;
	MatrixXd L_;
};