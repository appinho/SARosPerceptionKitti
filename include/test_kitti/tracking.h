#include "detection.h"
#include <vector>
#include "Eigen/Dense"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;

struct Track{
	VectorXd x;
	MatrixXd P;
    MatrixXd Xsig_pred;
    std::vector<VectorXd> hist_pos;

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
        Xsig_pred = MatrixXd::Zero(TRA_N_X, 2 * TRA_N_AUG + 1);
        VectorXd pos = VectorXd::Zero(2);
		pos << cluster.x, cluster.y;
		hist_pos.push_back(pos);
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
	void DataAssociation(const std::vector<Cluster> & detected_clusters);
	void Update(const std::vector<Cluster> & detected_clusters);

	// Members
	std::vector<Track> tracks_;		///* List of tracks
	bool is_initialized_;			///* True after first processMeasurements call
	int n_z_laser_;  				///* Laser measurement dimension
	double last_time_stamp_;		///* Buffer for last time stamp
	VectorXd weights_;

	// Prediction buffer variables
	VectorXd x_aug_;				///* Buffer state vector for prediction
	MatrixXd P_aug_;				///* Buffer state covariance for prediction
	MatrixXd Xsig_aug_;
	MatrixXd L_;

	// Update buffer variables
	MatrixXd Zsig_;
	VectorXd z_pred_;
	MatrixXd S_;
	MatrixXd Tc_;
	MatrixXd R_laser_;

	// Data association
	std::vector<int> data_association_;
	float CalculateDistance(const Track & track, const Cluster detected_cluster);

	visualization_msgs::MarkerArray marker_array_;

};