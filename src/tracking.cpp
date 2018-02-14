#include "../include/test_kitti/tracking.h"

Tracking::Tracking(){

	is_initialized_ = false;

	n_z_laser_ = 2;

	x_aug_ = VectorXd(TRA_N_AUG);
	P_aug_ = MatrixXd(TRA_N_AUG, TRA_N_AUG);
	Xsig_aug_ = MatrixXd(TRA_N_AUG, 2 * TRA_N_AUG + 1);
	Xsig_pred_ = MatrixXd::Zero(TRA_N_X, 2 * TRA_N_AUG + 1);

	// Init weights
	weights_ = VectorXd(2 * TRA_N_AUG + 1);
	// Set weights
	weights_(0) = TRA_LAMBDA / (TRA_LAMBDA + TRA_N_AUG);
	for (int i = 1; i < 2 * TRA_N_AUG + 1; i++) {  //2n+1 weights
		weights_(i) = 0.5 / (TRA_N_AUG + TRA_LAMBDA);
	}

	std::cout << "weights = " << std::endl << weights_ << std::endl;

}

Tracking::~Tracking(){
	
}

void Tracking::processMeasurements(const std::vector<Cluster> & detected_clusters, const double time_stamp){

	// All other frames
	if(is_initialized_){

		// Calculate time difference between frames
		double delta_t = time_stamp - last_time_stamp_;

		// Prints
		std::cout << "Delta t " << delta_t << std::endl;

		// Prediction
		Prediction(delta_t);

		// Update
		Update(detected_clusters);
	}
	// First frame
	else{
		// Initialize tracks
		Track new_track = Track(detected_clusters[13]);
		tracks_.push_back(new_track);

		// Set initialized to true
		is_initialized_ = true;
	}

	last_time_stamp_ = time_stamp;

	// Prints
	std::cout << "Number of tracks " << tracks_.size() << std::endl;

}

void Tracking::Prediction(const double delta_t){

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

	  	// 1. Generate augmented sigma points

	  	// Fill augmented mean state
	  	x_aug_.head(5) = track.x;
	  	x_aug_(5) = 0;
	  	x_aug_(6) = 0;

	  	// Fill augmented covariance matrix
	  	P_aug_.fill(0.0);
	  	P_aug_.topLeftCorner(5,5) = track.P;
	  	P_aug_(5,5) = TRA_STD_A * TRA_STD_A;
	  	P_aug_(6,6) = TRA_STD_YAWDD * TRA_STD_YAWDD;

	  	// Create square root matrix
	  	L_ = P_aug_.llt().matrixL();

	  	// Create augmented sigma points
	  	Xsig_aug_.col(0)  = x_aug_;
	  	for (int j = 0; j< TRA_N_AUG; j++){
	    	Xsig_aug_.col(j + 1)          		= x_aug_ + sqrt(TRA_LAMBDA + TRA_N_AUG) * L_.col(j);
	    	Xsig_aug_.col(j + 1 + TRA_N_AUG) 	= x_aug_ - sqrt(TRA_LAMBDA + TRA_N_AUG) * L_.col(j);
	  	}
		//std::cout << "Generated sigma points " << std::endl << Xsig_aug_ << std::endl << std::endl;

	  	
	  	// 2. Predict sigma points
	  	for (int j = 0; j< 2 * TRA_N_AUG + 1; j++){

		    //extract values for better readability
		    double p_x = Xsig_aug_(0,j);
		    double p_y = Xsig_aug_(1,j);
		    double v = Xsig_aug_(2,j);
		    double yaw = Xsig_aug_(3,j);
		    double yawd = Xsig_aug_(4,j);
		    double nu_a = Xsig_aug_(5,j);
		    double nu_yawdd = Xsig_aug_(6,j);

		    //predicted state values
		    double px_p, py_p;

		    //avoid division by zero
		    if (fabs(yawd) > 0.001) {
		        px_p = p_x + v/yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
		        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
		    }
		    else {
		        px_p = p_x + v * delta_t * cos(yaw);
		        py_p = p_y + v * delta_t * sin(yaw);
		    }

		    double v_p = v;
		    double yaw_p = yaw + yawd * delta_t;
		    double yawd_p = yawd;

		    //add noise
		    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		    v_p = v_p + nu_a * delta_t;

		    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		    yawd_p = yawd_p + nu_yawdd * delta_t;

		    //write predicted sigma point into right column
		    Xsig_pred_(0,j) = px_p;
		    Xsig_pred_(1,j) = py_p;
		    Xsig_pred_(2,j) = v_p;
		    Xsig_pred_(3,j) = yaw_p;
		    Xsig_pred_(4,j) = yawd_p;
	  	}
	  	//std::cout << "Predicted sigma points " << std::endl << Xsig_pred_ << std::endl << std::endl;
	  	

		// 3. Predict mean and covariance
		// Predicted state mean
		track.x.fill(0.0);
		for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //iterate over sigma points
		    track.x = track.x + weights_(j) * Xsig_pred_.col(j);
		}

		//predicted state covariance matrix
		track.P.fill(0.0);
		for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //iterate over sigma points

		    // state difference
		    VectorXd x_diff = Xsig_pred_.col(j) - track.x;

		    //angle normalization
		    while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
		    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

		    track.P = track.P + weights_(j) * x_diff * x_diff.transpose() ;
		}
	  	
	  	// Print prediction
	  	std::cout << "Prediction of track " << i << std::endl;
	  	std::cout << "x = " << std::endl << track.x << std::endl;
	  	std::cout << "P = " << std::endl << track.P << std::endl;
	  	std::cout << std::endl << std::endl;
	}
}

void Tracking::Update(const std::vector<Cluster> & detected_clusters){

}

visualization_msgs::MarkerArray & Tracking::showTracks(){

}


std::vector<Track> & Tracking::getTracks(){
	return tracks_;
}
