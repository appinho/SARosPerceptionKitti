#include "../include/test_kitti/tracking.h"

Tracking::Tracking(){

	is_initialized_ = false;

	x_aug_ = VectorXd(TRA_N_AUG);
	P_aug_ = MatrixXd(TRA_N_AUG, TRA_N_AUG);
	Xsig_aug_ = MatrixXd(TRA_N_AUG, 2 * TRA_N_AUG + 1);


	z_pred_ = VectorXd(TRA_Z_LASER);
	//innovation covariance matrix S
	S_ = MatrixXd(TRA_Z_LASER, TRA_Z_LASER);
	//create matrix for cross correlation Tc
	Tc_ = MatrixXd(TRA_N_X, TRA_Z_LASER);

	R_laser_ = MatrixXd(TRA_Z_LASER, TRA_Z_LASER);
  	R_laser_ << TRA_STD_LASPX * TRA_STD_LASPX, 0,
          		0, TRA_STD_LASPY * TRA_STD_LASPY;

	// Init weights
	weights_ = VectorXd(2 * TRA_N_AUG + 1);
	// Set weights
	weights_(0) = TRA_LAMBDA / (TRA_LAMBDA + TRA_N_AUG);
	for (int i = 1; i < 2 * TRA_N_AUG + 1; i++) {  //2n+1 weights
		weights_(i) = 0.5 / (TRA_N_AUG + TRA_LAMBDA);
	}

	//std::cout << "weights = " << std::endl << weights_ << std::endl;

	// Initialize visualization for detected bounding boxes
    marker_array_ = visualization_msgs::MarkerArray();
    for(int i = 0; i < TRA_BUFFER_SIZE; ++i){

    	// Fill in marker information
    	visualization_msgs::Marker marker;
    	marker.header.frame_id = "base_link";
    	marker.header.stamp = ros::Time();
    	marker.ns = "my_namespace";
    	marker.id = i;
    	marker.text = "Object";
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	   	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	    marker.frame_locked = true;

	    // Display box with certain color and alpha
	    marker.color.a = 0.5;
	    marker.color.r = 0.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;

	    // Push back marker to marker array
    	marker_array_.markers.push_back(marker);
    }
}

Tracking::~Tracking(){
	
}

void Tracking::processMeasurements(const std::vector<Cluster> & detected_clusters, const double time_stamp){

	// All other frames
	if(is_initialized_){

		// Calculate time difference between frames
		double delta_t = time_stamp - last_time_stamp_;

		// Prints
		ROS_INFO("Delta T [%f]", delta_t);


		// Prediction
		Prediction(delta_t);

		// DataAssociation
		DataAssociation(detected_clusters);

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
	ROS_INFO("Number of tracks [%d]", int(tracks_.size()));

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
		    track.Xsig_pred(0,j) = px_p;
		    track.Xsig_pred(1,j) = py_p;
		    track.Xsig_pred(2,j) = v_p;
		    track.Xsig_pred(3,j) = yaw_p;
		    track.Xsig_pred(4,j) = yawd_p;
	  	}
	  	//std::cout << "Predicted sigma points " << std::endl << track.Xsig_pred << std::endl << std::endl;
	  	

		// 3. Predict mean and covariance
		// Predicted state mean
		track.x.fill(0.0);
		for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //iterate over sigma points
		    track.x = track.x + weights_(j) * track.Xsig_pred.col(j);
		}

		//predicted state covariance matrix
		track.P.fill(0.0);
		for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //iterate over sigma points

		    // state difference
		    VectorXd x_diff = track.Xsig_pred.col(j) - track.x;

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

void Tracking::DataAssociation(const std::vector<Cluster> & detected_clusters){

	data_association_.clear();
	data_association_ = std::vector<int>(tracks_.size(),-1);

	// Define help variables
	float dist;
	// Loop through tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Set minimum distance
		float min_dist = DA_NN_MAX_DIST;

		// Loop through detected objects
		for(int j = 0; j < detected_clusters.size(); ++j){

			// Calculate distance between track and detected object
			dist = CalculateDistance(tracks_[i], detected_clusters[j]);
			//std::cout << i << " " << j << " " << dist << std::endl;

			// If new minimum distance has been found update it and save object index
			if(dist < min_dist){
				min_dist = dist;
				data_association_[i] = j;
			}
		}
	}

}

float Tracking::CalculateDistance(const Track & track, const Cluster detected_cluster){
	return sqrt( pow(track.x(0) - detected_cluster.x,2) + pow(track.x(1) - detected_cluster.y,2) );
}

void Tracking::Update(const std::vector<Cluster> & detected_clusters){

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Check if data association has found measurement for track i
		if(data_association_[i] == -1){

			std::cout << "No lidar measurement found for track " << i << std::endl;
		}
		else{

			// Grab track and measurement
			Track & track = tracks_[i];
			VectorXd z = VectorXd(TRA_Z_LASER);
			z << detected_clusters[ data_association_[i] ].x, detected_clusters[ data_association_[i] ].y;

			// Prints
			//std::cout << "Lidar measurement found for track " << i << std::endl 
  			//   << z << std::endl;

  			// 1. Predict measurement
	  		// Init measurement sigma points
			Zsig_ = track.Xsig_pred.topLeftCorner(TRA_Z_LASER,2 * TRA_N_AUG + 1);

			//mean predicted measurement
			z_pred_.fill(0.0);
			for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {
			    z_pred_ = z_pred_ + weights_(j) * Zsig_.col(j);
			}

			S_.fill(0.0);
			Tc_.fill(0.0);
			for (int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //2n+1 simga points

			    //residual
			    VectorXd z_sig_diff = Zsig_.col(j) - z_pred_;

			    //angle normalization
			    while (z_sig_diff(1) >  M_PI) z_sig_diff(1) -= 2. * M_PI;
			    while (z_sig_diff(1) < -M_PI) z_sig_diff(1) += 2. * M_PI;

			    S_ = S_ + weights_(j) * z_sig_diff * z_sig_diff.transpose();

			    // state difference
			    VectorXd x_diff = track.Xsig_pred.col(j) - track.x;
			    //angle normalization
			    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
			    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

			    Tc_ = Tc_ + weights_(j) * x_diff * z_sig_diff.transpose();
			}

			//add measurement noise covariance matrix
			S_ = S_ + R_laser_;

			// 2. State update

			//Kalman gain K;
			MatrixXd K = Tc_ * S_.inverse();

			//residual
			VectorXd z_diff = z - z_pred_;

			//angle normalization
			while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
			while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

			// Print
			/*
			std::cout << "Laser Measurement" << std::endl;
			std::cout << "z_pred " << std::endl << z_pred_ << std::endl;
			std::cout << "z " << std::endl << z << std::endl;
			std::cout << "z_diff " << std::endl << z_diff << std::endl;
			std::cout << "S " << std::endl << S_ << std::endl;
			std::cout << "K " << std::endl << K << std::endl;
			*/

			//update state mean and covariance matrix
			track.x = track.x + K * z_diff;
			track.P = track.P - K * S_ * K.transpose();
			VectorXd pos = VectorXd::Zero(2);
			pos << track.x(0), track.x(1);
			track.hist_pos.push_back(pos);

			// Print prediction
			std::cout << "Laser Update" << std::endl;
			std::cout << "x = " << std::endl << track.x << std::endl;
			std::cout << "P = " << std::endl << track.P << std::endl;
			//std::cout << "eps = " << std::endl << CalculateNIS(z_diff,S) << std::endl;
			std::cout << std::endl << std::endl;
		}
	}
}

visualization_msgs::MarkerArray & Tracking::showTracks(){

	// Loop through tracks
  	for(int i = 0; i < tracks_.size(); ++i){

		// Fill in current position, orientation
	    marker_array_.markers[i].pose.position.x = tracks_[i].x(0);
	    marker_array_.markers[i].pose.position.y = tracks_[i].x(1);
	    marker_array_.markers[i].pose.position.z = 1.0;
		// Fill in current dimension
		marker_array_.markers[i].scale.x = 0.1;
		marker_array_.markers[i].scale.y = 0.1;
		marker_array_.markers[i].scale.z = 2.0;
	}
	// Loop through remaining buffer size
	for(int i = tracks_.size(); i < TRA_BUFFER_SIZE; ++i){

		// Fill in current position, orientation
	    marker_array_.markers[i].pose.position.x = 0.0;
	    marker_array_.markers[i].pose.position.y = 0.0;
	    marker_array_.markers[i].pose.position.z = 1.0;
		// Fill in current dimension
		marker_array_.markers[i].scale.x = 0.1;
		marker_array_.markers[i].scale.y = 0.1;
		marker_array_.markers[i].scale.z = 2.0;
  	}

  	return marker_array_;
}


std::vector<Track> & Tracking::getTracks(){
	return tracks_;
}
