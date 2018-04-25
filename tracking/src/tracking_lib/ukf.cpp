/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <tracking_lib/ukf.h>

namespace tracking{

/******************************************************************************/

UnscentedKF::UnscentedKF(ros::NodeHandle nh, ros::NodeHandle private_nh):
	nh_(nh),
	private_nh_(private_nh)
	{

	// Define parameters
	private_nh_.param("data_association/ped/dist/position",
		params_.da_ped_dist_pos, params_.da_ped_dist_pos);
	private_nh_.param("data_association/ped/dist/form",
		params_.da_ped_dist_form, params_.da_ped_dist_form);
	private_nh_.param("data_association/car/dist/position",
		params_.da_car_dist_pos, params_.da_car_dist_pos);
	private_nh_.param("data_association/car/dist/form",
		params_.da_car_dist_form, params_.da_car_dist_form);

	private_nh_.param("tracking/dim/z", params_.tra_dim_z,
		params_.tra_dim_z);
	private_nh_.param("tracking/dim/x", params_.tra_dim_x,
		params_.tra_dim_x);
	private_nh_.param("tracking/dim/x_aug", params_.tra_dim_x_aug,
		params_.tra_dim_x_aug);

	private_nh_.param("tracking/std/lidar/x", params_.tra_std_lidar_x,
		params_.tra_std_lidar_x);
	private_nh_.param("tracking/std/lidar/y", params_.tra_std_lidar_y,
		params_.tra_std_lidar_y);
	private_nh_.param("tracking/std/acc", params_.tra_std_acc,
		params_.tra_std_acc);
	private_nh_.param("tracking/std/yaw_rate", params_.tra_std_yaw_rate,
		params_.tra_std_yaw_rate);
	private_nh_.param("tracking/lambda", params_.tra_lambda,
		params_.tra_lambda);
	private_nh_.param("tracking/aging/bad", params_.tra_aging_bad,
		params_.tra_aging_bad);

	// Print parameters
	ROS_INFO_STREAM("da_ped_dist_pos " << params_.da_ped_dist_pos);
	ROS_INFO_STREAM("da_ped_dist_form " << params_.da_ped_dist_form);
	ROS_INFO_STREAM("da_car_dist_pos " << params_.da_car_dist_pos);
	ROS_INFO_STREAM("da_car_dist_form " << params_.da_car_dist_form);
	ROS_INFO_STREAM("tra_dim_z " << params_.tra_dim_z);
	ROS_INFO_STREAM("tra_dim_x " << params_.tra_dim_x);
	ROS_INFO_STREAM("tra_dim_x_aug " << params_.tra_dim_x_aug);
	ROS_INFO_STREAM("tra_std_lidar_x " << params_.tra_std_lidar_x);
	ROS_INFO_STREAM("tra_std_lidar_y " << params_.tra_std_lidar_y);
	ROS_INFO_STREAM("tra_std_acc " << params_.tra_std_acc);
	ROS_INFO_STREAM("tra_std_yaw_rate " << params_.tra_std_yaw_rate);
	ROS_INFO_STREAM("tra_lambda " << params_.tra_lambda);
	ROS_INFO_STREAM("tra_aging_bad " << params_.tra_aging_bad);

	// Set initialized to false at the beginning
	is_initialized_ = false;

	// Measurement covariance
	R_laser_ = MatrixXd(params_.tra_dim_z, params_.tra_dim_z);
  	R_laser_ << params_.tra_std_lidar_x * params_.tra_std_lidar_x, 0,
		0, params_.tra_std_lidar_y * params_.tra_std_lidar_y;

	// Define weights for UKF
	weights_ = VectorXd(2 * params_.tra_dim_x_aug + 1);
	weights_(0) = params_.tra_lambda /
		(params_.tra_lambda + params_.tra_dim_x_aug);
	for (int i = 1; i < 2 * params_.tra_dim_x_aug + 1; i++) {
		weights_(i) = 0.5 / (params_.tra_dim_x_aug + params_.tra_lambda);
	}

	// Start ids for track with 0
	track_id_counter_ = 0;

	// Define Subscriber
	list_detected_objects_sub_ = nh.subscribe("/detection/objects", 2,
		&UnscentedKF::process, this);

	// Define Publisher
	list_tracked_objects_pub_ = nh_.advertise<ObjectArray>(
		"/tracking/objects", 2);

	// Random color for track
	rng_(2345);
		
	// Init counter for publishing
	time_frame_ = 0;
}

UnscentedKF::~UnscentedKF(){

}

void UnscentedKF::process(const ObjectArrayConstPtr & detected_objects){

	// Read current time
	double time_stamp = detected_objects->header.stamp.toSec();

	// All other frames
	if(is_initialized_){

		// Calculate time difference between frames
		double delta_t = time_stamp - last_time_stamp_;

		// Prediction
		Prediction(delta_t);

		// Data association
		//DataAssociation(detected_objects);

		// Update
		Update(detected_objects);

		// Track management
		TrackManagement(detected_objects);

	}
	// First frame
	else{

		// Initialize tracks
		for(int i = 0; i < detected_objects->list.size(); ++i){
			initTrack(detected_objects->list[i]);
		}

		// Set initialized to true
		is_initialized_ = true;
	}

	// Store time stamp for next frame
	last_time_stamp_ = time_stamp;

	// Print Tracks
	printTracks();

	// Publish and print
	publishTracks(detected_objects->header);

	// Increment time frame
	time_frame_++;
}
void UnscentedKF::Prediction(const double delta_t){

	/*
	// Buffer variables
	VectorXd x_aug = VectorXd(TRA_N_AUG);
	MatrixXd P_aug = MatrixXd(TRA_N_AUG, TRA_N_AUG);
	MatrixXd Xsig_aug = MatrixXd(TRA_N_AUG, 2 * TRA_N_AUG + 1);

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

		// 1. Generate augmented sigma points
		// Fill augmented mean state
		x_aug.head(5) = track.sta.x;
		x_aug(5) = 0;
		x_aug(6) = 0;

		// Fill augmented covariance matrix
		P_aug.fill(0.0);
		P_aug.topLeftCorner(5,5) = track.sta.P;
		P_aug(5,5) = TRA_STD_A * TRA_STD_A;
		P_aug(6,6) = TRA_STD_YAWDD * TRA_STD_YAWDD;

		// Create square root matrix
		MatrixXd L = P_aug.llt().matrixL();

		// Create augmented sigma points
		Xsig_aug.col(0)  = x_aug;
		for(int j = 0; j < TRA_N_AUG; j++){
			Xsig_aug.col(j + 1) = 
				x_aug + sqrt(TRA_LAMBDA + TRA_N_AUG) * L.col(j);
			Xsig_aug.col(j + 1 + TRA_N_AUG) = 
				x_aug - sqrt(TRA_LAMBDA + TRA_N_AUG) * L.col(j);
		}
		//std::cout << "Generated sigma points " << std::endl << Xsig_aug
		//	<< std::endl << std::endl;

		// 2. Predict sigma points
		for(int j = 0; j < 2 * TRA_N_AUG + 1; j++){

			// Extract values for better readability
			double p_x = Xsig_aug(0,j);
			double p_y = Xsig_aug(1,j);
			double v = Xsig_aug(2,j);
			double yaw = Xsig_aug(3,j);
			double yawd = Xsig_aug(4,j);
			double nu_a = Xsig_aug(5,j);
			double nu_yawdd = Xsig_aug(6,j);

			// Predicted state values
			double px_p, py_p;

			// Avoid division by zero
			if(fabs(yawd) > 0.001){
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

			// Add noise
			px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
			py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
			v_p = v_p + nu_a * delta_t;
			yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
			yawd_p = yawd_p + nu_yawdd * delta_t;

			// Write predicted sigma point into right column
			track.sta.Xsig_pred(0,j) = px_p;
			track.sta.Xsig_pred(1,j) = py_p;
			track.sta.Xsig_pred(2,j) = v_p;
			track.sta.Xsig_pred(3,j) = yaw_p;
			track.sta.Xsig_pred(4,j) = yawd_p;
		}
		//std::cout << "Predicted sigma points " << std::endl << track.sta.Xsig_pred
		//	<< std::endl << std::endl;

		// 3. Predict mean and covariance
		// Predicted state mean
		track.sta.x.fill(0.0);
		for(int j = 0; j < 2 * TRA_N_AUG + 1; j++) {
			track.sta.x = track.sta.x + weights_(j) * track.sta.Xsig_pred.col(j);
		}

		//predicted state covariance matrix
		track.sta.P.fill(0.0);
		//iterate over sigma points
		for(int j = 0; j < 2 * TRA_N_AUG + 1; j++) {

			// state difference
			VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

			//angle normalization
			while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
			while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

			track.sta.P = track.sta.P + weights_(j) * x_diff * x_diff.transpose() ;
		}

		// Print prediction
		//std::cout << "Prediction of track " << i << std::endl;
		//std::cout << "x = " << std::endl << track.sta.x << std::endl;
		//std::cout << "P = " << std::endl << track.sta.P << std::endl;
		//std::cout << std::endl << std::endl;
	}
	*/
}

void UnscentedKF::Update(const ObjectArrayConstPtr & detected_objects){

	/*
	// Buffer variables
	VectorXd z = VectorXd(TRA_Z_LASER);
	MatrixXd Zsig;
	VectorXd z_pred = VectorXd(TRA_Z_LASER);
	MatrixXd S = MatrixXd(TRA_Z_LASER, TRA_Z_LASER);
	MatrixXd Tc = MatrixXd(TRA_N_X, TRA_Z_LASER);

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

		// Check if data association has found measurement for track i
		if(track_association_[i] == -1){

			// Update History
			track.hist.bad_age++;

			// Print
			//std::cout << "No measurement found for track " << track.id
			//	<< std::endl;
		}
		else{

			// Grab measurement
			z << detected_objects->list[ track_association_[i] ].world_pose.point.x, 
				 detected_objects->list[ track_association_[i] ].world_pose.point.y;

			// Prints
			//std::cout << "Lidar measurement found for track " << i 
			//	<< std::endl  << z << std::endl;

			// 1. Predict measurement
			// Init measurement sigma points
			Zsig = track.sta.Xsig_pred.topLeftCorner(TRA_Z_LASER,2 * TRA_N_AUG + 1);

			//std::cout << "Zsig " << Zsig << std::endl;

			// Mean predicted measurement
			z_pred.fill(0.0);
			for(int j = 0; j < 2 * TRA_N_AUG + 1; j++) {
				z_pred = z_pred + weights_(j) * Zsig.col(j);
			}

			//std::cout << "Z pred " << z_pred << std::endl;

			S.fill(0.0);
			Tc.fill(0.0);
			for(int j = 0; j < 2 * TRA_N_AUG + 1; j++) {  //2n+1 simga points

				// Residual
				VectorXd z_sig_diff = Zsig.col(j) - z_pred;

				// Angle normalization
				//while(z_sig_diff(1) >  M_PI) z_sig_diff(1) -= 2. * M_PI;
				//while(z_sig_diff(1) < -M_PI) z_sig_diff(1) += 2. * M_PI;

				S = S + weights_(j) * z_sig_diff * z_sig_diff.transpose();

				// State difference
				VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

				//std::cout << j << " " << weights_(j) << " "
				//	<< x_diff << " " << z_sig_diff.transpose() << std::endl;
				// Angle normalization
				while(x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
				while(x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

				Tc = Tc + weights_(j) * x_diff * z_sig_diff.transpose();
			}

			//add measurement noise covariance matrix
			S = S + R_laser_;

			// 2. State update
			//Kalman gain K;
			MatrixXd K = Tc * S.inverse();

			//residual
			VectorXd z_diff = z - z_pred;

			// Print
			//std::cout << "Laser Measurement" << std::endl;
			//std::cout << "z_pred " << std::endl << z_pred << std::endl;
			//std::cout << "z " << std::endl << z << std::endl;
			//std::cout << "z_diff " << std::endl << z_diff << std::endl;

			//std::cout << "S " << std::endl << S << std::endl;
			//std::cout << "Tc " << std::endl << Tc << std::endl;
			//std::cout << "S_inv " << std::endl << S.inverse() << std::endl;
			//std::cout << "K " << std::endl << K << std::endl;
			//std::cout << "x diff " << std::endl << K * z_diff << std::endl;

			//update state mean and covariance matrix
			track.sta.x = track.sta.x + K * z_diff;
			track.sta.P = track.sta.P - K * S * K.transpose();

			//VectorXd pos = VectorXd::Zero(2);
			//pos << track.sta.x(0), track.sta.x(1);
			//track.hist_pos.push_back(pos);

			// Update History
			track.hist.good_age++;
			track.hist.bad_age = 0;

			// Update geometry
			if(detected_objects->list[ track_association_[i] ].height <= TRA_MIN_OBJ_HEIGHT){
				track.geo.h = TRA_MIN_OBJ_HEIGHT;
				std::cout << "WARN: Car seems to be too short and is set from "
					<< detected_objects->list[ track_association_[i] ].height << " to "
					<< TRA_MIN_OBJ_HEIGHT << std::endl;
			}
			else{
				track.geo.h = detected_objects->list[ track_association_[i] ].height;
			}
			float detected_area = detected_objects->list[ track_association_[i] ].length *
				detected_objects->list[ track_association_[i] ].width;
			float tracked_area = track.geo.l * track.geo.w;
			float occluded_factor = 2.0;
			if(occluded_factor * detected_area < tracked_area){
				ROS_WARN("Track [%d] probably occluded because of dropping size from [%f] to [%f]",
					track.id, tracked_area, detected_area);
			}
			else{
				track.geo.l = detected_objects->list[ track_association_[i] ].length;
				track.geo.w = detected_objects->list[ track_association_[i] ].width;
				track.geo.o = detected_objects->list[ track_association_[i] ].orientation;
				track.sta.z = detected_objects->list[ track_association_[i] ].world_pose.point.z;
			}


			// Print prediction
			std::cout << "Laser Update [" << track.hist.good_age << "," 
				<< track.hist.bad_age
				<< "] of track " << track.id << " with ["
				<< z_diff[0] << " , " << z_diff[1] << "]" << std::endl;
			//std::cout << "P = " << std::endl << track.sta.P << std::endl;
			//std::cout << "eps = " << std::endl << CalculateNIS(z_diff,S)
			//	<< std::endl;
			//std::cout << std::endl << std::endl;
		}
	}
	*/
}

void UnscentedKF::TrackManagement(const ObjectArrayConstPtr & detected_objects){

	/*
	// Delete spuriors tracks
	for(int i = 0; i < tracks_.size() ; ++i){

		// Deletion condition
		if(tracks_[i].hist.bad_age >= TRA_BAD_AGE){

			std::cout << "Deletion of track " << tracks_[i].id << std::endl;

			// Swap track with end of vector and pop back
			std::swap(tracks_[i],tracks_.back());
			tracks_.pop_back();
		}
	}

	// Create new ones out of untracked new detected object hypothesis
	// Initialize tracks
	for(int i = 0; i < detected_objects->list.size(); ++i){

		// Unassigned object condition
		if(object_association_[i] == -1){

			// Init new track
			initTrack(detected_objects->list[i]);
		}
	}
	*/
}

void UnscentedKF::initTrack(const Object & obj){

	// Create new track
	Track tr = Track();

	// Add id and increment
	tr.id = track_id_counter_;
	track_id_counter_++;

	// Add state information
	tr.sta.x = VectorXd::Zero(params_.tra_dim_x);
	tr.sta.x[0] = obj.world_pose.point.x;
	tr.sta.x[1] = obj.world_pose.point.y;
	tr.sta.z = obj.world_pose.point.z;
	tr.sta.P = MatrixXd::Zero(params_.tra_dim_x, params_.tra_dim_x);
	tr.sta.P <<  	1,  0,  0,  0,  0,
					0,  1,  0,  0,  0,
					0,  0, 10,  0,  0,
					0,  0,  0,  1,  0,
					0,  0,  0,  0,  1;
	tr.sta.Xsig_pred = MatrixXd::Zero(params_.tra_dim_x, 
		2 * params_.tra_dim_x_aug + 1);

	// Add geometric information
	tr.geo.width = obj.width;
	tr.geo.length = obj.length;
	tr.geo.height = obj.height;
	tr.geo.orientation = obj.orientation;

	// Add semantic information
	tr.sem.name = obj.semantic_name;
	tr.sem.id = obj.semantic_id;
	tr.sem.confidence = obj.semantic_confidence;

	// Add unique color
	tr.r = rng_.uniform(0, 255);
	tr.g = rng_.uniform(0, 255);
	tr.b = rng_.uniform(0, 255);
	
	// Push back to track list
	tracks_.push_back(tr);
}

void UnscentedKF::publishTracks(const std_msgs::Header & header){

	// Create track message
	ObjectArray track_list;
	track_list.header = header;

	// Loop over all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

		// Create new message and fill it
		Object track_msg;
		track_msg.id = track.id;
		track_msg.world_pose.header.frame_id = "world";
		track_msg.world_pose.point.x = track.sta.x[0];
		track_msg.world_pose.point.y = track.sta.x[1];
		track_msg.world_pose.point.z = track.sta.z;

		try{
			listener_.transformPoint("camera_color_left",
				track_msg.world_pose,
				track_msg.cam_pose);
			listener_.transformPoint("velo_link",
				track_msg.world_pose,
				track_msg.velo_pose);
		}
		catch(tf::TransformException& ex){
			ROS_ERROR("Received an exception trying to transform a point from"
				"\"velo_link\" to \"world\": %s", ex.what());
		}
		track_msg.heading = track.sta.x[3];
		track_msg.velocity = track.sta.x[2];
		track_msg.width = track.geo.width;
		track_msg.length = track.geo.length;
		track_msg.height = track.geo.height;
		track_msg.orientation = track.geo.orientation;
		track_msg.semantic_name = track.sem.name;
		track_msg.semantic_id = track.sem.id;
		track_msg.semantic_confidence = track.sem.confidence;
		track_msg.r = track.r;
		track_msg.g = track.g;
		track_msg.b = track.b;
		track_msg.is_track = true;

		// Push back track message
		track_list.list.push_back(track_msg);
	}

	// Print
	ROS_INFO("Publishing Tracking [%d]: # Tracks [%d]", time_frame_,
		int(tracks_.size()));

	// Publish
	list_tracked_objects_pub_.publish(track_list);
}

void UnscentedKF::printTrack(const Track & tr){

	ROS_INFO("Track [%d] is [%s] with to [%f],"
	" pos[x,y,z] [%f,%f,%f]"
	" form[w,l,h,o] [%f,%f,%f,%f]",
	tr.id, tr.sem.name.c_str(), tr.sem.confidence,
	tr.sta.x[0], tr.sta.x[1], tr.sta.z,
	tr.geo.width, tr.geo.length,
	tr.geo.height, tr.geo.orientation);
}

void UnscentedKF::printTracks(){

	for(int i = 0; i < tracks_.size(); ++i){
		printTrack(tracks_[i]);
	}
}

} // namespace tracking