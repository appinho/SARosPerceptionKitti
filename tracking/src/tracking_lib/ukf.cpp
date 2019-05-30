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
	private_nh_.param("tracking/occlusion_factor", params_.tra_occ_factor, 
		params_.tra_occ_factor);
	private_nh_.param("track/P_init/x", params_.p_init_x,
		params_.p_init_x);
	private_nh_.param("track/P_init/y", params_.p_init_y,
		params_.p_init_y);
	private_nh_.param("track/P_init/v", params_.p_init_v,
		params_.p_init_v);
	private_nh_.param("track/P_init/yaw", params_.p_init_yaw,
		params_.p_init_yaw);
	private_nh_.param("track/P_init/yaw_rate", params_.p_init_yaw_rate, 
		params_.p_init_yaw_rate);

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
	ROS_INFO_STREAM("tra_occ_factor " << params_.tra_occ_factor);
	ROS_INFO_STREAM("p_init_x " << params_.p_init_x);
	ROS_INFO_STREAM("p_init_y " << params_.p_init_y);
	ROS_INFO_STREAM("p_init_v " << params_.p_init_v);
	ROS_INFO_STREAM("p_init_yaw " << params_.p_init_yaw);
	ROS_INFO_STREAM("p_init_yaw_rate " << params_.p_init_yaw_rate);

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
	rng_(2);
		
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
		GlobalNearestNeighbor(detected_objects);

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

	// Buffer variables
	VectorXd x_aug = VectorXd(params_.tra_dim_x_aug);
	MatrixXd P_aug = MatrixXd(params_.tra_dim_x_aug, params_.tra_dim_x_aug);
	MatrixXd Xsig_aug = 
		MatrixXd(params_.tra_dim_x_aug, 2 * params_.tra_dim_x_aug + 1);

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

/******************************************************************************
 * 1. Generate augmented sigma points
 */

		// Fill augmented mean state
		x_aug.head(5) = track.sta.x;
		x_aug(5) = 0;
		x_aug(6) = 0;

		// Fill augmented covariance matrix
		P_aug.fill(0.0);
		P_aug.topLeftCorner(5,5) = track.sta.P;
		P_aug(5,5) = params_.tra_std_acc * params_.tra_std_acc;
		P_aug(6,6) = params_.tra_std_yaw_rate * params_.tra_std_yaw_rate;

		// Create square root matrix
		MatrixXd L = P_aug.llt().matrixL();

		// Create augmented sigma points
		Xsig_aug.col(0)  = x_aug;
		for(int j = 0; j < params_.tra_dim_x_aug; j++){
			Xsig_aug.col(j + 1) = x_aug + 
				sqrt(params_.tra_lambda + params_.tra_dim_x_aug) * L.col(j);
			Xsig_aug.col(j + 1 + params_.tra_dim_x_aug) = x_aug - 
				sqrt(params_.tra_lambda + params_.tra_dim_x_aug) * L.col(j);
		}

/******************************************************************************
 * 2. Predict sigma points
 */

		for(int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++){

			// Grab values for better readability
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

/******************************************************************************
 * 3. Predict state vector and state covariance
 */
		// Predicted state mean
		track.sta.x.fill(0.0);
		for(int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++) {
			track.sta.x = track.sta.x + weights_(j) *
				track.sta.Xsig_pred.col(j);
		}

		// Predicted state covariance matrix
		track.sta.P.fill(0.0);

		// Iterate over sigma points
		for(int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++) {

			// State difference
			VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

			// Angle normalization
			while (x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
			while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

			track.sta.P = track.sta.P + weights_(j) * x_diff * 
				x_diff.transpose() ;
		}

		/*
		// Print prediction
		ROS_INFO("Pred of T[%d] xp=[%f,%f,%f,%f,%f], Pp=[%f,%f,%f,%f,%f]",
			track.id, track.sta.x(0), track.sta.x(1), track.sta.x(2), 
			track.sta.x(3), track.sta.x(4),	track.sta.P(0), track.sta.P(6), 
			track.sta.P(12), track.sta.P(18), track.sta.P(24)
		);
		*/
	}
}

void UnscentedKF::GlobalNearestNeighbor(
	const ObjectArrayConstPtr & detected_objects){

	// Define assoication vectors
	da_tracks = std::vector<int>(tracks_.size(),-1);
	da_objects = std::vector<int>(detected_objects->list.size(),-1);

	// Loop through tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Buffer variables
		std::vector<float> distances;
		std::vector<int> matches;

		// Loop through detected objects
		for(int j = 0; j < detected_objects->list.size(); ++j){

			// Calculate distance between track and detected object
			if(tracks_[i].sem.id == detected_objects->list[j].semantic_id){
				float dist = CalculateDistance(tracks_[i], 
					detected_objects->list[j]);

				if(dist < tracks_[i].sem.pos_gate){
					distances.push_back(dist);
					matches.push_back(j);
				}
			}
		}

		// If track exactly finds one match assign it
		if(matches.size() == 1){

			float box_dist = CalculateEuclideanAndBoxOffset(tracks_[i], 
				detected_objects->list[matches[0]]);
			if(box_dist < tracks_[i].sem.com_gate){
				da_tracks[i] = matches[0];
				da_objects[matches[0]] = i;
			}
		}
		// If found more then take best match and block other measurements
		else if(matches.size() > 1){

			// Block other measurements to NOT be initialized
			ROS_WARN("Multiple associations for track [%d]", tracks_[i].id);

			// Calculate all box distances and find minimum
			float min_box_dist = tracks_[i].sem.com_gate;
			int min_box_index = -1;

			for(int k = 0; k < matches.size(); ++k){

				float box_dist = CalculateEuclideanAndBoxOffset(tracks_[i],
					detected_objects->list[matches[k]]);
				if(box_dist < min_box_dist){
					min_box_index = k;
					min_box_dist = box_dist;
				}
			}

			for(int k = 0; k < matches.size(); ++k){
				if(k == min_box_index){
					da_objects[matches[k]] = i;
					da_tracks[i] = matches[k];
				}
				else{
					da_objects[matches[k]] = -2;
				}
			}
		}
		else{
			ROS_WARN("No measurement found for track [%d]", tracks_[i].id);
		}
	}
}

float UnscentedKF::CalculateDistance(const Track & track,
	const Object & object){

	// Calculate euclidean distance in x,y,z coordinates of track and object
	return fabs(track.sta.x(0) - object.world_pose.point.x) +
		fabs(track.sta.x(1) - object.world_pose.point.y) +
		fabs(track.sta.z - object.world_pose.point.z);
}

float UnscentedKF::CalculateBoxMismatch(const Track & track,
	const Object & object){

	// Calculate mismatch of both tracked cube and detected cube
	float box_wl_switched =  fabs(track.geo.width - object.length) +
		fabs(track.geo.length - object.width);
	float box_wl_ordered = fabs(track.geo.width - object.width) +
		fabs(track.geo.length - object.length);
	float box_mismatch = (box_wl_switched < box_wl_ordered) ? 
		box_wl_switched : box_wl_ordered;
	box_mismatch += fabs(track.geo.height - object.height);
	return box_mismatch;
}

float UnscentedKF::CalculateEuclideanAndBoxOffset(const Track & track,
	const Object & object){

	// Sum of euclidean offset and box mismatch
	return CalculateDistance(track, object) + 
		CalculateBoxMismatch(track, object);
}

void UnscentedKF::Update(const ObjectArrayConstPtr & detected_objects){

	// Buffer variables
	VectorXd z = VectorXd(params_.tra_dim_z);
	MatrixXd Zsig;
	VectorXd z_pred = VectorXd(params_.tra_dim_z);
	MatrixXd S = MatrixXd(params_.tra_dim_z, params_.tra_dim_z);
	MatrixXd Tc = MatrixXd(params_.tra_dim_x, params_.tra_dim_z);

	// Loop through all tracks
	for(int i = 0; i < tracks_.size(); ++i){

		// Grab track
		Track & track = tracks_[i];

		// If track has not found any measurement
		if(da_tracks[i] == -1){

			// Increment bad aging
			track.hist.bad_age++;
		}
		// If track has found a measurement update it
		else{

			// Grab measurement
			z << detected_objects->list[ da_tracks[i] ].world_pose.point.x, 
				 detected_objects->list[ da_tracks[i] ].world_pose.point.y;

/******************************************************************************
 * 1. Predict measurement
 */
			// Init measurement sigma points
			Zsig = track.sta.Xsig_pred.topLeftCorner(params_.tra_dim_z, 
				2 * params_.tra_dim_x_aug + 1);

			// Mean predicted measurement
			z_pred.fill(0.0);
			for(int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++) {
				z_pred = z_pred + weights_(j) * Zsig.col(j);
			}

			S.fill(0.0);
			Tc.fill(0.0);
			for(int j = 0; j < 2 * params_.tra_dim_x_aug + 1; j++) {

				// Residual
				VectorXd z_sig_diff = Zsig.col(j) - z_pred;
				S = S + weights_(j) * z_sig_diff * z_sig_diff.transpose();

				// State difference
				VectorXd x_diff = track.sta.Xsig_pred.col(j) - track.sta.x;

				// Angle normalization
				while(x_diff(3) >  M_PI) x_diff(3) -= 2. * M_PI;
				while(x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

				Tc = Tc + weights_(j) * x_diff * z_sig_diff.transpose();
			}

			// Add measurement noise covariance matrix
			S = S + R_laser_;

/******************************************************************************
 * 2. Update state vector and covariance matrix
 */
			// Kalman gain K;
			MatrixXd K = Tc * S.inverse();

			// Residual
			VectorXd z_diff = z - z_pred;

			// Update state mean and covariance matrix
			track.sta.x = track.sta.x + K * z_diff;
			track.sta.P = track.sta.P - K * S * K.transpose();

			// Update History
			track.hist.good_age++;
			track.hist.bad_age = 0;

/******************************************************************************
 * 3. Update geometric information of track
 */
			// Calculate area of detection and track
			float det_area = 
				detected_objects->list[ da_tracks[i] ].length *
				detected_objects->list[ da_tracks[i] ].width;
			float tra_area = track.geo.length * track.geo.width;

			// If track became strongly smaller keep the shape
			if(params_.tra_occ_factor * det_area < tra_area){
				ROS_WARN("Track [%d] probably occluded because of dropping size"
					" from [%f] to [%f]", track.id, tra_area, det_area);
			}

			// Update geometric info
			setTrackShape(track, detected_objects->list[ da_tracks[i] ]);

			// Update  ground level
			// TODO: Lower Z
			track.sta.z = 
				detected_objects->list[ da_tracks[i] ].world_pose.point.z;

			/*
			// Print Update
			ROS_INFO("Update of T[%d] A[%d] z=[%f,%f] x=[%f,%f,%f,%f,%f],"
				" P=[%f,%f,%f,%f,%f]", track.id, track.hist.good_age,
				z_diff[0], z_diff[1],
				track.sta.x(0), track.sta.x(1), track.sta.x(2), 
				track.sta.x(3), track.sta.x(4),	
				track.sta.P(0), track.sta.P(6), track.sta.P(12), 
				track.sta.P(18), track.sta.P(24)
			);
			*/
		}
	}
}

void UnscentedKF::TrackManagement(const ObjectArrayConstPtr & detected_objects){

	// Delete spuriors tracks
	for(int i = 0; i < tracks_.size() ; ++i){

		// Deletion condition
		if(tracks_[i].hist.bad_age >= params_.tra_aging_bad){

			// Print
			ROS_INFO("Deletion of T [%d]", tracks_[i].id);

			// Swap track with end of vector and pop back
			std::swap(tracks_[i],tracks_.back());
			tracks_.pop_back();
		}
	}

	// TODO: Delete duplicated tracks

	// Create new ones out of untracked new detected object hypothesis
	// Initialize tracks
	for(int i = 0; i < detected_objects->list.size(); ++i){

		// Unassigned object condition
		if(da_objects[i] == -1){

			// Init new track
			initTrack(detected_objects->list[i]);
		}
	}
}

bool UnscentedKF::is_too_close_to_a_track(const Object & obj){

	// Check if other track exists in near surrounding
	for(const auto & track: tracks_){
		if(obj.semantic_id == track.sem.id){
			if(CalculateDistance(track, obj) < track.sem.box_gate){
				ROS_WARN("Not gonna init track because another one is close %f %f, %f %f",
					track.sta.x(0), track.sta.x(1), obj.world_pose.point.x, obj.world_pose.point.y);
				return true;
			}
		}
	}

	return false;
}

void UnscentedKF::initTrack(const Object & obj){

	// Only if object can be a track
	if(! obj.is_new_track)
		return;

	if(is_too_close_to_a_track(obj))
		return;

	// Create new track
	Track track = Track();

	// Add id and increment
	track.id = track_id_counter_;
	track_id_counter_++;

	// Add state information
	track.sta.x = VectorXd::Zero(params_.tra_dim_x);
	track.sta.x[0] = obj.world_pose.point.x;
	track.sta.x[1] = obj.world_pose.point.y;
	track.sta.z = obj.world_pose.point.z;
	track.sta.P = MatrixXd::Zero(params_.tra_dim_x, params_.tra_dim_x);
	track.sta.P << params_.p_init_x,  0,  0,  0,  0,
				0,  params_.p_init_y,  0,  0,  0,
				0,  0,	params_.p_init_v,  0,  0,
				0,  0,  0,params_.p_init_yaw,  0,
				0,  0,  0,  0,  params_.p_init_yaw_rate;
	track.sta.Xsig_pred = MatrixXd::Zero(params_.tra_dim_x, 
		2 * params_.tra_dim_x_aug + 1);

	// Add semantic information
	track.sem.name = obj.semantic_name;
	track.sem.id = obj.semantic_id;
	// Pedestrian
	if(obj.semantic_id == 11){
		track.sem.pos_gate = params_.da_ped_dist_pos;
		track.sem.box_gate = params_.da_ped_dist_form;
	}
	// Car
	else if(obj.semantic_id == 13){
		track.sem.pos_gate = params_.da_car_dist_pos;
		track.sem.box_gate = params_.da_car_dist_form;
	}
	track.sem.com_gate = track.sem.box_gate + track.sem.pos_gate;
	track.sem.confidence = obj.semantic_confidence;

	// Add geometric information
	setTrackShape(track, obj);

	// Add unique color
	track.r = rng_.uniform(0, 255);
	track.g = rng_.uniform(0, 255);
	track.b = rng_.uniform(0, 255);
	track.prob_existence = 0.6f;
	
	// Push back to track list
	tracks_.push_back(track);
}

void UnscentedKF::setTrackShape(Track & track, const Object & obj){

	// Update the form of the track with measurement
	track.geo.length = obj.length;
	track.geo.width = obj.width;
	track.geo.orientation = obj.orientation;

	// Find longer side of length and width and set it together with the height to a minimum
	float & max_side = (track.geo.length < track.geo.width) ? 
		track.geo.width : track.geo.length;
	if(track.sem.id == 11){
		track.geo.height = std::max(1.7f, obj.height);
		max_side = std::max(max_side, 0.5f);
	}
	else if(track.sem.id == 13){
		track.geo.height = std::max(1.3f, obj.height);
		max_side = std::max(max_side, 1.75f);
	}
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
		track_msg.a = track.prob_existence;

		// Push back track message
		track_list.list.push_back(track_msg);
	}

	// Print
	ROS_INFO("Publishing Tracking [%d]: # Tracks [%d]", time_frame_,
		int(tracks_.size()));

	// Publish
	list_tracked_objects_pub_.publish(track_list);
}

void UnscentedKF::printTrack(const Track & track){

	ROS_INFO("Track [%d] x=[%f,%f,%f,%f,%f], z=[%f]"
		" P=[%f,%f,%f,%f,%f] is [%s, %f] A[%d, %d] [w,l,h,o] [%f,%f,%f,%f]", 
		track.id,
		track.sta.x(0), track.sta.x(1), track.sta.x(2), track.sta.x(3), track.sta.x(4),
		track.sta.z,
		track.sta.P(0), track.sta.P(6), track.sta.P(12), track.sta.P(18), track.sta.P(24),
		track.sem.name.c_str(), track.sem.confidence,
		track.hist.good_age, track.hist.bad_age,
		track.geo.width, track.geo.length,
		track.geo.height, track.geo.orientation
	);
}

void UnscentedKF::printTracks(){

	for(int i = 0; i < tracks_.size(); ++i){
		printTrack(tracks_[i]);
	}
}

} // namespace tracking