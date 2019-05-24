/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

// Include guard
#ifndef unscentedkf_H
#define unscentedkf_H

// Includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <helper/ObjectArray.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

// Namespaces
namespace tracking{

using namespace helper;
using namespace Eigen;

struct Parameter{

	float da_ped_dist_pos;
	float da_ped_dist_form;
	float da_car_dist_pos;
	float da_car_dist_form;

	int tra_dim_z;
	int tra_dim_x;
	int tra_dim_x_aug;

	float tra_std_lidar_x;
	float tra_std_lidar_y;
	float tra_std_acc;
	float tra_std_yaw_rate;
	float tra_lambda;
	int tra_aging_bad;

	float tra_occ_factor;

	float p_init_x;
	float p_init_y;
	float p_init_v;
	float p_init_yaw;
	float p_init_yaw_rate;
};

struct History{

	int good_age;
	int bad_age;

};

struct Geometry{

	float width;
	float length;
	float height;
	float orientation;
};

struct Semantic{

	int id;
	std::string name;
	float confidence;
};

struct State{

	VectorXd x;
	float z;
	MatrixXd P;
	VectorXd x_aug;
	VectorXd P_aug;
	MatrixXd Xsig_pred;
};

struct Track{

	// Attributes
	int id;
	State sta;
	Geometry geo;
	Semantic sem;
	History hist;
	int r;
	int g;
	int b;
	float prob_existence;
};

class UnscentedKF{

public:

	// Default constructor
	UnscentedKF(ros::NodeHandle nh, ros::NodeHandle private_nh);

	// Virtual destructor
	virtual ~UnscentedKF();

	virtual void process(const ObjectArrayConstPtr & detected_objects);

protected:

	// Class member
	Parameter params_;

private:

	// Node handle
	ros::NodeHandle nh_, private_nh_;

	// Processing
	bool is_initialized_;
	int track_id_counter_;
	int time_frame_;
	tf::TransformListener listener_;

	// Visualization
	cv::RNG rng_;

	// UKF
	MatrixXd R_laser_;
	VectorXd weights_;
	std::vector<Track> tracks_;

	// Prediction
	double last_time_stamp_;

	// Subscriber
	ros::Subscriber list_detected_objects_sub_;

	// Publisher
	ros::Publisher list_tracked_objects_pub_;
	void publishTracks(const std_msgs::Header & header);

	// Class functions
	void Prediction(const double delta_t);
	void Update(const ObjectArrayConstPtr & detected_objects);
	void TrackManagement(const ObjectArrayConstPtr & detected_objects);
	void initTrack(const Object & obj);
	void printTrack(const Track & tr);
	void printTracks();

	// Data Association members
	std::vector<int> da_tracks;
	std::vector<int> da_objects;

	// Data Association functions
	void GlobalNearestNeighbor(const ObjectArrayConstPtr & detected_objects);
	float CalculateDistance(const Track & track, const Object & object);
	float CalculateBoxMismatch(const Track & track, const Object & object);
	float CalculateEuclideanAndBoxOffset(const Track & track, 
		const Object & object);

	// Others
	void setTrackHeight(Track & track, const float h);

};

} // namespace tracking

#endif // unscentedkf_H