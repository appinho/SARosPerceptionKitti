#include "../include/test_kitti/evaluation.h"


Evaluation::Evaluation(std::string filename){

	// Load tracklets from filename
	boost::filesystem::path trackletsPath = boost::filesystem::path(filename);
    tracklets_.loadFromFile(trackletsPath.string());
    
    //std::cout << "Number of tracklets " << tracklets_.numberOfTracklets() << std::endl;

    // Initialize visualization for ground truth bounding boxes
    marker_array_ = visualization_msgs::MarkerArray();
    for(int i = 0; i < tracklets_.numberOfTracklets(); ++i){

    	// Fill in marker information
    	visualization_msgs::Marker marker;
    	marker.header.frame_id = "base_link";
    	marker.header.stamp = ros::Time();
    	marker.ns = "my_namespace";
    	marker.id = i;
    	marker.text = tracklets_.getTracklet(i)->objectType;
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	   	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	    marker.frame_locked = true;

	    // Display box with certain color and alpha
	    marker.color.a = 0.5;
	    marker.color.r = 1.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;

	    // Push back marker to marker array
    	marker_array_.markers.push_back(marker);

    	// Remember frame when tracklet i has been seen first and last
    	std::pair<int,int> start_stop;
    	start_stop.first = tracklets_.getTracklet(i)->first_frame;
    	start_stop.second = tracklets_.getTracklet(i)->first_frame + tracklets_.getTracklet(i)->poses.size();
    	start_stop_vector_.push_back(start_stop);
    }

    // Set frame counter to 0
    frame_counter_ = 0;
}

Evaluation::~Evaluation(){

}

visualization_msgs::MarkerArray & Evaluation::showTracklets(){

	// Loop through tracklets
	for(int i = 0; i < tracklets_.numberOfTracklets(); ++i){

		// If tracklet is visible for current time frame frame_counter
		if(start_stop_vector_[i].first <= frame_counter_ && start_stop_vector_[i].second > frame_counter_){

			// Calculate index within poses vector and grab tracklet and rotation information
			int index = frame_counter_ - start_stop_vector_[i].first;
			Tracklets::tTracklet tracklet = *tracklets_.getTracklet(i);
			tf::Quaternion quat = tf::createQuaternionFromRPY(tracklet.poses[index].rz, tracklet.poses[index].ry, tracklet.poses[index].rx);

			// Fill in current position, orientation
		    marker_array_.markers[i].pose.position.x = tracklet.poses[index].tx + TRA_X_OFFSET;
		    marker_array_.markers[i].pose.position.y = tracklet.poses[index].ty;
		    marker_array_.markers[i].pose.position.z = tracklet.h/2;
		    // Fill in current orientation
    		marker_array_.markers[i].pose.orientation.w = quat[3];
	   		marker_array_.markers[i].pose.orientation.x = quat[2];
	    	marker_array_.markers[i].pose.orientation.y = quat[1];
	    	marker_array_.markers[i].pose.orientation.z = quat[0];
	    	// Fill in current dimension
	    	marker_array_.markers[i].scale.x = tracklet.l;
    		marker_array_.markers[i].scale.y = tracklet.w;
    		marker_array_.markers[i].scale.z = tracklet.h;

	    	// Prints
	    	/*
    		std::cout << "Tracklet " << i << " there in frame " << frame_counter_ << " at "
				<< tracklet.poses[index].tx + TRA_X_OFFSET << " "
				<< tracklet.poses[index].ty << " ROT "
				<< tracklet.poses[index].rx << " "
				<< tracklet.poses[index].ry << " "
				<< tracklet.poses[index].rz << " "
				<<std::endl;
			std::cout << "Quat " << quat[0] << " " << quat[1] << " " << quat[2] << " " << quat[3] << " " << std::endl;
			*/
		}
		// If tracklet is NOT visible for current time frame frame_counter
		else{
		    marker_array_.markers[i].pose.position.x = TRA_X_OFFSET;
		    marker_array_.markers[i].pose.position.y = 0;
		    marker_array_.markers[i].pose.position.z = 0;
		    marker_array_.markers[i].scale.x = 0.1; //tracklets_.getTracklet(1)->l;
    		marker_array_.markers[i].scale.y = 0.1; //tracklets_.getTracklet(1)->w;
    		marker_array_.markers[i].scale.z = 2.0; //tracklets_.getTracklet(1)->h;

    		// Prints
    		//std::cout << "Tracklet " << i << " NOT there in frame " << frame_counter_ << std::endl;
		}
	}

	// Increment frame counter
	frame_counter_++;

    return marker_array_;
}

void Evaluation::calculateRMSE(){

	// TODO
	std::cout << "Frame " << frame_counter_ << "  " << tracklets_.getTracklet(1)->objectType << std::endl;
	std::cout << tracklets_.getTracklet(1)->poses[frame_counter_].tx << ","
		<< tracklets_.getTracklet(1)->poses[frame_counter_].ty << std::endl;
	frame_counter_++;
}