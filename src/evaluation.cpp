#include "../include/test_kitti/evaluation.h"


Evaluation::Evaluation(){
	boost::filesystem::path trackletsPath = boost::filesystem::path("/home/simonappel/Coding/RosbagKitti/0005/tracklet_labels.xml");
    tracklets_.loadFromFile(trackletsPath.string());
    std::cout << "Number of tracklets " << tracklets_.numberOfTracklets() << std::endl;
    frame_counter_ = 0;

    marker_array_ = visualization_msgs::MarkerArray();
    for(int i = 0; i < tracklets_.numberOfTracklets(); ++i){

    	visualization_msgs::Marker marker;
    	marker.header.frame_id = "base_link";
    	marker.header.stamp = ros::Time();
    	marker.ns = "my_namespace";
    	marker.id = i;
    	marker.text = tracklets_.getTracklet(i)->objectType;
	    marker.type = visualization_msgs::Marker::CUBE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.color.a = 1.0; // Don't forget to set the alpha!
	    marker.color.r = 1.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0.0;
	    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	    marker.frame_locked = true;
    	marker_array_.markers.push_back(marker);

    	std::pair<int,int> start_stop;
    	start_stop.first = tracklets_.getTracklet(i)->first_frame;
    	start_stop.second = tracklets_.getTracklet(i)->first_frame + tracklets_.getTracklet(i)->poses.size();
    	std::cout << i << " " << start_stop.first << " " << start_stop.second << " " << tracklets_.getTracklet(i)->finished <<std::endl;
    	start_stop_vector_.push_back(start_stop);
    }
}

Evaluation::~Evaluation(){

}


void Evaluation::calculateRMSE(){

	std::cout << "Frame " << frame_counter_ << "  " << tracklets_.getTracklet(1)->objectType << std::endl;
	std::cout << tracklets_.getTracklet(1)->poses[frame_counter_].tx << ","
		<< tracklets_.getTracklet(1)->poses[frame_counter_].ty << std::endl;
	frame_counter_++;
}

visualization_msgs::Marker & Evaluation::plotBike(){
	visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.text = "OBJECT";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = tracklets_.getTracklet(1)->poses[frame_counter_].tx - 0.81;
    marker.pose.position.y = tracklets_.getTracklet(1)->poses[frame_counter_].ty;
    marker.pose.position.z = 1.0; //tracklets_.getTracklet(1)->h/2;
    marker.pose.orientation.x = tracklets_.getTracklet(1)->poses[frame_counter_].rx;
    marker.pose.orientation.y = tracklets_.getTracklet(1)->poses[frame_counter_].ry;
    marker.pose.orientation.z = tracklets_.getTracklet(1)->poses[frame_counter_].rz;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; //tracklets_.getTracklet(1)->l;
    marker.scale.y = 0.1; //tracklets_.getTracklet(1)->w;
    marker.scale.z = 2.0; //tracklets_.getTracklet(1)->h;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.frame_locked = true;

	frame_counter_++;

    return marker;
}

visualization_msgs::MarkerArray & Evaluation::showTracklets(){

	
	for(int i = 0; i < tracklets_.numberOfTracklets(); ++i){
		if(start_stop_vector_[i].first <= frame_counter_ && start_stop_vector_[i].second > frame_counter_){

			int index = frame_counter_ - start_stop_vector_[i].first;
		    marker_array_.markers[i].pose.position.x = tracklets_.getTracklet(i)->poses[index].tx - 0.81;
		    marker_array_.markers[i].pose.position.y = tracklets_.getTracklet(i)->poses[index].ty;
		    marker_array_.markers[i].pose.position.z = 1.0;
		    marker_array_.markers[i].scale.x = 0.1; //tracklets_.getTracklet(1)->l;
    		marker_array_.markers[i].scale.y = 0.1; //tracklets_.getTracklet(1)->w;
    		marker_array_.markers[i].scale.z = 2.0; //tracklets_.getTracklet(1)->h;

    		std::cout << "Tracklet " << i << " there in frame " << frame_counter_ << " at "
				<< tracklets_.getTracklet(i)->poses[frame_counter_].tx - 0.81 << " "
				<< tracklets_.getTracklet(i)->poses[frame_counter_].ty << " FF "
				<< start_stop_vector_[i].first << " I  " << index << std::endl;
		}
		else{
			std::cout << "Tracklet " << i << " NOT there in frame " << frame_counter_ << std::endl;
		    marker_array_.markers[i].pose.position.x = -0.81;
		    marker_array_.markers[i].pose.position.y = 0;
		    marker_array_.markers[i].pose.position.z = 0;
		    marker_array_.markers[i].scale.x = 0.1; //tracklets_.getTracklet(1)->l;
    		marker_array_.markers[i].scale.y = 0.1; //tracklets_.getTracklet(1)->w;
    		marker_array_.markers[i].scale.z = 2.0; //tracklets_.getTracklet(1)->h;
		}
	}

	frame_counter_++;

    return marker_array_;
}