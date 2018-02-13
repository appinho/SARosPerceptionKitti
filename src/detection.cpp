#include "../include/test_kitti/detection.h"


Detection::Detection(){

	// Initialize visualization for detected bounding boxes
    marker_array_ = visualization_msgs::MarkerArray();
    for(int i = 0; i < DET_BUFFER_SIZE; ++i){

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
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;

	    // Push back marker to marker array
    	marker_array_.markers.push_back(marker);
    }
}

Detection::~Detection(){
	
}

void Detection::runConnectedComponent(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// Clear image
	cv::Mat image_ = cv::Mat::zeros(DET_CELLS_PER_EDGE, DET_CELLS_PER_EDGE, CV_8UC1);
	cv::Mat label_ = cv::Mat::zeros(DET_CELLS_PER_EDGE, DET_CELLS_PER_EDGE, CV_8UC1);

	// Fill image
	int img_x, img_y;
	for(int i = 0; i < cloud->size(); ++i){

		transformWorldToImage(cloud->at(i).x, cloud->at(i).y, &img_x, &img_y);

		// std::cout << i << " " << img_x << " " << img_y << " " 
		// 	<< cloud->at(i).x << " " << cloud->at(i).y << std::endl;
		image_.at<char>(img_x, img_y) = 255;
	}

	//std::cout << "Image" << std::endl << image_ << std::endl;

	// Run connected component
	cv::Mat stats, centroids;
	int number_of_clusters = cv::connectedComponentsWithStats(image_, label_, stats, centroids, 8);

	//std::cout << "Label" << std::endl << label_ << std::endl;
	//std::cout << "Number of clusters " << number_of_clusters << std::endl;
	//	<< " " << stats << std::endl
	//  	<< " " << centroids << std::endl;

	// Fill clusters
	clusters_.clear();
	float x,y;
	for(int i = 1; i < number_of_clusters; ++i){

		// New cluster
		Cluster c;
		c.y = DET_RANGE - (stats.at<int>(i,0) + stats.at<int>(i,2) / 2.0) * DET_GRID_CELL_SIZE;
		c.x = DET_RANGE - (stats.at<int>(i,1) + stats.at<int>(i,3) / 2.0) * DET_GRID_CELL_SIZE + TRA_X_OFFSET;
		c.l_y = stats.at<int>(i,2) * DET_GRID_CELL_SIZE;
		c.l_x = stats.at<int>(i,3) * DET_GRID_CELL_SIZE;

		//std::cout << "Cluster # " << i << " at "
	 	//	<< c.x << "," << c.y << " and dim " 
	 	//	<< c.l_x << "," << c.l_y << std::endl;

		clusters_.push_back(c); 
	}
}

void Detection::transformWorldToImage(const float x, const float y, int * img_x, int * img_y){

	*img_x = (DET_RANGE - x) / DET_GRID_CELL_SIZE;
	*img_y = (DET_RANGE - y) / DET_GRID_CELL_SIZE;
}

visualization_msgs::MarkerArray & Detection::showDetection(){

  // Loop through clusters
  for(int i = 0; i < clusters_.size(); ++i){

	// Fill in current position, orientation
    marker_array_.markers[i].pose.position.x = clusters_[i].x;
    marker_array_.markers[i].pose.position.y = clusters_[i].y;
    marker_array_.markers[i].pose.position.z = 1.0;
	// Fill in current dimension
	marker_array_.markers[i].scale.x = clusters_[i].l_x;
	marker_array_.markers[i].scale.y = clusters_[i].l_y;
	marker_array_.markers[i].scale.z = 2.0;
  }
  // Loop through remaining buffer size
  for(int i = clusters_.size(); i < DET_BUFFER_SIZE; ++i){

	// Fill in current position, orientation
    marker_array_.markers[i].pose.position.x = 0.0;
    marker_array_.markers[i].pose.position.y = 0.0;
    marker_array_.markers[i].pose.position.z = 0.0;
	// Fill in current dimension
	marker_array_.markers[i].scale.x = 0.1;
	marker_array_.markers[i].scale.y = 0.1;
	marker_array_.markers[i].scale.z = 2.0;
  }

  return marker_array_;
}

std::vector<Cluster> & Detection::getClusters(){
	return clusters_;
}