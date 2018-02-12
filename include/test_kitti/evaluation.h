#include "tracking.h"
#include <string>
#include "tracklets.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <utility>
#include <vector>

class Evaluation{

public:
	Evaluation();
	~Evaluation();

	void calculateRMSE();
	visualization_msgs::Marker & plotBike();
	visualization_msgs::MarkerArray & showTracklets();

private:

	Tracklets tracklets_;
	int frame_counter_;
	std::vector< std::pair<int,int> > start_stop_vector_;
	visualization_msgs::MarkerArray marker_array_;
};