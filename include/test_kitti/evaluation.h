#include "tracking.h"
#include <string>
#include "tracklets.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <utility>
#include <vector>
#include <tf/transform_datatypes.h>

class Evaluation{

public:
	Evaluation(std::string filename);
	~Evaluation();

	visualization_msgs::MarkerArray & showTracklets();
	void calculateRMSE();


private:

	Tracklets tracklets_;
	int frame_counter_;
	std::vector< std::pair<int,int> > start_stop_vector_;
	visualization_msgs::MarkerArray marker_array_;
};