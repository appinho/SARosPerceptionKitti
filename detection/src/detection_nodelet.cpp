/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 23/04/2018
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <detection_lib/dbscan.h>

namespace detection{

class DetectionNodelet: public nodelet::Nodelet{

public:
	DetectionNodelet() {}
	~DetectionNodelet() {}

private:
	virtual void onInit()
	{
		detector_.reset(
			new DbScan(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<DbScan> detector_;
};

} // namespace sensor