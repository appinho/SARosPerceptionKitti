#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <detection_lib/detection.h>

namespace detection{

class DetectionNodelet: public nodelet::Nodelet{

public:
	DetectionNodelet() {}
	~DetectionNodelet() {}

private:
	virtual void onInit()
	{
		detector_.reset(
			new Detection(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<Detection> detector_;
};

} // namespace sensor