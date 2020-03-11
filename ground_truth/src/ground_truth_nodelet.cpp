/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 11/03/2020
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ground_truth_lib/ground_truth.h>

namespace ground_truth{

class GroundTruthNodelet: public nodelet::Nodelet{

public:
	GroundTruthNodelet() {}
	~GroundTruthNodelet() {}

private:
	virtual void onInit()
	{
		ground_truth_publisher_.reset(
			new GroundTruth(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<GroundTruth> ground_truth_publisher_;
};

} // namespace sensor