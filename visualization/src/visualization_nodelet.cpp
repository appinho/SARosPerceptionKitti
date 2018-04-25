/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 25/04/2018
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <visualization_lib/visualization.h>

namespace visualization{

class VisualizationNodelet: public nodelet::Nodelet{

public:
	VisualizationNodelet() {}
	~VisualizationNodelet() {}

private:
	virtual void onInit()
	{
		visualizer_.reset(
			new Visualization(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<Visualization> visualizer_;
};

} // namespace sensor