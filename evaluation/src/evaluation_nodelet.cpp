/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 26/04/2018
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <evaluation_lib/evaluation.h>

namespace evaluation{

class EvaluationNodelet: public nodelet::Nodelet{

public:
	EvaluationNodelet() {}
	~EvaluationNodelet() {}

private:
	virtual void onInit()
	{
		evaluator_.reset(
			new Evaluation(getNodeHandle(), getPrivateNodeHandle()));
	}

	boost::shared_ptr<Evaluation> evaluator_;
};

} // namespace sensor