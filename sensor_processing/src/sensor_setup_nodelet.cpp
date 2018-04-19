/******************************************************************************
 *
 * Author: Simon Appel (simonappel62@gmail.com)
 * Date: 19/04/2018
 *
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_processing/sensor_fusion.h>

namespace sensor_processing{

class SensorSetupNodelet: public nodelet::Nodelet{
public:
  SensorSetupNodelet() {}
  ~SensorSetupNodelet() {}

private:
  virtual void onInit()
  {
    sensor_fusion_.reset(
      new SensorFusion(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<SensorFusion> sensor_fusion_;
};

} // namespace sensor