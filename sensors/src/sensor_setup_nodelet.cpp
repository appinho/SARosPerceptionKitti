#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensors_lib/sensor_setup.h>

namespace sensors{

class SensorSetupNodelet: public nodelet::Nodelet{
public:
  SensorSetupNodelet() {}
  ~SensorSetupNodelet() {}

private:
  virtual void onInit()
  {
    sensor_setup_.reset(
      new SensorSetup(getNodeHandle(), getPrivateNodeHandle()));
  }

  boost::shared_ptr<SensorSetup> sensor_setup_;
};

} // namespace sensors