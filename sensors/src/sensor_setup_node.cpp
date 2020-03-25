#include <ros/ros.h>
#include <sensors_lib/sensor_setup.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "sensor_setup_node");
	sensors::SensorSetup sensor_setup(
		ros::NodeHandle(), ros::NodeHandle("~"));
	ros::spin();

	return 0;
}
