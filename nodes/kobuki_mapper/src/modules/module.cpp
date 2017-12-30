#include "module.h"

Module::Module(ros::NodeHandle * nodeHandle) {
	this->nh = nodeHandle;
}

void Module::read() {
	ROS_INFO_STREAM("> Sensor read");
}
