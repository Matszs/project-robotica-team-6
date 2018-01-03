#include "drive.h"

Drive::Drive(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {

	ROS_INFO_STREAM("Drive:: Module initialized.");
}

void Drive::read() {
	autonomousDriving();
	ROS_INFO_STREAM("Drive:: read.");
}

void Drive::autonomousDriving() {
	if((static_cast<Button *>(ModuleLoader::get("button")))->isActive()) {
		// todo driving
	}
}