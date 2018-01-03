#include "info_publisher.h"

InfoPublisher::InfoPublisher(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	infoPublisher = nodeHandle->advertise<kobuki_mapper::Info>("/info", 1);
	timerStartTime = ros::Time::now();

	ROS_INFO_STREAM("InfoPublisher:: Module initialized.");
}

void InfoPublisher::read() {
	publishInfo();
	ROS_INFO_STREAM("InfoPublisher:: read.");
}

void InfoPublisher::publishInfo() {
	kobuki_mapper::Info info;
	info.time = ros::Time::now().toSec() - timerStartTime.toSec();
	info.speed = (static_cast<Location *>(ModuleLoader::get("location")))->getDrivingSpeed();
	info.degrees = (static_cast<Location *>(ModuleLoader::get("location")))->getDegrees();
	info.battery = (static_cast<Battery *>(ModuleLoader::get("battery")))->getBatteryLevel();
	info.found = (static_cast<ObjectTracker *>(ModuleLoader::get("object_tracker")))->objectFound();

	infoPublisher.publish(info);
}