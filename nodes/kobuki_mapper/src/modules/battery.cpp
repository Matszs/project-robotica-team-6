#include "battery.h"

void Battery::coreCallback(const kobuki_msgs::SensorStateConstPtr& msg) {
	batteryLevel = (int)(((float)msg->battery / 160 * 100));
}

Battery::Battery(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	coreSubscriber = nodeHandle->subscribe("/mobile_base/sensors/core", 1, &Battery::coreCallback, this);
	ROS_INFO_STREAM("Battery:: Module initialized.");
}

void Button::read() {
	ROS_INFO_STREAM("Battery:: read.");
}

int Button::getBatteryLevel() {
	return batteryLevel;
}