#include "button.h"

void Button::buttonPressCallback(const kobuki_msgs::ButtonEventConstPtr& msg) {
	ROS_INFO_STREAM("Button:: Button pressed (button: " << (int)msg->button << ", state: " << (int)msg->state << ").");

    if(msg->button == 2 && msg->state == 1) {
		isActivated = !isActivated;
		ROS_INFO_STREAM("Button:: Status changed (value: " << isActivated << ").");
	}
}

Button::Button(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	buttonSubscriber = nodeHandle->subscribe("/mobile_base/events/button", 100, &Button::buttonPressCallback, this);
	ROS_INFO_STREAM("Button:: Module initialized.");
}

void Button::read() {
	ROS_INFO_STREAM("Button:: read.");
}

bool Button::isActive() {
	return isActivated;
}