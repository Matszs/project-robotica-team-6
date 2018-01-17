#include "button.h"

void Button::buttonPressCallback(const kobuki_msgs::ButtonEventConstPtr& msg) {
	ROS_INFO_STREAM("Button:: Button pressed (button: " << (int)msg->button << ", state: " << (int)msg->state << ").");

    if(msg->button == 2 && msg->state == 1) {
		isActivated = !isActivated;
		ROS_INFO_STREAM("Button:: Status changed (value: " << isActivated << ").");
	}

    if(msg->button == 1 && msg->state == 1) {
        resetPosition = !resetPosition;

        if(!resetPosition)
            (static_cast<Arm *>(ModuleLoader::get("arm")))->setStartPosition();
        else
            (static_cast<Arm *>(ModuleLoader::get("arm")))->resetPosition();

		ROS_INFO_STREAM("Button:: Button 1, Status changed (value: " << isActivated << ").");
	}

    if(msg->button == 0) {
        (static_cast<Arm *>(ModuleLoader::get("arm")))->setGripperState(msg->state * 100);

		ROS_INFO_STREAM("Button:: Button 0, Status changed (value: " << isActivated << ").");
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