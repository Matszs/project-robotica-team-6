#include "bumper.h"

void Bumper::bumperPressCallback(const kobuki_msgs::BumperEventConstPtr& msg) {
	if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
		setBumperState(msg->bumper, true);
	} else{
		setBumperState(msg->bumper, false);
	}
}

Bumper::Bumper(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	bumperSubscriber = nodeHandle->subscribe("/mobile_base/events/bumper", 10, &Bumper::bumperPressCallback, this);
	ROS_INFO_STREAM("Bumper:: Module initialized.");
}

void Bumper::read() {
	ROS_INFO_STREAM("Bumper:: read.");
}

void Bumper::setBumperState(int index, bool state) {
    bumperSides[index] = state;
}

bool Bumper::getBumperStates(){
    return (bumperSides[0] || bumperSides[1] || bumperSides[2]);
}

bool Bumper::getBumperState(int index){
    return bumperSides[index];
}