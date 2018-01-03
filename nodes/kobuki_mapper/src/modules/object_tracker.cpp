#include "object_tracker.h"

void ObjectTracker::visionTrackedPositionCallback(const vision::TrackedPositionConstPtr & msg) {

	kobuki_msgs::Sound soundObject;
	soundObject.value = 6;
	soundsPublisher.publish(soundObject);

	objectHasBeenFound = true;
	objectFoundTimer = ros::Time::now();
}

ObjectTracker::ObjectTracker(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	visionTrackedPositionSubscriber = nodeHandle->subscribe("/vision/tracked_position", 1, &ObjectTracker::visionTrackedPositionCallback, this);
	soundsPublisher = nodeHandle->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 100);

	ROS_INFO_STREAM("ObjectTracker:: Module initialized.");
}

void ObjectTracker::read() {
	checkExpirationOfFoundObject();
	ROS_INFO_STREAM("ObjectTracker:: read.");
}

bool ObjectTracker::objectFound() {
	checkExpirationOfFoundObject();
	return objectHasBeenFound;
}

// This method checks if the 'objectHasBeenFound' bool has to be reset to false
// This is the case after 3 seconds.
void ObjectTracker::checkExpirationOfFoundObject() {
	if(objectHasBeenFound)
		if((ros::Time::now().toSec() - objectFoundTimer.toSec()) > 3)
			objectHasBeenFound = false;
}