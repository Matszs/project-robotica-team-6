#include "object_tracker.h"

void ObjectTracker::visionTrackedPositionCallback(const vision::TrackedPositionConstPtr & msg) {

	kobuki_msgs::Sound soundObject;
	soundObject.value = 6;
	soundsPublisher.publish(soundObject);

	// TODO create method 'objectIsFound', boolean
	// this method will check if the time passed by < 3 sec, reset
}

ObjectTracker::ObjectTracker(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	visionTrackedPositionSubscriber = nodeHandle->subscribe("/vision/tracked_position", 1, &ObjectTracker::visionTrackedPositionCallback, this);
	soundsPublisher = nodeHandle->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 100);

	ROS_INFO_STREAM("ObjectTracker:: Module initialized.");
}

void ObjectTracker::read() {
	ROS_INFO_STREAM("ObjectTracker:: read.");
}