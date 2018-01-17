#include "object_tracker.h"

void ObjectTracker::visionTrackedPositionCallback(const vision::TrackedPositionConstPtr & msg) {

	kobuki_msgs::Sound soundObject;
	soundObject.value = 2;
	//soundsPublisher.publish(soundObject);

	objectHasBeenFound = true;
	objectFoundTimer = ros::Time::now();

	x = msg->x;
	y = msg->y;
	z = msg->z;
}

ObjectTracker::ObjectTracker(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	visionTrackedPositionSubscriber = nodeHandle->subscribe("/vision/tracked_position", 1, &ObjectTracker::visionTrackedPositionCallback, this);
	soundsPublisher = nodeHandle->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 100);
	velocityPublisher = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

	ROS_INFO_STREAM("ObjectTracker:: Module initialized.");
}

void ObjectTracker::read() {
	checkExpirationOfFoundObject();

	if((static_cast<Button *>(ModuleLoader::get("button")))->isActive() && (objectFound() || automaticallyDriveToObject))
	    driveToObject();

	ROS_INFO_STREAM("ObjectTracker:: read.");
}

void ObjectTracker::driveToObject() {
    double rotation = -x * 2;

    geometry_msgs::Twist driveObj;
    driveObj.angular.z = rotation;

    if(!automaticallyDriveToObject) {
        if(rotation < 1 && rotation > -1) {
            double filter = z;
            if(filter > 4)
                filter = 4;

            driveObj.linear.x = filter / 10;
        }
    }

    ROS_INFO_STREAM(" z " << z);
    ROS_INFO_STREAM(" automaticallyDriveToObject " << automaticallyDriveToObject);

    if(z < 0.4 && !automaticallyDriveToObject) {
        automaticallyDriveToObject = true;

        driveObj.linear.x = 0.4;
    } else if(z > 0.4) {
        automaticallyDriveToObject = false;
    }


    velocityPublisher.publish(driveObj);
}

bool ObjectTracker::objectFound() {
	checkExpirationOfFoundObject();
	return objectHasBeenFound;
}

// This method checks if the 'objectHasBeenFound' bool has to be reset to false
// This is the case after 3 seconds.
void ObjectTracker::checkExpirationOfFoundObject() {
	if(objectHasBeenFound) {
		if((ros::Time::now().toSec() - objectFoundTimer.toSec()) > 3) {
			objectHasBeenFound = false;
			automaticallyDriveToObject = false;
        }
    }
}