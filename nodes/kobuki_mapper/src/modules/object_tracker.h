#ifndef __OBJECT_TRACKER_H_INCLUDED__
#define __OBJECT_TRACKER_H_INCLUDED__

#include "module.h"
#include "../module_loader.h"
#include "button.h"
#include <kobuki_msgs/Sound.h>
#include <vision/TrackedPosition.h>
#include <geometry_msgs/Twist.h>

class ObjectTracker : public Module {
	private:
		void checkExpirationOfFoundObject();
		void visionTrackedPositionCallback(const vision::TrackedPositionConstPtr & msg);
		ros::Subscriber visionTrackedPositionSubscriber;
		ros::Publisher soundsPublisher;
		ros::Publisher velocityPublisher;

		float x;
		float y;
		float z;

		bool automaticallyDriveToObject = false;

		bool objectHasBeenFound = false;
		ros::Time objectFoundTimer;

		void driveToObject();

	public:
		ObjectTracker(ros::NodeHandle * nodeHandle);
		void read();
		bool objectFound();
};

#endif