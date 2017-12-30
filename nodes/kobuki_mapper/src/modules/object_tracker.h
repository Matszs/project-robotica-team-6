#ifndef __OBJECT_TRACKER_H_INCLUDED__
#define __OBJECT_TRACKER_H_INCLUDED__

#include "module.h"
#include <kobuki_msgs/Sound.h>
#include <vision/TrackedPosition.h>

class ObjectTracker : public Module {
	private:
		void visionTrackedPositionCallback(const vision::TrackedPositionConstPtr & msg);
		ros::Subscriber visionTrackedPositionSubscriber;
		ros::Publisher soundsPublisher;

	public:
		ObjectTracker(ros::NodeHandle * nodeHandle);
		void read();
};

#endif