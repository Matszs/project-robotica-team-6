#ifndef __DRIVE_H_INCLUDED__
#define __DRIVE_H_INCLUDED__

#include "module.h"
#include <geometry_msgs/Twist.h>
#include "../module_loader.h"
#include "button.h"
#include "location.h"

class Drive : public Module {
	private:
		ros::Publisher velocityPublisher;
		// 'remembers' the degrees to rotate from by the 'rotateBy'-method
		double rotateByStartDegrees = -1;
		// 'remembers' the degrees it started before rotating by the 'rotateTo'-method
		double rotateToStartDegrees = -1;

	public:
		Drive(ros::NodeHandle * nodeHandle);
		void read();
		void autonomousDriving();
		bool rotateTo(int degrees);
		bool rotateTo(int degrees, bool fixDegrees);
		bool rotateBy(int degrees, bool clockwise);

};

#endif