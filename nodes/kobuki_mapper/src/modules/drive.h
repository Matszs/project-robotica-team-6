#ifndef __DRIVE_H_INCLUDED__
#define __DRIVE_H_INCLUDED__

#include "module.h"
#include <geometry_msgs/Twist.h>
#include "../module_loader.h"
#include "../map.h"
#include "../rotation.h"

#include "button.h"
#include "location.h"
#include "ultrasonic.h"
#include "bumper.h"
#include "object_tracker.h"

#define LINEAR 0.2
#define ANGULAR 1.4
#define WALL_DISTANCE 20
#define MIN_WALL_DISTANCE 100

class Drive : public Module {
	private:
		ros::Publisher velocityPublisher;
		ros::Publisher obstaclePublisher;
		// 'remembers' the degrees to rotate from by the 'rotateBy'-method
		double rotateByStartDegrees = -1;
		// 'remembers' the degrees it started before rotating by the 'rotateTo'-method
		double rotateToStartDegrees = -1;

		int newDriveDirection = -1;

		Rotation * rotation;

		void publishObstacleData();

	public:
		Drive(ros::NodeHandle * nodeHandle);
		void read();
		void autonomousDriving();
		bool rotateTo(int degrees);
		bool rotateTo(int degrees, bool fixDegrees);
		bool rotateBy(int degrees, bool clockwise);
		void stop();
		void forward();
		int findDirection();
		void checkUltrasonicSensors();
		void checkBumperSensors();
		void checkAlreadyVisitedLocations();

};

#endif