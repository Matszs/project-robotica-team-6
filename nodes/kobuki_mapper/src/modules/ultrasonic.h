#ifndef __ULTRASONIC_H_INCLUDED__
#define __ULTRASONIC_H_INCLUDED__

#include "module.h"
#include <kobuki_ultrasone/UltrasoneSensors.h>

class Ultrasonic : public Module {
	private:
		void ultrasonicSensorsReadCallback(const kobuki_ultrasone::UltrasoneSensorsConstPtr& msg);
		ros::Subscriber ultrasonicSubscriber;
		int values[4] = { 1000, 1000, 1000, 1000 }; // front, right, back, left (before-front view)

	public:
		Ultrasonic(ros::NodeHandle * nodeHandle);
		void read();
		void setUltrasonicSensorDistance(int sensor, int distance);
		int getSensorDistance(int sensor);
};

#endif