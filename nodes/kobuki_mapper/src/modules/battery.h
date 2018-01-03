#ifndef __BATTERY_H_INCLUDED__
#define __BATTERY_H_INCLUDED__

#include "module.h"

class Battery : public Module {
	private:
		void coreCallback(const kobuki_msgs::SensorStateConstPtr& msg);
		ros::Subscriber coreSubscriber;
		int batteryLevel = 0;

	public:
		Battery(ros::NodeHandle * nodeHandle);
		void read();
		int getBatteryLevel();

};

#endif