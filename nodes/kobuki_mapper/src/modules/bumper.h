#ifndef __BUMPER_H_INCLUDED__
#define __BUMPER_H_INCLUDED__

#include "module.h"
#include <kobuki_msgs/BumperEvent.h>

class Bumper : public Module {
	private:
		void bumperPressCallback(const kobuki_msgs::BumperEventConstPtr& msg);
		ros::Subscriber bumperSubscriber;
		bool bumperSides[3] = { false, false, false }; // left, center, right

	public:
		Bumper(ros::NodeHandle * nodeHandle);
		void read();
		void setBumperState(int index, bool state);
        bool getBumperStates();
        bool getBumperState(int index);

};

#endif