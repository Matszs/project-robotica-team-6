#ifndef __BUTTON_H_INCLUDED__
#define __BUTTON_H_INCLUDED__

#include "module.h"
#include <kobuki_msgs/ButtonEvent.h>


class Button : public Module {
	private:
		void buttonPressCallback(const kobuki_msgs::ButtonEventConstPtr& msg);
		ros::Subscriber buttonSubscriber;
		bool isActivated = false;

	public:
		Button(ros::NodeHandle * nodeHandle);
		void read();
		bool isActive();

};

#endif