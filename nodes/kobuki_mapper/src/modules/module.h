#ifndef __MODULE_H_INCLUDED__
#define __MODULE_H_INCLUDED__

#include <ros/ros.h>

class Module {
	private:
		ros::NodeHandle * nh;

	public:
		Module(ros::NodeHandle * nodeHandle);
		virtual void read();

};

#endif