#ifndef __DRIVE_H_INCLUDED__
#define __DRIVE_H_INCLUDED__

#include "module.h"
#include "../module_loader.h"
#include "button.h"

class Drive : public Module {
	private:

	public:
		Drive(ros::NodeHandle * nodeHandle);
		void read();
		void autonomousDriving();

};

#endif