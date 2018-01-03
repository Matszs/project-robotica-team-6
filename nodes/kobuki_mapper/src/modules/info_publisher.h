#ifndef __INFO_H_INCLUDED__
#define __INFO_H_INCLUDED__

#include "module.h"
#include <kobuki_mapper/Info.h>
#include "../module_loader.h"
#include "location.h"
#include "battery.h"
#include "object_tracker.h"

class InfoPublisher : public Module {
	private:
		ros::Publisher infoPublisher;
        ros::Time timerStartTime;

	public:
		InfoPublisher(ros::NodeHandle * nodeHandle);
		void read();
		void publishInfo();

};

#endif