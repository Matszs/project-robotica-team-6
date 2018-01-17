#ifndef __ARM_H_INCLUDED__
#define __ARM_H_INCLUDED__

#include "module.h"
#include "sensor_msgs/JointState.h"

#define ARM_BASE "j1"
#define ARM_SHOULDER "j2"
#define ARM_ELBOW "j3"
#define ARM_WRIST "j4"
#define ARM_WRIST_ROT "j5"
#define ARM_HAND "gripper"

class Arm : public Module {
	private:
	    ros::Publisher armPublisher;

	public:
		Arm(ros::NodeHandle * nodeHandle);
		void read();

		void setGripperState(int level);
		void setStartPosition();
		void resetPosition();

};

#endif