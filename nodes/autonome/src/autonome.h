#ifndef AUTONOME_H
#define AUTONOME_H

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <vision/TrackedPosition.h>

#include <ros/ros.h>

#define SPEED 0.3
#define ANGLE 0.6

bool emergency_stop = false;
bool stop = false;

class Autonome {

private:
    enum bumper_states { B_LEFT, B_CENTER, B_RIGHT, B_NONE };


	ros::NodeHandle nodeHandler;
	ros::Subscriber cliff_event_subscriber;
	ros::Subscriber bumper_event_subscriber;
	ros::Subscriber tracked_position_event_subscriber;
	ros::Publisher velocity_publisher;

	ros::Duration turning_duration;
    ros::Time turning_start;

	bool left_bumper_pressed;
	bool right_bumper_pressed;
	bool center_bumper_pressed;

    bumper_states last_pressed_bumper;

	bool change_direction;
	bool is_turning;
	int turning_direction;


public:

    void init();
    void run();


    // events
    void cliffEvent(const kobuki_msgs::CliffEventConstPtr msg);
    void trackedPositionEvent(const vision::TrackedPositionConstPtr& msg);
    void bumperEvent(const kobuki_msgs::BumperEventConstPtr msg);
    void spin();

};

#endif