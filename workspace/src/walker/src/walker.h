
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

#include <ecl/threads/thread.hpp>

bool emergency_stop = false;

class Walker {

private:
	ros::NodeHandle nodeHandler;
	ros::Subscriber cliff_event_subscriber;
	ros::Subscriber bumper_event_subscriber;
	ros::Subscriber tracked_position_event_subscriber;
	ros::Publisher velocity_publisher;
	ecl::Thread inputThread;

public:

    void init();
    void run();


    // events
    void cliffEvent(const kobuki_msgs::CliffEventConstPtr msg);
    void trackedPositionEvent(const vision::TrackedPositionConstPtr& msg);
    void bumperEvent(const kobuki_msgs::BumperEventConstPtr msg);
    void inputHandler();
    void spin();



};