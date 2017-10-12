#include "walker.h"

void Walker::init() {

    ROS_INFO("Walker:: initalization started.");

    bumper_event_subscriber = nodeHandler.subscribe("/mobile_base/events/bumper", 10, &Walker::bumperEvent, this);
    tracked_position_event_subscriber = nodeHandler.subscribe("/vision/tracked_position", 10, &Walker::trackedPositionEvent, this);
    velocity_publisher = nodeHandler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    inputThread.start(&Walker::inputHandler, *this);

    ROS_INFO("Walker:: initalization done.");

}

void Walker::run() {

    ROS_INFO("Walker:: run started");
    ros::spin();
    ROS_INFO("Walker:: run done.");

}


void Walker::cliffEvent(const kobuki_msgs::CliffEventConstPtr msg) {
    ROS_INFO("Walker:: cliffEvent trigger.");
    ROS_INFO("Walker:: state: %d", msg->state);
}

void Walker::trackedPositionEvent(const vision::TrackedPositionConstPtr& msg) {

    ROS_INFO("Walker:: trackedPosition trigger.");
    ROS_INFO("x: [%f], y: [%f], z: [%f], ", -msg->x * 2, -msg->y, -msg->z);

}

void Walker::bumperEvent(const kobuki_msgs::BumperEventConstPtr msg) {

    ROS_INFO("Walker:: bumperEvent trigger.");
    ROS_INFO("Walker:: state: %d", msg->state);

}

void Walker::inputHandler() {
    ros::Rate spin_rate(10);

    while (ros::ok() && !emergency_stop) {
        ROS_INFO("Walker:: inputHandler.");

        this->spin();

        spin_rate.sleep();
    }

}

void Walker::spin() {

    //ROS_INFO("Walker:: spin.");

}







// main program

int main(int argc, char **argv) {
	ROS_INFO("Starting the node");

	ros::init(argc, argv, "walker");
	ROS_INFO("ros::init done");

	Walker walker;

	walker.init();
	walker.run();

	ROS_INFO("Exiting the node");
	return 0;
}

