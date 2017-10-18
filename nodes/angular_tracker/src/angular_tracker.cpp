#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <vision/TrackedPosition.h>

#include <cstdio>

using namespace std;

ros::Publisher driver;

void trackedCallback(const vision::TrackedPositionConstPtr& msg) {
  ROS_INFO("x: [%f], y: [%f], z: [%f], ", -msg->x * 2, -msg->y, -msg->z);

  double rotation = -msg->x * 2;

  geometry_msgs::Twist driveObj;
  driveObj.angular.z = rotation;

  if(rotation < 1 && rotation > -1) {

    double filter = msg->z;
    if(filter > 4)
        filter = 4;

    driveObj.linear.x = filter / 10;
  }

  driver.publish(driveObj);

}

int main(int argc, char **argv) {
	ROS_INFO("Starting the node");

	ros::init(argc, argv, "angular_tracker");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/vision/tracked_position", 100, trackedCallback);
	driver = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);


    ros::spin();


	ROS_INFO("Exiting the node");
	return 0;
}

