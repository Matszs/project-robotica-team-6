// a* pathfinding:  https://www.raywenderlich.com/4946/introduction-to-a-pathfinding

#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <kobuki_mapper/GridPoint.h>
#include <kobuki_mapper/Info.h>
#include <kobuki_msgs/SensorState.h>
#include <kobuki_depth/CameraPoint.h>
#include <kobuki_ultrasone/UltrasoneSensors.h>
#include <kobuki_msgs/ButtonEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include <stdlib.h>
#include <cstdio>

#include "robot.h"

using namespace std;
using namespace kobuki_mapper; // instead of kobuki_mapper::GridPoint, we can use GridPoint
using namespace kobuki_depth;
using namespace kobuki_ultrasone;

bool is_activated = false;

Robot robot;

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    // Get current location from Odom
    float positionX = msg->pose.pose.position.x;
    float positionY = msg->pose.pose.position.y;

    // Calculate grid position, because of rotation we use Y for X, X for Y.
    int positionXGrid = Robot::calculateGridDistance(positionY);
    int positionYGrid = Robot::calculateGridDistance(positionX);

    // Let the robot know it's rotation
    robot.setRotation(msg->pose.pose.orientation.z, msg->twist.twist.angular.z);

    // Let the robot know it's position
    robot.setCurrentPosition(positionXGrid, positionYGrid);
    robot.setSpeed(msg->twist.twist.linear.x);
}

void buttonsCallback(const kobuki_msgs::ButtonEventConstPtr& msg) {

    ROS_INFO_STREAM("button: " << msg->button);
    ROS_INFO_STREAM("state: " << msg->state);

    // When pressing the first button
    if(msg->button == 2 && msg->state == 1) {
        is_activated = !is_activated;

        ROS_INFO_STREAM("activated: " << is_activated);
    }
}

void cameraPointsCallback(const CameraPointConstPtr& msg) {
    robot.setCameraDepth(msg->z);
}

void ultrasoneSensorsCallback(const UltrasoneSensorsConstPtr& msg) {
    robot.setUltrasoneSensorDistance(0, msg->sensor1_distance);
    robot.setUltrasoneSensorDistance(1, msg->sensor2_distance);
    robot.setUltrasoneSensorDistance(2, msg->sensor3_distance);
    robot.setUltrasoneSensorDistance(3, msg->sensor4_distance);
}

void imuDataCallback(const sensor_msgs::ImuConstPtr& msg) {
    robot.setOrientation(msg->orientation);
}

void bumperCallback(const kobuki_msgs::BumperEventConstPtr& msg){

    if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
        ROS_INFO_STREAM("Bumper: " << msg->bumper);
        robot.setBumperState(msg->bumper, true);
    } else{
        robot.setBumperState(msg->bumper, false);
    }
}

void batteryCallback(const kobuki_msgs::SensorStateConstPtr& msg){
    robot.setBatteryPercentage(msg->battery / 160 * 100);
}


void spin() {
    ros::Rate spin_rate(10);
    while(ros::ok) {
        if(is_activated) {
            robot.drive();

        }

        ros::spinOnce();
        spin_rate.sleep();
        //robot.publishTime();
        robot.runTasks();

    }
}

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_mapper node");

	ros::init(argc, argv, "kobuki_mapper");
	ROS_INFO("ros::init done");

    ros::NodeHandle n;
    robot.init(&n);

	ros::Subscriber odom_sub            = n.subscribe("/odom", 100, odomCallback);
	ros::Subscriber cameraPoints        = n.subscribe("/camera_points", 100, cameraPointsCallback);
	ros::Subscriber ultrasoneSensors    = n.subscribe("/ultrasone_sensors", 100, ultrasoneSensorsCallback);

	ros::Subscriber buttons             = n.subscribe("/mobile_base/events/button", 100, buttonsCallback);
	ros::Subscriber imuData             = n.subscribe("/mobile_base/sensors/imu_data", 1, imuDataCallback);
	ros::Subscriber bumperSubscriber    = n.subscribe("/mobile_base/events/bumper", 10, bumperCallback);
	ros::Subscriber battery             = n.subscribe("/mobile_base/sensors/core", 10, batteryCallback);

	/*robot.printRotationPossibilities();
	robot.decreaseRotationPossibilities(315, 90, 2);
	robot.printRotationPossibilities();*/

    spin(); // uncomment to drive

	ROS_INFO("Exiting the node");
	return 0;
}

