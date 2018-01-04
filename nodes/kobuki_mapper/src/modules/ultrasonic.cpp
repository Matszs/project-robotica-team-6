#include "ultrasonic.h"

void Ultrasonic::ultrasonicSensorsReadCallback(const kobuki_ultrasone::UltrasoneSensorsConstPtr& msg) {
    setUltrasonicSensorDistance(0, msg->sensor1_distance);
    setUltrasonicSensorDistance(1, msg->sensor2_distance);
    setUltrasonicSensorDistance(2, msg->sensor3_distance);
    setUltrasonicSensorDistance(3, msg->sensor4_distance);
}

Ultrasonic::Ultrasonic(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	ultrasonicSubscriber = nodeHandle->subscribe("/ultrasone_sensors", 100, &Ultrasonic::ultrasonicSensorsReadCallback, this);
	ROS_INFO_STREAM("Ultrasonic:: Module initialized.");
}

void Ultrasonic::read() {
	ROS_INFO_STREAM("Ultrasonic:: read.");
}

void Ultrasonic::setUltrasonicSensorDistance(int sensor, int distance) {
    bufferedValues[sensor] = values[sensor];
    values[sensor] = distance;
}

int Ultrasonic::getSensorDistance(int sensor) {
	return values[sensor];
}

int Ultrasone::getAvgSensorDistance(int sensor){
    return (bufferedValues[sensor] + values[sensor]) / 2;
}