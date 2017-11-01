#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <kobuki_ultrasone/UltrasoneSensors.h>
#include <stdlib.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "serial/serial.h"

using namespace std;
using namespace kobuki_ultrasone; // instead of kobuki_ultrasone::UltrasoneSensors, we can use UltrasoneSensors

ros::Publisher ultrasonePublisher;


void spin() {

    serial::Serial serialCon("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(1000));
    int sensorIndex = 0;
    std::vector<int> sensorReadings;
    string numberValue = "";
    bool is_reading = false;
    bool send_data = false;
    bool sensor_has_been_initialized = false;

    if(serialCon.isOpen()) {
        ROS_INFO_STREAM("Serial :: Connection open");
    } else {
        ROS_INFO_STREAM("Serial :: Connection closed");
    }

    while(ros::ok()) {
        /*ros::Rate spin_rate(240);
        ros::spinOnce();
        spin_rate.sleep();*/

        string readIn = serialCon.read(1);
        if(readIn == "B") {
            is_reading = true;

        } else if(is_reading) {
            if(readIn == "E" || readIn == ",") {
                try {
                    sensorReadings.push_back(stoi(numberValue.c_str()));

                    if(readIn == "E") {
                        sensorIndex = 0;
                        is_reading = false;
                        send_data = true;
                    } else {
                        sensorIndex++;
                    }
                    numberValue = "";
                } catch (exception& e) {
                    ROS_INFO_STREAM("ros:: EXCEPTION " << e.what());
                }
            } else {
                numberValue.append(readIn);
            }
        }

        if(send_data) {
            if(sensorReadings.size() == 4) {
                UltrasoneSensors ultrasone_sensors;
                ultrasone_sensors.sensor1_distance = sensorReadings.at(0);
                ultrasone_sensors.sensor2_distance = sensorReadings.at(1);
                ultrasone_sensors.sensor3_distance = sensorReadings.at(2);
                ultrasone_sensors.sensor4_distance = sensorReadings.at(3);

                ultrasonePublisher.publish(ultrasone_sensors);
            }

            send_data = false;
            sensorReadings.clear();
        }

    }

}


int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_ultrasone node");

	ros::init(argc, argv, "kobuki_ultrasone");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;

	ultrasonePublisher = n.advertise<UltrasoneSensors>("/ultrasone_sensors", 100);

    spin();

	ROS_INFO("Exiting the node");
	return 0;
}

