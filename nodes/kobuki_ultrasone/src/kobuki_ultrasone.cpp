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

int getAverage(std::vector<std::vector<int> > & sensorReadings, int index) {
    std::vector<int> readings = sensorReadings.at(index);

    int sum = 0;
    for (std::size_t i = 0; i != readings.size(); ++i) {
        sum += readings[i];
        if(readings.size() > 1 && i < 1)
            sensorReadings.at(index).erase(sensorReadings.at(index).begin() + i);
    }

    /*if(index == 3)
        ROS_INFO_STREAM("total: " << readings.size());*/

    if(readings.size() > 0)
        return sum / readings.size();
    return 0;
}

void spin() {

    serial::Serial serialCon("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));
    int sensorIndex = 0;
    std::vector<std::vector<int> > sensorReadings;
    string numberValue = "";
    bool is_reading = false;
    bool send_data = false;
    bool sensor_has_been_initialized = false;

    if(serialCon.isOpen()) {
        ROS_INFO_STREAM("Serial :: Connection open");
    } else {
        ROS_INFO_STREAM("Serial :: Connection closed");
    }

    int counter;
    for(counter = 0; counter < 4; counter++) {
        vector <int> sensorReadingData;
        sensorReadings.push_back(sensorReadingData);
    }

    while(ros::ok()) {

        /*ros::Rate spin_rate(240);
        ros::spinOnce();
        spin_rate.sleep();*/

        string readIn = serialCon.read(1);

        //ROS_INFO_STREAM("char: " << readIn);

        if(readIn == "B") {
            is_reading = true;

        } else if(is_reading) {
            if(readIn == "E" || readIn == ",") {
                try {

                    //ROS_INFO_STREAM("INDEX " << sensorIndex << ": " << numberValue);
                    if((sensorIndex + 1) <= sensorReadings.size())
                        sensorReadings.at(sensorIndex).push_back(stoi(numberValue.c_str()));

                } catch (exception& e) {
                    ROS_INFO_STREAM("ros:: EXCEPTION " << e.what() << " == " << numberValue);
                }

                if(readIn == "E") {
                    sensorIndex = 0;
                    is_reading = false;
                    send_data = true;
                } else {
                    sensorIndex++;
                }
                numberValue = "";
            } else {
                numberValue.append(readIn);
            }
        }

        if(send_data) {
            //ROS_INFO_STREAM("length: " << sensorReadings.size());
            if(sensorReadings.size() == 4) {

                //ROS_INFO_STREAM("avg1 " << getAverage(sensorReadings, 0));

                UltrasoneSensors ultrasone_sensors;
                ultrasone_sensors.sensor1_distance = getAverage(sensorReadings, 0);
                ultrasone_sensors.sensor2_distance = getAverage(sensorReadings, 3);
                ultrasone_sensors.sensor3_distance = getAverage(sensorReadings, 2);
                ultrasone_sensors.sensor4_distance = getAverage(sensorReadings, 1);

                ultrasonePublisher.publish(ultrasone_sensors);

            }

            send_data = false;
            //sensorReadings.clear();
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

