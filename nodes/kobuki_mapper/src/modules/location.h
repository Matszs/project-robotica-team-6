#ifndef __LOCATION_H_INCLUDED__
#define __LOCATION_H_INCLUDED__

#include "module.h"
#include "../map.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <kobuki_mapper/GridPoint.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class Location : public Module {
	private:
		ros::Subscriber imuDataSubscriber;
		ros::Subscriber odomSubscriber;
		ros::Publisher currentLocationPublisher;

		void odomCallback(const nav_msgs::OdometryConstPtr& msg);
		void imuDataCallback(const sensor_msgs::ImuConstPtr& msg);

		Map * map;
		int currentX; // current location horizontal
		int currentY; // current location vertical

		float drivingSpeed = 0; // driving speed
		geometry_msgs::Quaternion orientation; // orientation of the kobuki

	public:
		Location(ros::NodeHandle * nodeHandle);
		void read();
		double getDegrees();
		float getDrivingSpeed();
		void setCurrentPosition(int x, int y);

		static int calculateGridDistance(float input);

};

#endif