#include "location.h"


void Location::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    // Get current location from Odom
    float positionX = msg->pose.pose.position.x;
    float positionY = msg->pose.pose.position.y;

    // Get forward driving speed
    drivingSpeed = msg->twist.twist.linear.x;

    // Calculate grid position, because of rotation we use Y for X, X for Y.
    int positionXGrid = Location::calculateGridDistance(positionY);
    int positionYGrid = Location::calculateGridDistance(positionX);

	setCurrentPosition(positionXGrid, positionYGrid);
}

void Location::imuDataCallback(const sensor_msgs::ImuConstPtr& msg) {
	this->orientation = msg->orientation;
}

Location::Location(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
	map = new Map(nodeHandle);

	imuDataSubscriber 			= nodeHandle->subscribe("/mobile_base/sensors/imu_data", 1, &Location::imuDataCallback, this);
	odomSubscriber 				= nodeHandle->subscribe("/odom", 100, &Location::odomCallback, this);

	currentLocationPublisher 	= nodeHandle->advertise<GridPoint>("/location", 100);

	ROS_INFO_STREAM("Location:: Module initialized.");
}

void Location::read() {
	ROS_INFO_STREAM("Location:: read.");
}

double Location::getDegrees() {
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation, q);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	double degrees = (yaw / M_PI * 180);
	if(degrees < 0)
		degrees = abs(degrees);
	else
		degrees = 360 - degrees;

	return degrees;
}

float Location::getDrivingSpeed() {
	return drivingSpeed;
}


// GRID

void Location::setCurrentPosition(int x, int y) {
    GridPoint * gridPoint = map->getTile(x, y);

    if(gridPoint == nullptr) {
        map->addTile(x, y, 1); // 1 because it is the current position, so it is walkable.
    }

    if(x != currentX || y != currentY) {
        currentX = x;
        currentY = y;

        GridPoint gridPointLocation;
        gridPointLocation.x = currentX;
        gridPointLocation.y = currentY;
        gridPointLocation.z = 0;
        gridPointLocation.type = 2; // 1 = walkable, 0 = not walkable, 2 = current location

        // Publish current location to other nodes.
        currentLocationPublisher.publish(gridPointLocation);
    }
}

int Location::getCurrentX() {
    return currentX;
}

int Location::getCurrentY() {
    return currentY;
}

Map * Location::getMap() {
    return map;
}

// STATIC

int Location::calculateGridDistance(float input) {
    return (int)(-input * 10);
}