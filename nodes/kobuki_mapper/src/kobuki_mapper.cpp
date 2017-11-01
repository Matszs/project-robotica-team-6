#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_mapper/GridPoint.h>
#include <kobuki_depth/CameraPoint.h>
#include <kobuki_ultrasone/UltrasoneSensors.h>
#include <stdlib.h>

#include <cstdio>

using namespace std;
using namespace kobuki_mapper; // instead of kobuki_mapper::GridPoint, we can use GridPoint
using namespace kobuki_depth;
using namespace kobuki_ultrasone;

ros::Publisher gridFieldPublisher;
ros::Publisher currentLocationPublisher;
std::vector<GridPoint> grid;
float degrees = 0;
int posX = -100;
int posY = 100;

int gridPositionIndex(int x, int y) {

    unsigned int i;
    for (i = 0; i < grid.size(); i++) {
        GridPoint gridPoint = grid.at(i);

        if(gridPoint.x == x && gridPoint.y == y)
            return i;
    }

    return -1;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg) {

    float positionX = msg->pose.pose.position.x;
    float positionY = msg->pose.pose.position.y;

    int positionXGrid = (int)(positionX * 10);
    int positionYGrid = (int)(positionY * 10);

    // rotation of movement
    /*int positionXGrid = (int)(-positionY * 10);
    int positionYGrid = (int)(positionX * 10);*/

    float rotation = msg->pose.pose.orientation.z;

    // get 2 decimals from the current rotation
    degrees = (float)((int)(rotation * 100)) / 100;
    // format rotation into degrees
    degrees = (degrees > 0 ? 360 - (360 * degrees / 2) : 360 * abs(degrees) / 2);


    if(gridPositionIndex(positionXGrid, positionYGrid) == -1) {

        ROS_INFO("gridX = %d, Y = %d", positionXGrid, positionYGrid);

        // Add grid point
        GridPoint gridPoint;
        gridPoint.x = positionXGrid;
        gridPoint.y = positionYGrid;
        gridPoint.z = 0;
        gridPoint.type = 1; // 1 = walkable, 0 = not walkable, 2 = current location

        grid.push_back(gridPoint);
        gridFieldPublisher.publish(gridPoint);
    }

    if(positionXGrid != posX || positionYGrid != posY) {

        GridPoint gridPointLocation;
        gridPointLocation.x = positionXGrid;
        gridPointLocation.y = positionYGrid;
        gridPointLocation.z = 0;
        gridPointLocation.type = 2; // 1 = walkable, 0 = not walkable, 2 = current location

        currentLocationPublisher.publish(gridPointLocation);

        posX = positionXGrid;
        posY = positionYGrid;
    }
}

void gridCallback(const GridPointConstPtr& msg) {
    int positionXGrid = msg->x;
    int positionYGrid = msg->y;
    int type = msg->type;


    int gridIndex = gridPositionIndex(positionXGrid, positionYGrid);
    ROS_INFO("index %d", gridIndex);
    if(gridIndex != -1) {
        ROS_INFO("NEW: gridX = %d, Y = %d", positionXGrid, positionYGrid);

        kobuki_mapper::GridPoint gridPoint;
        gridPoint.x = positionXGrid;
        gridPoint.y = positionYGrid;
        gridPoint.z = 0;
        gridPoint.type = type; // 1 = walkable, 0 = not walkable

        grid.push_back(gridPoint);
        gridFieldPublisher.publish(gridPoint); // publish on /grid_field

    } else {
        ROS_INFO("UPDATE: gridX = %d, Y = %d", positionXGrid, positionYGrid);

        GridPoint gridPoint;
        gridPoint.x = positionXGrid;
        gridPoint.y = positionYGrid;
        gridPoint.type = type;
        //grid[gridIndex] = gridPoint;

        gridFieldPublisher.publish(gridPoint); // publish on /grid_field*/

    }
}

void cameraPointsCallback(const CameraPointConstPtr& msg) {
    float distance = msg->z;
    int distanceGrid = (int)(distance * 10);


    int gridDistX = round(sin(degrees * M_PI / 180) * distanceGrid);
    int gridDistY = round(cos(degrees * M_PI / 180) * distanceGrid);

    int gridIndex = gridPositionIndex(gridDistX, gridDistY);

    if(gridIndex == -1) {
        kobuki_mapper::GridPoint gridPoint;
        gridPoint.x = gridDistX;
        gridPoint.y = gridDistY;
        gridPoint.z = 0;
        gridPoint.type = 0; // 1 = walkable, 0 = not walkable

        grid.push_back(gridPoint);
        gridFieldPublisher.publish(gridPoint); // publish on /grid_field
    }
}

void ultrasoneSensorsCallback(const UltrasoneSensorsConstPtr& msg) {
    int front = msg->sensor1_distance;
    int right = msg->sensor2_distance;
    int bottom = msg->sensor3_distance;
    int left = msg->sensor4_distance;

    if(front > 10 && front < 100) {

        int distanceGrid = round(front / 10);

        ROS_INFO_STREAM("blokkie " << distanceGrid << " degrees: " << round(degrees) << " posX " << posX << " posY " << posY);

        int gridDistY = posY + round(sin(round(degrees) * M_PI / 180) * distanceGrid);
        int gridDistX = posX + -round(cos(round(degrees) * M_PI / 180) * distanceGrid);

        ROS_INFO_STREAM("gridDistX " << gridDistX << " gridDistY: " << gridDistY);


        int gridIndex = gridPositionIndex(gridDistX, gridDistY);

        if(gridIndex == -1) {
            kobuki_mapper::GridPoint gridPoint;
            gridPoint.x = gridDistX;
            gridPoint.y = gridDistY;
            gridPoint.z = 0;
            gridPoint.type = 0; // 1 = walkable, 0 = not walkable

            grid.push_back(gridPoint);
            gridFieldPublisher.publish(gridPoint); // publish on /grid_field

            ROS_INFO("US: gridX = %d, Y = %d", gridDistX, gridDistY);
        }
    }

}

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_mapper node");

	ros::init(argc, argv, "kobuki_mapper");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);

	gridFieldPublisher = n.advertise<GridPoint>("/grid_field", 100);
	currentLocationPublisher = n.advertise<GridPoint>("/location", 100);
	ros::Subscriber gridFeedback = n.subscribe("/grid", 100, gridCallback);
	ros::Subscriber cameraPoints = n.subscribe("/camera_points", 100, cameraPointsCallback);
	ros::Subscriber ultrasoneSensors = n.subscribe("/ultrasone_sensors", 100, ultrasoneSensorsCallback);

    ros::spin();

	ROS_INFO("Exiting the node");
	return 0;
}

