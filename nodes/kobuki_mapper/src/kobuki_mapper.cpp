#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <kobuki_mapper/GridPoint.h>
#include <stdlib.h>

#include <cstdio>

using namespace std;
using namespace kobuki_mapper; // instead of kobuki_mapper::GridPoint, we can use GridPoint

ros::Publisher gridFieldPublisher;
std::vector<GridPoint> grid;

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


    if(gridPositionIndex(positionXGrid, positionYGrid) == -1) {

        ROS_INFO("gridX = %d, Y = %d", positionXGrid, positionYGrid);

        // Add grid point
        GridPoint gridPoint;
        gridPoint.x = positionXGrid;
        gridPoint.y = positionYGrid;
        gridPoint.z = 0;
        gridPoint.type = 1; // 1 = walkable, 0 = not walkable

        grid.push_back(gridPoint);
        gridFieldPublisher.publish(gridPoint);
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

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_mapper node");

	ros::init(argc, argv, "kobuki_mapper");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);

	gridFieldPublisher = n.advertise<GridPoint>("/grid_field", 100);
	ros::Subscriber gridFeedback = n.subscribe("/grid", 100, gridCallback);

    ros::spin();

	ROS_INFO("Exiting the node");
	return 0;
}

