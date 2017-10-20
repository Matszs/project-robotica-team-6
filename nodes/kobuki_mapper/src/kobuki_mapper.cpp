#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>

#include <cstdio>

using namespace std;

ros::Publisher odom;
std::vector<struct WayPoint> wayPointVector;


struct WayPoint{
    string name;
    bool poi;
    geometry_msgs::Vector3 point;
} ;



bool closeToWayPoints(float x, float y, float offset) {


    unsigned int i;
    for (i=0; i<wayPointVector.size(); i++) {
    //for (std::vector<WayPoint>::iterator it = wayPointVector.begin() ; it != wayPointVector.end(); ++it){

        WayPoint wayPoint = wayPointVector.at(i);

        if(wayPointVector.size() > 3 && i < (wayPointVector.size() - 3))
            continue;

        int distanceX = (wayPoint.point.x - x) * (wayPoint.point.x - x);
        int distanceY = (wayPoint.point.y - y) * (wayPoint.point.y - y);

        if(sqrt(distanceX - distanceY) < offset)
            return true;
    }

    return false;
}



void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    //ROS_INFO("x: [%f], y: [%f], z: [%f], ", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);


    //ROS_INFO("Xl = [%f], ", waypointVector.back().point.x);
    ROS_INFO("Length: %lu", wayPointVector.size());

    float angularZ = abs(msg->twist.twist.angular.z);
    float linearX = msg->twist.twist.linear.x;

    if((linearX < 0.1 && angularZ > 0.4) || (linearX == 0 && angularZ == 0)){

        //if(wayPointVector.size() == 0 || (wayPointVector.size() > 0 && wayPointVector.back().point.x != msg->pose.pose.position.x)){
        if(!closeToWayPoints(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.3)){
            WayPoint wp;

            geometry_msgs::Vector3 odom_new;
            odom_new.z = msg->pose.pose.position.z;
            odom_new.y = msg->pose.pose.position.y;
            odom_new.x = msg->pose.pose.position.x;

            wp.point = odom_new;

            wayPointVector.push_back(wp);

            /*for (std::vector<Waypoint>::iterator it = waypointVector.begin() ; it != waypointVector.end(); ++it){
            ROS_INFO("x = [%f]", it->point.x);
            }*/

            odom.publish(odom_new);
        }
    }
}

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_mapper node");

	ros::init(argc, argv, "kobuki_mapper");
	ROS_INFO("ros::init done");

	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("/odom", 100, odomCallback);
	odom = n.advertise<geometry_msgs::Vector3>("/new_odom", 100);


    ros::spin();


	ROS_INFO("Exiting the node");
	return 0;
}

