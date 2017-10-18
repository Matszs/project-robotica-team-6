#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <vision/TrackedPosition.h>

#include <cstdio>

using namespace std;

ros::Publisher driver;

//initial values for the kalman filter
double rotation_x_est_last =  -msg->x * 2;
double rotation_P_last = 0;

double acceleration_x_est_last = 0;
double acceleration_P_last = 0;

double kalmanFilter(double input, double x_est_last, double P_last){
    //the noise in the system
    double Q = 0.022;
    double R = 0.617;

    double K;
    double P;
    double P_temp;
    double x_temp_est;
    double x_est;
    double z_real = 0; //the ideal value we wish to measure(not used in this case)
    double z_measured; //measured data

    //do a prediction
    x_temp_est = z_real + x_est_last;
    P_temp = P_last + Q;
    //calculate the Kalman gain
    K = P_temp * (1.0/(P_temp + R));
    //measure
    z_measured = input;
    //correct
    x_est = x_temp_est + K * (z_measured - x_temp_est);
    P = (1- K) * P_temp;
    //update our last's
    P_last = P;
    x_est_last = x_est;
    return x_est;
}


void trackedCallback(const vision::TrackedPositionConstPtr& msg) {
  ROS_INFO("x: [%f], y: [%f], z: [%f], ", -msg->x * 2, -msg->y, -msg->z);

  double rotation = kalmanFilter(-msg->x, rotation_x_est_last, rotation_P_last) * 2;

  geometry_msgs::Twist driveObj;
  driveObj.angular.z = rotation;

  if(rotation < 1 && rotation > -1) {

    double filter = (msg->z, acceleration_x_est_last, acceleration_P_last);

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

