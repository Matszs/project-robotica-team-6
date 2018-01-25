#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <geometry_msgs/Twist.h>
#include <vision/TrackedPosition.h>

#include <cstdio>

using namespace std;

ros::Publisher driver;

double speed;

//initial values for the lowpass filter
double rawData;
double smoothData;
double LPF_Beta = 0.5; // 0<ß<1

double lowpassFilter(double rawData) {
    smoothData = smoothData - (LPF_Beta * (smoothData - rawData));
    return smoothData;
}


//drives the robot
void drive(double speed, double rotation){
    geometry_msgs::Twist driveObj;
    driveObj.angular.z = rotation;
    driveObj.linear.x = speed;
    ROS_INFO("x: [%f], z: [%f]", rotation, speed);
    driver.publish(driveObj);
}


//Kalman
double estimate = 1; //or initial estimate (needs initialisation, could be anything)
double errorEstimate = 0.8; //or initial estimate (needs initialisation, give a good range)
double previousEstimate = estimate; // for init it is the same as the estimate
double errorMeasurement = 0.8; //(needs initialisation)
double measurement; //needs initialisation with by using new measurements
int loops = 2000; // number of maximum cycles for the kalman loop
int loopCount = 0;
bool newData;

void kalmanFilter(double measurement, double speed) {
    double errorEstimate = 0.8;
    while(newData){
        loopCount++;

        double kalmanGain = errorEstimate / (errorEstimate + errorMeasurement);

        estimate = previousEstimate + kalmanGain * (measurement - previousEstimate);
        errorEstimate = (1 - kalmanGain) * errorEstimate;

        ROS_INFO("prediction: %f \t measured value: %f \t diff: %f \n", estimate, measurement, (estimate - measurement));
        previousEstimate = estimate;
        double rotation = estimate;
        ros::spinOnce(); //get new messages, if so trackedCallback gets called;
        ROS_INFO("loop %i", loopCount);
        drive(speed / 10, rotation);
        if(loopCount > 10 ){
			newData = false; 
		} 	
		break;
	}
}


void trackedCallback(const vision::TrackedPositionConstPtr& msg) {
    //speed
    newData = true;
    loopCount = 0;
	ROS_INFO("TRACKED CALLBACK");
    double speed = lowpassFilter(msg->z);
    if (speed > 4)
        speed = 4;

	//rotation
    kalmanFilter(-msg->x, speed);
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
