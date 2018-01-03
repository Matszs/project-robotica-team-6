// a* pathfinding:  https://www.raywenderlich.com/4946/introduction-to-a-pathfinding

#include <ros/ros.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <kobuki_mapper/GridPoint.h>
#include <kobuki_depth/CameraPoint.h>
#include <stdlib.h>
#include <cstdio>

#include "module_loader.h"
#include "modules/button.h"
#include "modules/object_tracker.h"
#include "modules/location.h"
#include "modules/battery.h"
#include "modules/bumper.h"
#include "modules/ultrasonic.h"
#include "modules/info_publisher.h"
#include "modules/drive.h"

using namespace std;

std::map<std::string, Module *> ModuleLoader::modules; // defines modules map

/*void cameraPointsCallback(const kobuki_depth::CameraPointConstPtr& msg) {
    robot.setCameraDepth(msg->z);
}*/

void spin() {
    ros::Rate spin_rate(10);
    while(ros::ok) {

		ModuleLoader::performReadings();

        ros::spinOnce();
        spin_rate.sleep();
        //robot.publishTime();
        //robot.runTasks();

    }
}

int main(int argc, char **argv) {
	ROS_INFO("Starting the kobuki_mapper node");

	ros::init(argc, argv, "kobuki_mapper");
	ROS_INFO("ros::init done");

    ros::NodeHandle n;

    //robot.init(&n);
	//ros::Subscriber cameraPoints = n.subscribe("/camera_points", 100, cameraPointsCallback);

	// load modules
	ModuleLoader::add("button", new Button(&n));
	ModuleLoader::add("object_tracker", new ObjectTracker(&n));
	ModuleLoader::add("location", new Location(&n));
	ModuleLoader::add("battery", new Battery(&n));
	ModuleLoader::add("bumper", new Bumper(&n));
	ModuleLoader::add("ultrasonic", new Ultrasonic(&n));
	ModuleLoader::add("info_publisher", new InfoPublisher(&n));
	ModuleLoader::add("drive", new Drive(&n));

    spin(); // uncomment to drive

	ROS_INFO("Exiting the node");
	return 0;
}

