#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include <ros/ros.h>
#include <algorithm>

#include <kobuki_mapper/Info.h>
#include <kobuki_mapper/GridPoint.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

class Rotation;
class Map;

#include "pathfinder.h"

#define WALL_DISTANCE_MIN 100

using namespace std;
using namespace kobuki_mapper;

class Robot {

    private:
        ros::Publisher currentLocationPublisher;
        ros::Publisher cmd_vel_publisher;
        ros::Publisher degrees_publisher;
        ros::Publisher time_publisher;
        ros::Publisher speed_publisher;
        ros::Publisher info_publisher;

        ros::Time startTime;

        std::vector<GridPoint> grid;

        Map * _map;
        Rotation * _rotation;

        int currentX = -100;
        int currentY = 100;

        float speedBuffer = 0;
        int timeBuffer = 0;
        int batteryBuffer = 0;

        float degrees = 0;
        float angularZ = 0;

        float cameraDepth = 0;

        bool bumper[3] = { false, false, false }; // left, center, right
        int ultrasoneValues[4] = { 1000, 1000, 1000, 1000 }; // front, right, back, left (vooraanzicht)

        bool driveForward = true;
        bool isStopped = false;
        bool mustRotate = false;
        bool isRotating = false;
        int turnDirection = 0;

        float wallDistance = 20;

        float linear = 0.2;
        float angle = 0.75;

        int degreesOfRotation = -1;
        int degreesAddRotation = -1;

        bool isDrivingKnownPath = false;

        bool gapIsRight = false;
        bool gapIsLeft = false;

        bool gapRightStartFound = false;
        GridPoint gapRightStart;
        GridPoint gapRightEnd;

        bool gapLeftStartFound = false;
        GridPoint gapLeftStart;
        GridPoint gapLeftEnd;

        bool endOfGap = false;
        bool hasRotatedBecauseOfGap = false;

        ros::Duration driveToGapDuration;
        ros::Time driveToGapStartTime;

        int distanceToLeft = -1;
        int distanceToRight = -1;

        double startDegrees = -1;
        double startRelativeDegrees = -1;

        bool driveToPoint = false;
        GridPoint drivePoint;

        void driveAutonomous();
        void driveByPath();
        void findGap();

    public:

        Map * map();
        Rotation * rotation();

        void init(ros::NodeHandle * nodeHandle);
        void setRotation(float orientationZ, float angularZ);
        void setCurrentPosition(int x, int y);
        void setCameraDepth(float depth);
        void setUltrasoneSensorDistance(int sensor, int distance);
        void publishTime();
        void setBumperState(int index, bool state);
        bool getBumperStates();
        bool getBumperState(int index);
        static int calculateGridDistance(float input);
        void drive();
        void calculatePath(); // testing the pathfinder...
        void setOrientation(geometry_msgs::Quaternion orientation);
        double getDegrees();
        bool rotateTo(int degrees);
        bool rotateTo(int degrees, bool fixDegrees);
        bool rotateBy(int degrees, bool clockwise);
        geometry_msgs::Quaternion orientation;
        bool hasOrientation;
        void setSpeed(float speed);
        void runTasks();
        void setBatteryPercentage(int batteryPercentage);

        int getCurrentX();
        int getCurrentY();
};

#endif