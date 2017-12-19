#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include <ros/ros.h>
#include <algorithm>

#include <kobuki_mapper/GridPoint.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float64.h"

#include "pathfinder.h"

#define WALL_DISTANCE_MIN 100

using namespace std;
using namespace kobuki_mapper;

class Robot {

    private:
        ros::Publisher gridFieldPublisher;
        ros::Publisher currentLocationPublisher;
        ros::Publisher cmd_vel_publisher;
        ros::Publisher degrees_publisher;

        std::vector<GridPoint> grid;
        int currentX = -100;
        int currentY = 100;

        float degrees = 0;
        float angularZ = 0;

        float cameraDepth = 0;

        int ultrasoneValues[4] = {1000, 1000, 1000, 1000}; // front, right, back, left (vooraanzicht)

        bool driveForward = true;
        bool isStopped = false;
        bool mustRotate = false;
        bool isRotating = false;
        int turnDirection = 0;

        float wallDistance = 30;
        ros::Duration turningDuration; // old?
        ros::Time turningStarted; // old?

        float linear = 0.2;
        float angle = 0.75;

        int rotationPossibilities[360]; // 0 - 359


        int degreesOfRotation;



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

        double startDegrees = -1;
        double startRelativeDegrees = -1;



        bool driveToPoint = false;
        GridPoint drivePoint;

        void updateRotationPossibilities(int offset, int length, int math);
        void drive_autonomous();
        void drive_to_point();

        bool bumper[3] = {false,false,false};


    public:

        void init(ros::NodeHandle * nodeHandle);
        void addTile(int x, int y, int type);
        GridPoint * getTile(int x, int y);
        void setRotation(float orientationZ, float angularZ);
        void setCurrentPosition(int x, int y);
        void setCameraDepth(float depth);
        void setUltrasoneSensorDistance(int sensor, int distance);

        void setBumperState(int index, bool state);
        bool getBumperStates();
        bool getBumperState(int index);

        static int calculateGridDistance(float input);
        void drive();
        void resetRotationPossibilities();
        void increaseRotationPossibilities(int offset, int length, int steps);
        void decreaseRotationPossibilities(int index, int length, int steps);
        void printRotationPossibilities();
        int getRotationDirection();

        void calculatePath();

        void setOrientation(geometry_msgs::Quaternion orientation);

        double getDegrees();
        bool rotateTo(int degrees);
        bool rotateBy(int degrees, bool clockwise);


        geometry_msgs::Quaternion orientation;
        bool hasOrientation;

};

#endif