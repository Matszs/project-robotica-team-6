#ifndef __ROBOT_H_INCLUDED__
#define __ROBOT_H_INCLUDED__

#include <ros/ros.h>
#include <algorithm>

#include <kobuki_mapper/GridPoint.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace kobuki_mapper;

class Robot {

    private:
        ros::Publisher gridFieldPublisher;
        ros::Publisher currentLocationPublisher;
        ros::Publisher cmd_vel_publisher;

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

        float wallDistance = 60;
        ros::Duration turningDuration;
        ros::Time turningStarted;

        float linear = 0.2;
        float angle = 0.5;

        int rotationPossibilities[360]; // 0 - 359


        void updateRotationPossibilities(int offset, int length, int math);


    public:

        void init(ros::NodeHandle * nodeHandle);
        void addTile(int x, int y, int type);
        GridPoint * getTile(int x, int y);
        void setRotation(float orientationZ, float angularZ);
        void setCurrentPosition(int x, int y);
        void setCameraDepth(float depth);
        void setUltrasoneSensorDistance(int sensor, int distance);

        static int calculateGridDistance(float input);
        void drive();
        void resetRotationPossibilities();
        void increaseRotationPossibilities(int offset, int length, int steps);
        void decreaseRotationPossibilities(int index, int length, int steps);
        void printRotationPossibilities();
        int getRotationDirection();

};

#endif