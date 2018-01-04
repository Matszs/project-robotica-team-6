#ifndef __ROTATION_H_INCLUDED__
#define __ROTATION_H_INCLUDED__

#include <ros/ros.h>
#include <kobuki_mapper/Obstacle.h>

#define DEFAULT_ROTATION_VALUE 10
using namespace std;

class Rotation {

    private:
        int rotationPossibilities[360]; // 0 - 359

    public:
        void reset();
        void update(int index, int length, int math);
        void decrease(int index, int length, int steps);
        void increase(int index, int length, int steps);
        void print();
        kobuki_mapper::Obstacle getRotationPossibilitiesObstacle();
        int getDirection();


};

#endif