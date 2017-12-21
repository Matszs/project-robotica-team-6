#ifndef __ROTATION_H_INCLUDED__
#define __ROTATION_H_INCLUDED__

#include <ros/ros.h>
#define DEFAULT_ROTATION_VALUE 5
using namespace std;

class Robot;

class Rotation {

    private:
        Robot * robot;
        int rotationPossibilities[360]; // 0 - 359

    public:
        Rotation(Robot * robot);
        void reset();
        void update(int index, int length, int math);
        void decrease(int index, int length, int steps);
        void increase(int index, int length, int steps);
        void print();
        int getDirection();


};

#endif