#ifndef __MAP_H_INCLUDED__
#define __MAP_H_INCLUDED__

#include <kobuki_mapper/GridPoint.h>
#include <ros/ros.h>

enum direction {
    FRONT,
    RIGHT,
    BACK,
    LEFT
};

using namespace std;
using namespace kobuki_mapper;

class Map {

    private:
        vector<GridPoint> grid;
        ros::Publisher gridFieldPublisher;
        Robot * robot;
        Map(ros::NodeHandle * nodeHandle, Robot * robot);


    public:
        Map(ros::NodeHandle * nodeHandle, Robot * robot);
        GridPoint* getTile(int x, int y);
        void addTile(int x, int y, int type);
        bool checkTileDirection(enum direction);


};

#endif