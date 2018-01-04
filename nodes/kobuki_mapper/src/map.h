#ifndef __MAP_H_INCLUDED__
#define __MAP_H_INCLUDED__

#include <kobuki_mapper/GridPoint.h>
#include <ros/ros.h>

enum directions {
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

    public:
        Map(ros::NodeHandle * nodeHandle);
        GridPoint* getTile(int x, int y);
        void addTile(int x, int y, int type);
        bool checkTileDirection(enum directions, double degrees, int currentX, int currentY);


};

#endif