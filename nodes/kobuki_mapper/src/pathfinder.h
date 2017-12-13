// a* pathfinding:  https://www.raywenderlich.com/4946/introduction-to-a-pathfinding

#ifndef __PATHFINDER_H_INCLUDED__
#define __PATHFINDER_H_INCLUDED__

#include <kobuki_mapper/GridPoint.h>
#include "square.h"

using namespace std;
using namespace kobuki_mapper;

class Pathfinder {

    private:
        vector<Square> openList;
        vector<Square> closedList;
        vector<GridPoint> grid;

        int currentX;
        int currentY;

        int getPointWithLowestF();

    public:
        void setCurrentLocation(int x, int y);
        void calculatePath();
        void setDataSet(vector<GridPoint> grid);

};

#endif