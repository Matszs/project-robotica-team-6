#include "pathfinder.h"

void Pathfinder::setCurrentLocation(int x, int y) {
    this->currentX = x;
    this->currentY = y;
}

void Pathfinder::setDataSet(vector<GridPoint> grid) {
    this->grid = grid;
}

int Pathfinder::getPointWithLowestF() {
    /*int lowestF = -1;
    int lowestFValue = 1000;

    for (auto it = openList.begin(); it != openList.end(); ++it) {
        if(it.getF() <= lowestF) {
            lowestF = distance(openList.begin(), it); // get index of element
            lowestFValue = it.getF();
        }
    }

    return lowestF;*/
}

void Pathfinder::calculatePath() {
    if(grid.size() < 0)
        return;

    Square startPosition = Square(this->currentX, this->currentY);
    closedList.push_back(startPosition);

    while(openList.size() > 0) {

        Square currentSquare = openList.at(getPointWithLowestF());


    }

}