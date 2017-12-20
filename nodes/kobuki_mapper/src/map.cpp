#include "map.h"

Map:Map(ros::NodeHandle * nodeHandle, Robot * robot) {
    gridFieldPublisher = nodeHandle->advertise<GridPoint>("/grid_field", 100);
    this->robot = &robot;
}

GridPoint* Map::getTile(int x, int y) {
    unsigned int i;
    for (i = 0; i < grid.size(); i++) {
        GridPoint gridPoint = grid.at(i);

        if(gridPoint.x == x && gridPoint.y == y)
            return &(grid.at(i));
    }

    return nullptr;
}

void Map::addTile(int x, int y, int type) {
    // Add grid point
    GridPoint gridPoint;
    gridPoint.x = x;
    gridPoint.y = y;
    gridPoint.z = 0;
    gridPoint.type = type; // 1 = walkable, 0 = not walkable, 2 = current location

    // Add gridpoint to the grid vector
    grid.push_back(gridPoint);
    // Publish the data to other nodes
    gridFieldPublisher.publish(gridPoint);
}

bool Map::checkTileDirection(enum direction d){
    float directionDegrees = getDegrees() + (d * 90);

    if(directionDegrees > 359)
        directionDegrees -= 360;

    ROS_INFO_STREAM("Map: directionDegrees: " << directionDegrees);

    int directionY = currentY - round(cos(round(directionDegrees) * M_PI / 180) * 1); // cos(0 * pi / 180) * 5 = 5
    int directionX = currentX + round(sin(round(directionDegrees) * M_PI / 180) * 1); // sin(0 * pi / 180) * 5 = 0

    ROS_INFO_STREAM("Map: tile: x: " << directionX << " y: " << directionY);

    return (getTile(directionX, directionY) != nullptr)
}