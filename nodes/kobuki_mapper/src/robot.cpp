#include "robot.h"

void Robot::init(ros::NodeHandle * nodeHandle) {
    gridFieldPublisher          = nodeHandle->advertise<GridPoint>("/grid_field", 100);
    currentLocationPublisher    = nodeHandle->advertise<GridPoint>("/location", 100);
    cmd_vel_publisher           = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    resetRotationPossibilities();
}

GridPoint* Robot::getTile(int x, int y) {
    unsigned int i;
    for (i = 0; i < grid.size(); i++) {
        GridPoint gridPoint = grid.at(i);

        if(gridPoint.x == x && gridPoint.y == y)
            return &(grid.at(i));
    }

    return nullptr;
}

void Robot::addTile(int x, int y, int type) {
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

void Robot::setRotation(float orientationZ, float angularZ) {
    // get 2 decimals from the current rotation
    degrees = (float)((int)(orientationZ * 100)) / 100;
    // format rotation into degrees
    degrees = (degrees > 0 ? 360 - (360 * degrees / 2) : 360 * abs(degrees) / 2);

    this->angularZ = angularZ;
}

void Robot::setCurrentPosition(int x, int y) {
    GridPoint * gridPoint = getTile(x, y);

    if(gridPoint == nullptr) {
        addTile(x, y, 1); // 1 because it is the current position, so it is walkable.
    }

    if(x != currentX || y != currentY) {
        currentX = x;
        currentY = y;

        GridPoint gridPointLocation;
        gridPointLocation.x = currentX;
        gridPointLocation.y = currentY;
        gridPointLocation.z = 0;
        gridPointLocation.type = 2; // 1 = walkable, 0 = not walkable, 2 = current location

        // Publish current location to other nodes.
        currentLocationPublisher.publish(gridPointLocation);
    }
}


void Robot::setCameraDepth(float depth) {
    cameraDepth = depth;
}

void Robot::setUltrasoneSensorDistance(int sensor, int distance) {
    ultrasoneValues[sensor] = distance;
}

void Robot::drive() {
    resetRotationPossibilities();

    bool canRideForward = (ultrasoneValues[0] > wallDistance);
    bool canRideBackward = (ultrasoneValues[2] > wallDistance);
    bool canTurnLeft = (ultrasoneValues[3] > wallDistance);
    bool canTurnRight = (ultrasoneValues[1] > wallDistance);

    if(driveForward) {
        if(!canRideForward) {
            driveForward = false;
        }

        geometry_msgs::TwistPtr cmd_vel_msg_ptr;
        cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

        cmd_vel_msg_ptr->linear.x = linear;
        cmd_vel_publisher.publish(cmd_vel_msg_ptr);
    } else {
        if(!isStopped) {
            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);

            isStopped = true;
            mustRotate = true;
        }
    }

    if(!driveForward && isStopped && mustRotate) {
        mustRotate = false;

        if(!canTurnRight)
            decreaseRotationPossibilities(225, 90, 2);
        if(!canTurnLeft)
            decreaseRotationPossibilities(45, 90, 2);
        if(!canRideForward)
            decreaseRotationPossibilities(315, 90, 2);
        if(!canRideBackward)
            decreaseRotationPossibilities(135, 90, 2);

        // TODO: Filter grid & bumper

        int degreesOfRotation = getRotationDirection();

        if(degreesOfRotation == -1) {
            ROS_INFO_STREAM("Cannot rotate.");
            return;
        }

        ROS_INFO_STREAM("rotate to: " << degreesOfRotation);

        if(degreesOfRotation > 180) {
            turnDirection = -1;
            degreesOfRotation =- 180;
        } else {
            turnDirection = 1;
        }

        turningDuration = ros::Duration(degreesOfRotation / 180 * (M_PI / angle));

        ROS_INFO_STREAM("Will rotate " << turningDuration.toSec() * angle / M_PI * 180 << " degrees.");
        isRotating = true;

        turningStarted = ros::Time::now();
    }

    if(!driveForward && isStopped && isRotating) {
        if ((ros::Time::now() - turningStarted) < turningDuration) {

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->angular.z = turnDirection * angle;
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);

        } else {

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);

            isRotating = false;

            // Reset turn sides
            turnLeft = false;
            turnRight = false;

            // Reset drive settings
            driveForward = true;
            isStopped = false;
        }
    }














}

void Robot::resetRotationPossibilities() {
    fill(begin(rotationPossibilities), end(rotationPossibilities), 5); // set all values to 5.
}

void Robot::updateRotationPossibilities(int index, int length, int math) {
    for( unsigned int i = index; i < (index + length); i++ ) {
        int degrees = i;
        if(i >= (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])) - 1)
            degrees =- 360;

        if(math == 0) {
            rotationPossibilities[degrees] = 0;
        } else {
            rotationPossibilities[degrees] += math;
        }

    }
}

void Robot::increaseRotationPossibilities(int index, int length, int steps) {
    updateRotationPossibilities(index, length, 1 * steps);
}

void Robot::decreaseRotationPossibilities(int index, int length, int steps) {
    updateRotationPossibilities(index, length, -1 * steps);
}

void Robot::printRotationPossibilities() {
    for( unsigned int i = 0; i < (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])); i++ ) {
        ROS_INFO_STREAM("rotationPossibilities " << i << " = " << rotationPossibilities[i]);
    }
}

int Robot::getRotationDirection() {
    std::vector<int> rotationDegrees;
    int highestValue = 0;

    for( unsigned int i = 0; i < (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])); i++ ) {
        int degrees = i + 1;
        int value = rotationPossibilities[i];

        if(value > highestValue) {
            rotationDegrees.clear();
            rotationDegrees.push_back(degrees);
            highestValue = value;
        } else if(value == highestValue) {
            rotationDegrees.push_back(degrees);
        }
    }

    if(rotationDegrees.size() == 0)
        return -1;

    if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 0) != rotationDegrees.end() )
        return 0;
    if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 90) != rotationDegrees.end() )
        return 90;
    else if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 180) != rotationDegrees.end() )
        return 180;
    else if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 270) != rotationDegrees.end() )
        return 270;

    if(rotationDegrees.size() > 90)
        return rotationDegrees.at(rotationDegrees.size() / 4);
    else
        return rotationDegrees.at(rotationDegrees.size() / 2);
}

// STATIC

int Robot::calculateGridDistance(float input) {
    return (int)(-input * 10);
}
