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

void Robot::drive_autonomous() {
    resetRotationPossibilities();

    int front = ultrasoneValues[0];
    int back = ultrasoneValues[2];
    int left = ultrasoneValues[3];
    int right = ultrasoneValues[1];

    bool canRideForward = (front > wallDistance);
    bool canRideBackward = (back > wallDistance);
    bool canTurnLeft = (left > wallDistance);
    bool canTurnRight = (right > wallDistance);

    if(driveForward) {
        if(!canRideForward) {
            driveForward = false;
        } else {

            /*
            todo:   check here if the tile in front of us is already visited.
                    If that is the case, and the robot is driving forward, we should check if the robot can drive
                    right of left. (fixed number?)
                    save the start and the end of the 'gap'. Ride back to the CENTER of the gap and make a rotation and go on.
            */

            // Check if we are driving over a path which we already mapped.
            if(isDrivingKnownPath) {
                // Are we already at the end of a gap?
                if(!endOfGap) {
                    // START OF RIGHT

                    // Check if we already set the start-point of a gap for the right side.
                    if(!gapRightStartFound) {
                        // if the distance to the right is bigger then the WALL_DISTANCE_MIN, there is a gap
                        if(right > WALL_DISTANCE_MIN) {
                            GridPoint gridPoint;
                                gridPoint.x = currentX;
                                gridPoint.y = currentY;
                                gridPoint.z = 0;
                                gridPoint.type = 1;

                            // save the start-point of the gap
                            gapRightStart = gridPoint;
                            gapRightStartFound = true;
                        }
                    } else if(right < WALL_DISTANCE_MIN) { // if the right-side has found the 'end' of the gap ...
                         GridPoint gridPoint;
                             gridPoint.x = currentX;
                             gridPoint.y = currentY;
                             gridPoint.z = 0;
                             gridPoint.type = 1;


                         // ... we can now save this as the end of the gap.
                         gapRightEnd = gridPoint;

                         // and because we have found a gap at the right, we have to save that
                         gapIsRight = true;
                         endOfGap = true;
                     }

                    // END OF RIGHT

                    // START OF LEFT

                    // Check if we already set the start-point of a gap for the left side.
                    if(!gapLeftStartFound) {
                        // if the distance to the left is bigger then the WALL_DISTANCE_MIN, there is a gap
                        if(left > WALL_DISTANCE_MIN) {
                            GridPoint gridPoint;
                                gridPoint.x = currentX;
                                gridPoint.y = currentY;
                                gridPoint.z = 0;
                                gridPoint.type = 1;

                            // save the start-point of the gap
                            gapLeftStart = gridPoint;
                            gapLeftStartFound = true;
                        }
                    } else if(left < WALL_DISTANCE_MIN) { // if the left-side has found the 'end' of the gap ...
                         GridPoint gridPoint;
                             gridPoint.x = currentX;
                             gridPoint.y = currentY;
                             gridPoint.z = 0;
                             gridPoint.type = 1;


                         // ... we can now save this as the end of the gap.
                         gapLeftEnd = gridPoint;

                         // and because we have found a gap at the left, we have to save that
                         gapIsLeft = true;
                         endOfGap = true;
                     }

                     // END OF LEFT
                } else {

                    // GAP FOUND, DO SOMETHING



                    // check if we already rotated the robot
                    if(!hasRotatedBecauseOfGap) {
                        driveForward = false;
                    } else {
                        // we already have rotated the robot, now lets drive it forward till the center of the gap.
                        // Check if we have calculated the time needed to drive forward.
                        if(!driveToGapDuration) {

                            int distanceX = 0;
                            int distanceY = 0;

                            if(gapIsLeft) {
                                distanceX = gapLeftEnd.x - gapLeftStart.x;
                                distanceY = gapLeftEnd.y - gapLeftStart.y;
                            } else {
                                distanceX = gapRightEnd.x - gapRightStart.x;
                                distanceY = gapRightEnd.y - gapRightStart.y;
                            }

                            distanceX -= currentX;
                            distanceY -= currentY

                            driveToGapDuration = ros::Duration((M_PI / linear)); // calculate time needed to drive forward.
                            driveToGapStartTime = ros::Time::now();
                        };


                        if ((ros::Time::now() - driveToGapStartTime) < driveToGapDuration) {
                            driveForward = false;
                        }
                    }

                }


            }

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->linear.x = linear;
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);

        }
    } else {
        if(!isStopped) {
            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);

            isStopped = true;
            mustRotate = true;
        }
    }

    // If we have found a gap while driving back, we go back to that gap.
    if(!driveForward && isStopped && mustRotate && endOfGap && !hasRotatedBecauseOfGap) {
        mustRotate = false;

        // we rotate 180 degrees because the robot was driving forward while the gap was found.
        float backDegrees = degrees + 180;
        if(backDegrees > 359)
            backDegrees -= 360;

        turnDirection = 1; // turn a side, makes no difference if left or right
        turningDuration = ros::Duration(backDegrees / 180 * (M_PI / angle)); // calculate time needed to rotate.

        ROS_INFO_STREAM("Will rotate " << turningDuration.toSec() * angle / M_PI * 180 << " degrees because of gap.");

        hasRotatedBecauseOfGap = true;
        // Let the rotation 'function' now it should rotate
        isRotating = true;
        // set start-time of rotation
        turningStarted = ros::Time::now();
    }

    if(!driveForward && isStopped && mustRotate && endOfGap && hasRotatedBecauseOfGap) {

        // TODO: rotate to gap

    }

    // If we can't drive forward anymore because the front-sensor has detected a obstacle.
    if(!driveForward && isStopped && mustRotate) {
        mustRotate = false;

        // determine if the robot can rotate to specific directions
        if(!canTurnRight)
            decreaseRotationPossibilities(225, 90, 2);
        if(!canTurnLeft)
            decreaseRotationPossibilities(45, 90, 2);
        if(!canRideForward)
            decreaseRotationPossibilities(315, 90, 2);
        if(!canRideBackward)
            decreaseRotationPossibilities(135, 90, 2);


        // check if the tile behind the robot is already visited.
        float backDegrees = degrees + 180;
        if(backDegrees > 359)
            backDegrees -= 360;

        int backY = currentY - round(cos(round(backDegrees) * M_PI / 180) * 1); // cos(0 * pi / 180) * 5 = 5
        int backX = currentX + round(sin(round(backDegrees) * M_PI / 180) * 1); // sin(0 * pi / 180) * 5 = 0

        // check if we already visited the tile behind the robot
        GridPoint * gridPoint = getTile(backX, backY);

        // if the tile exists ...
        if(gridPoint != nullptr) {
            // ... we decrease the possibilities of the 'back' tiles by one.
            decreaseRotationPossibilities(135, 90, 1);
        }


        // TODO: Filter grid & bumper

        int degreesOfRotation = getRotationDirection();

        if(degreesOfRotation >= 135 && degreesOfRotation <= 225) {
            isDrivingKnownPath = true;
        }

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

            // Reset drive settings
            driveForward = true;
            isStopped = false;
        }
    }
}

void Robot::drive_to_point() {

}

void Robot::drive() {
    if(driveToPoint) {
        // TODO: pathfinding etc

        drive_to_point();

    } else {
        drive_autonomous();
    }
}

void Robot::resetRotationPossibilities() {
    fill(begin(rotationPossibilities), end(rotationPossibilities), 5); // set all values to 5.
}

void Robot::updateRotationPossibilities(int index, int length, int math) {
    for( unsigned int i = index; i < (index + length); i++ ) {
        int degreesIndex = i;
        if(i >= (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])) - 1)
            degreesIndex =- 360;

        if(math == 0) {
            rotationPossibilities[degreesIndex] = 0;
        } else {
            rotationPossibilities[degreesIndex] += math;
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
        int degreesIndex = i + 1;
        int value = rotationPossibilities[i];

        if(value > highestValue) {
            rotationDegrees.clear();
            rotationDegrees.push_back(degreesIndex);
            highestValue = value;
        } else if(value == highestValue) {
            rotationDegrees.push_back(degreesIndex);
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
