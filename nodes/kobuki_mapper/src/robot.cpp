#include "robot.h"

void Robot::init(ros::NodeHandle * nodeHandle) {
    gridFieldPublisher          = nodeHandle->advertise<GridPoint>("/grid_field", 100);
    currentLocationPublisher    = nodeHandle->advertise<GridPoint>("/location", 100);
    cmd_vel_publisher           = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    degrees_publisher           = nodeHandle->advertise<std_msgs::Float64>("/degrees", 1);

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

void Robot::setOrientation(geometry_msgs::Quaternion orientation) {
    this->orientation = orientation;
    hasOrientation = true;

    std_msgs::Float64 degrees;
    degrees.data = getDegrees();

    degrees_publisher.publish(degrees);
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

void Robot::setBumperState(int index, bool state){
    bumper[index] = state;
}

bool Robot::getBumperStates(){
    return (bumper[0] || bumper[1] || bumper[2]);
}

bool Robot::getBumperState(int index){
    return bumper[index];
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
        ROS_INFO_STREAM("BUMPER 0: " << getBumperState(0) << " BUMPER 1: " << getBumperState(1) << " BUMPER 2: " << getBumperState(2));

        if(!canRideForward || getBumperStates()) {
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
                        if(driveToGapDuration.isZero()) {

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
                            distanceY -= currentY;

                            // calculate relative distance btween new position and current position
                            float totalDistance = sqrt(pow(distanceX - currentX, 2) + pow(distanceY - currentY, 2));
                            ROS_INFO_STREAM("Will drive forward " << (totalDistance * 10) << " meter to gap.");

                            driveToGapDuration = ros::Duration(linear * (totalDistance * 10)); // calculate time needed to drive forward.
                            ROS_INFO_STREAM("Which is " << driveToGapDuration.toSec() << " seconds.");
                            driveToGapStartTime = ros::Time::now();
                        }

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

        // set degrees to rotate to.
        degreesOfRotation = backDegrees;

        hasRotatedBecauseOfGap = true;
        // Let the rotation 'function' make the rotation
        isRotating = true;
    }

    if(!driveForward && isStopped && mustRotate && endOfGap && hasRotatedBecauseOfGap) {
        mustRotate = false;

        // Rotate to gap
        float backDegrees = degrees + 90;
        if(backDegrees > 359)
            backDegrees -= 360;

        isRotating = true;
        // set degrees to rotate to.
        degreesOfRotation = backDegrees;

        // Reset all settings of gap-finder because we are done.
        gapIsRight = false;
        gapIsLeft = false;

        gapRightStartFound = false;
        gapLeftStartFound = false;

        endOfGap = false;
        hasRotatedBecauseOfGap = false;

        isDrivingKnownPath = false;

    }

    // If we can't drive forward anymore because the front-sensor has detected an obstacle.
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

        // Check bumper states and subtract 1 from the rotation possibilities
        if(getBumperState(0))
            decreaseRotationPossibilities(225, 90, 1);
        if(getBumperState(1))
            decreaseRotationPossibilities(315, 90, 1);
        if(getBumperState(2))
            decreaseRotationPossibilities(45, 90, 1);


        // check if the tile behind the robot is already visited.
        float backDegrees = getDegrees() + 180;
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

            ROS_INFO_STREAM("gridPoint!");
        } else {
            ROS_INFO_STREAM("NO gridPoint!");
        }


        // TODO: Filter grid & bumper

        printRotationPossibilities();

        degreesAddRotation = getRotationDirection();
        turnDirection = 1;

        ROS_INFO_STREAM("DG: " << degreesAddRotation);

        // Check if we are going backwards. If so we are driving on a road we already discovered.
        if(degreesAddRotation >= 135 && degreesAddRotation <= 225) {
            isDrivingKnownPath = true;
        }

        if(degreesAddRotation == -1) {
            ROS_INFO_STREAM("Cannot rotate.");
            return;
        }

        ROS_INFO_STREAM("rotate by: " << degreesAddRotation);

        isRotating = true;
    }

    ROS_INFO_STREAM("!driveForward: " << !driveForward << " isStopped: " << isStopped << " isRotating: " << isRotating);

    if(!driveForward && isStopped && isRotating) {
        if(degreesOfRotation != -1) {
            if(rotateTo(degreesOfRotation)) {
                isRotating = false;
                driveForward = true;
                isStopped = false;

                degreesOfRotation = -1;
                degreesAddRotation = -1;
            }
        }
        if(degreesAddRotation != -1) {
            if(rotateBy(degreesAddRotation, turnDirection)) {
                isRotating = false;
                driveForward = true;
                isStopped = false;

                degreesOfRotation = -1;
                degreesAddRotation = -1;
            }
        }
    }
}

double Robot::getDegrees() {
    tf::Quaternion q;
    tf::quaternionMsgToTF(orientation, q);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double degrees = (yaw / M_PI * 180);
    if(degrees < 0)
        degrees = abs(degrees);
    else
        degrees = 360 - degrees;

    return degrees;
}

bool Robot::rotateTo(int degrees) {
    bool isDone = false;

    if(hasOrientation) {
        double currentDegrees = getDegrees();

        if(startDegrees == -1)
            startDegrees = currentDegrees;

        ROS_INFO_STREAM("start: " << startDegrees << " current: " << currentDegrees << " degrees: " << degrees);


        float absoluteDistanceStart = (((((int)degrees - (int)startDegrees) % 360) + 540) % 360) - 180;
        float absoluteDistanceCurrent = (((((int)degrees - (int)currentDegrees) % 360) + 540) % 360) - 180;

        bool clockwise = (absoluteDistanceStart >= 0);

        ROS_INFO_STREAM("clockwise: " << clockwise);


        float distance = abs(absoluteDistanceCurrent) / abs(absoluteDistanceStart);
        float speed = 1.4 * distance;
        ROS_INFO_STREAM("speed: " << speed);

        if(speed < 0.2)
            speed = 0.2;
        if(speed > 1)
            speed = 1;

        ROS_INFO_STREAM("distance: " << distance);

        geometry_msgs::Twist base_cmd;
        base_cmd.linear.x = base_cmd.linear.y = 0.0;
        base_cmd.angular.z = speed;
        if (clockwise)
            base_cmd.angular.z = -base_cmd.angular.z;

        cmd_vel_publisher.publish(base_cmd);

        if(abs(absoluteDistanceCurrent) < 10) {
            if(clockwise) {
                if(currentDegrees >= degrees) {
                    isDone = true;
                    startDegrees = -1;
                }
            } else {
                if(currentDegrees <= degrees) {
                    isDone = true;
                    startDegrees = -1;
                }
            }

            if(!isDone && abs(absoluteDistanceCurrent) < 1) {
                isDone = true;
                startDegrees = -1;
            }
        }

        if(isDone) {
            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);
        }
    }

    ROS_INFO_STREAM("isDone " << isDone);

    return isDone;
}

bool Robot::rotateBy(int degrees, bool clockwise) {
    if(!hasOrientation) return false;

    double currentDegrees = getDegrees();
    double positionToRotateTo = 0;

    if(startRelativeDegrees == -1)
        startRelativeDegrees = currentDegrees;

    if(clockwise){
        positionToRotateTo = startRelativeDegrees + degrees;
    } else {
        positionToRotateTo = startRelativeDegrees - degrees;
    }

    if(positionToRotateTo > 360)
        positionToRotateTo = (int)positionToRotateTo % 360;
    if(positionToRotateTo < 0)
        positionToRotateTo +=360;

    ROS_INFO_STREAM("currentDegrees: " << currentDegrees << " positionToRotateTo: " << positionToRotateTo << " degrees: " << degrees << " clockwise: " << clockwise << " startRelativeDegrees: " << startRelativeDegrees);

    if(rotateTo(positionToRotateTo)){
        startRelativeDegrees = -1;
        return true;
    }
    return false;
}

void Robot::drive_to_point() {
    // TODO: pathfinding etc
}

void Robot::drive() {
    if(driveToPoint) {
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
        if(i >= (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])))
            degreesIndex -= 360;

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

void Robot::calculatePath() {

    Pathfinder pathfinder;
    pathfinder.setCurrentLocation(currentX, currentY);
    pathfinder.setDataSet(grid);

    pathfinder.calculatePath();

}

// STATIC

int Robot::calculateGridDistance(float input) {
    return (int)(-input * 10);
}
