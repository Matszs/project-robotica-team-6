#include "robot.h"
#include "rotation.h"
#include "map.h"

void Robot::init(ros::NodeHandle * nodeHandle) {
    currentLocationPublisher    = nodeHandle->advertise<GridPoint>("/location", 100);
    cmd_vel_publisher           = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    degrees_publisher           = nodeHandle->advertise<std_msgs::Float64>("/degrees", 1);
    time_publisher              = nodeHandle->advertise<std_msgs::Int32>("/time", 1); // TODO INT?
    speed_publisher             = nodeHandle->advertise<std_msgs::Float64>("/speed", 1);
    info_publisher              = nodeHandle->advertise<Info>("/info", 1);
    sounds                      = nodeHandle->advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 100);

    startTime = ros::Time::now();

    _map = new Map(nodeHandle, this);
    _rotation = new Rotation(this);

    (*rotation()).reset();
}

void Robot::publishTime() {
    int counter = ros::Time::now().toSec() - startTime.toSec();

    ROS_INFO_STREAM("time: " << counter);

    if(timeBuffer != counter) {
        std_msgs::Int32 runningTime;
        runningTime.data = counter;
        timeBuffer = counter;
        time_publisher.publish(runningTime);
    }
}

void Robot::runTasks(){
    Info info;
    info.time = ros::Time::now().toSec() - startTime.toSec();
    info.speed = speedBuffer;
    info.degrees = getDegrees();
    info.battery = batteryBuffer;

    info_publisher.publish(info);
}

void Robot::setSpeed(float speed){
    speedBuffer = speed;
}

void Robot::setBatteryPercentage(int batteryPercentage){
    batteryBuffer = batteryPercentage;
}

void Robot::setOrientation(geometry_msgs::Quaternion orientation) {
    this->orientation = orientation;
    hasOrientation = true;
}

void Robot::setRotation(float orientationZ, float angularZ) {
    // get 2 decimals from the current rotation
    degrees = (float)((int)(orientationZ * 100)) / 100;
    // format rotation into degrees
    degrees = (degrees > 0 ? 360 - (360 * degrees / 2) : 360 * abs(degrees) / 2);

    this->angularZ = angularZ;
}

int Robot::getCurrentX() {
    return currentX;
}

int Robot::getCurrentY() {
    return currentY;
}

void Robot::setCurrentPosition(int x, int y) {
    GridPoint * gridPoint = (*map()).getTile(x, y);

    if(gridPoint == nullptr) {
        (*map()).addTile(x, y, 1); // 1 because it is the current position, so it is walkable.
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

void Robot::findGap() {

    int front = ultrasoneValues[0];
    int back = ultrasoneValues[2];
    int left = ultrasoneValues[3];
    int right = ultrasoneValues[1];


    // Are we already at the end of a gap?
    ROS_INFO_STREAM("!endOfGap " << !endOfGap);
    if(!endOfGap) {
        // START OF RIGHT
        // Check if we already set the start-point of a gap for the right side.
        ROS_INFO_STREAM("!gapRightStartFound " << !gapRightStartFound);
        ROS_INFO_STREAM("right < (distanceToRight + WALL_DISTANCE_MIN) || front <= wallDistance " << (right < (distanceToRight + WALL_DISTANCE_MIN) || front <= wallDistance));
        if(!gapRightStartFound) {
            // if the distance to the right is bigger then the WALL_DISTANCE_MIN, there is a gap
            ROS_INFO_STREAM("right > (distanceToRight + WALL_DISTANCE_MIN) " << (right > (distanceToRight + WALL_DISTANCE_MIN)));
            if(right > (distanceToRight + WALL_DISTANCE_MIN)) {
                GridPoint gridPoint;
                    gridPoint.x = currentX;
                    gridPoint.y = currentY;
                    gridPoint.z = 0;
                    gridPoint.type = 1;

                // save the start-point of the gap
                gapRightStart = gridPoint;
                gapRightStartFound = true;
            }
        } else if(right < (distanceToRight + WALL_DISTANCE_MIN) || front <= wallDistance) { // if the right-side has found the 'end' of the gap ...
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
        ROS_INFO_STREAM("!gapLeftStartFound " << !gapLeftStartFound);
        if(!gapLeftStartFound) {
            // if the distance to the left is bigger then the WALL_DISTANCE_MIN, there is a gap
            ROS_INFO_STREAM("left > (distanceToLeft + WALL_DISTANCE_MIN) " << (left > (distanceToLeft + WALL_DISTANCE_MIN)));
            ROS_INFO_STREAM("left < (distanceToLeft + WALL_DISTANCE_MIN) || front <= wallDistance " << (left < (distanceToLeft + WALL_DISTANCE_MIN) || front <= wallDistance));
            if(left > (distanceToLeft + WALL_DISTANCE_MIN)) {
                GridPoint gridPoint;
                    gridPoint.x = currentX;
                    gridPoint.y = currentY;
                    gridPoint.z = 0;
                    gridPoint.type = 1;

                // save the start-point of the gap
                gapLeftStart = gridPoint;
                gapLeftStartFound = true;
            }
        } else if(left < (distanceToLeft + WALL_DISTANCE_MIN) || front <= wallDistance) { // if the left-side has found the 'end' of the gap ...
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
        ROS_INFO_STREAM("!hasRotatedBecauseOfGap " << !hasRotatedBecauseOfGap);
        if(!hasRotatedBecauseOfGap) {
            driveForward = false;
        } else {
            // we already have rotated the robot, now lets drive it forward till the center of the gap.
            // Check if we have calculated the time needed to drive forward.
            ROS_INFO_STREAM("driveToGapDuration.isZero() " << driveToGapDuration.isZero());
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

            ROS_INFO_STREAM("(ros::Time::now() - driveToGapStartTime): " << (ros::Time::now().toSec() - driveToGapStartTime.toSec()));
            ROS_INFO_STREAM("driveToGapDuration: " << driveToGapDuration.toSec());
            ROS_INFO_STREAM("(ros::Time::now() - driveToGapStartTime) < driveToGapDuration: " << ((ros::Time::now().toSec() - driveToGapStartTime.toSec()) < driveToGapDuration.toSec()));

            if ((ros::Time::now().toSec() - driveToGapStartTime.toSec()) < driveToGapDuration.toSec()) {
                driveForward = false;
            }
        }
    }
}

void Robot::driveAutonomous() {
    (*rotation()).reset();

    int front = ultrasoneValues[0];
    int back = ultrasoneValues[2];
    int left = ultrasoneValues[3];
    int right = ultrasoneValues[1];

    bool canRideForward = (front > wallDistance);
    bool canRideBackward = (back > wallDistance);
    bool canTurnLeft = (left > wallDistance);
    bool canTurnRight = (right > wallDistance);

    ROS_INFO_STREAM("Robot: bumper links: " << getBumperState(0) << " bumper midden: " << getBumperState(1) << " bumper rechts: " << getBumperState(2));

    if(driveForward) {
        if(!canRideForward || getBumperStates()) {
            driveForward = false;
        } else {
            findGap();

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

    }

    // If we can't drive forward anymore because the front-sensor has detected an obstacle.
    if(!driveForward && isStopped && mustRotate) {
        mustRotate = false;

        // determine if the robot can rotate to specific directions, otherwise substract 2
        if(!canTurnRight)
            (*rotation()).decrease(225, 90, 2);
        if(!canTurnLeft)
            (*rotation()).decrease(45, 90, 2);
        if(!canRideForward)
            (*rotation()).decrease(315, 90, 2);
        if(!canRideBackward)
            (*rotation()).decrease(135, 90, 2);

        // Check bumper states and subtract 1 from the rotation possibilities
        if(getBumperState(0))
            (*rotation()).decrease(225, 90, 1);
        if(getBumperState(1))
            (*rotation()).decrease(315, 90, 1);
        if(getBumperState(2))
            (*rotation()).decrease(45, 90, 1);

        // check which direction we already visited.
        enum direction d;
        for(int d = FRONT; d <= LEFT; d++) {
            ROS_INFO_STREAM("direction: " << d << " <-> " << (enum direction)d);

            if((*map()).checkTileDirection((enum direction)d, getDegrees())) {
                int degreestToBlockFrom = 315 + (90 * d);

                if(degreestToBlockFrom > 359)
                    degreestToBlockFrom -= 360;

                (*rotation()).decrease(degreestToBlockFrom, 90, 1);
            }

        }

        // Print out all the degrees and their score.
        (*rotation()).print();

        // Get one direction from the possible directions based on the score.
        degreesAddRotation = (*rotation()).getDirection();
        turnDirection = 1;

        if(degreesAddRotation == -1) {
            ROS_INFO_STREAM("Cannot rotate.");
            return;
        }

        ROS_INFO_STREAM("rotate by: " << degreesAddRotation);

        isRotating = true;
    }

    ROS_INFO_STREAM("!driveForward: " << !driveForward << " isStopped: " << isStopped << " isRotating: " << isRotating);

    if(!driveForward && isStopped && isRotating) {
        bool isDoneRotating = false;

        if(degreesOfRotation != -1 && rotateTo(degreesOfRotation))
            isDoneRotating = true;
        if(degreesAddRotation != -1 && rotateBy(degreesAddRotation, turnDirection))
            isDoneRotating = true;

        if(isDoneRotating) {
            isRotating = false;
            driveForward = true;
            isStopped = false;

            degreesOfRotation = -1;
            degreesAddRotation = -1;

            distanceToLeft = left;
            distanceToRight = right;
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
    return rotateTo(degrees, true); // default fix numbers?
}

bool Robot::rotateTo(int degrees, bool fixDegrees) {

    if(fixDegrees) {
        if(degrees > 87 && degrees < 93)
            degrees = 90;
        else if(degrees > 177 && degrees < 183)
            degrees = 180;
        else if(degrees > 267 && degrees < 273)
            degrees = 270;
        else if(degrees > 357 && degrees < 3)
            degrees = 0;
    }


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

void Robot::driveByPath() {
    // TODO: pathfinding etc
}

void Robot::drive() {
    if(driveToPoint) {
        driveByPath();
    } else {
        driveAutonomous();
    }
}

void Robot::calculatePath() {
    Pathfinder pathfinder;
    pathfinder.setCurrentLocation(currentX, currentY);
    pathfinder.setDataSet(grid);

    pathfinder.calculatePath();
}

Map * Robot::map() {
    return _map;
}

Rotation * Robot::rotation() {
    return _rotation;
}

// STATIC

int Robot::calculateGridDistance(float input) {
    return (int)(-input * 10);
}