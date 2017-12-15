#include "robot.h"

void Robot::init(ros::NodeHandle * nodeHandle) {
    gridFieldPublisher          = nodeHandle->advertise<GridPoint>("/grid_field", 100);
    currentLocationPublisher    = nodeHandle->advertise<GridPoint>("/location", 100);
    cmd_vel_publisher           = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

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

        turnDirection = 1; // turn a side, makes no difference if left or right
        turningDuration = ros::Duration((float)backDegrees / 180 * (M_PI / angle)); // calculate time needed to rotate.

        ROS_INFO_STREAM("Will rotate " << turningDuration.toSec() * angle / M_PI * 180 << " degrees because of gap.");

        hasRotatedBecauseOfGap = true;
        // Let the rotation 'function' make the rotation
        isRotating = true;
        // set start-time of rotation
        turningStarted = ros::Time::now();
    }

    if(!driveForward && isStopped && mustRotate && endOfGap && hasRotatedBecauseOfGap) {
        mustRotate = false;

        // Rotate to gap
        float backDegrees = degrees + 90;
        if(backDegrees > 359)
            backDegrees -= 360;

        if(gapIsRight) {
            turnDirection = -1;
        } else if(gapIsLeft) {
            turnDirection = 1;
        } else {
            ROS_INFO_STREAM("No gap found ????");
        }

        turningDuration = ros::Duration((float)backDegrees / 180 * (M_PI / angle)); // calculate time needed to rotate.
        ROS_INFO_STREAM("Will rotate " << turningDuration.toSec() * angle / M_PI * 180 << " degrees to gap.");

        isRotating = true;

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

        printRotationPossibilities();

        int degreesOfRotation = getRotationDirection();

        // Check if we are going backwards. If so we are driving on a road we already discovered.
        if(degreesOfRotation >= 135 && degreesOfRotation <= 225) {
            isDrivingKnownPath = true;
        }

        if(degreesOfRotation == -1) {
            ROS_INFO_STREAM("Cannot rotate.");
            return;
        }

        ROS_INFO_STREAM("rotate to: " << degreesOfRotation);

        if(degreesOfRotation > 180) {
            turnDirection = 1;
            degreesOfRotation -= 180;
        } else {
            turnDirection = -1;
        }

        turningDuration = ros::Duration(0.65 * ((float)degreesOfRotation / 90) * (M_PI / angle));

        ROS_INFO_STREAM("Will rotate " << turningDuration.toSec() * angle / M_PI * 180 << " degrees.");
        isRotating = true;

        turningStarted = ros::Time::now();



    }

    ROS_INFO_STREAM("!driveForward: " << !driveForward << " isStopped: " << isStopped << " isRotating: " << isRotating);

    /*if(!driveForward && isStopped && isRotating) {
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
    }*/
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

bool Robot::rotateBy(int degrees, int clockwise) {
    
}







bool Robot::turnOdom(bool clockwise, double radians) {
    bool done = false;

    ros::Rate spin_rate(10);
    while (!done && ros::ok()) {
        ros::spinOnce();
        spin_rate.sleep();

        if(hasOrientation) {
            tf::Quaternion q;
            tf::quaternionMsgToTF(orientation, q);

            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            std::cout << "Yaw: " << yaw << " Degrees: " << (yaw / M_PI * 180) << std::endl;
        }
    }

    return false;



    /*tf::TransformListener listener;


    *//*while(radians < 0)
        radians += 2*M_PI;
    while(radians > 2*M_PI)
        radians -= 2*M_PI;*//*

    radians = fmod(fmod(radians, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
    if (radians > M_PI)
        radians -= 2.0 *M_PI;

    //wait for the listener to get the first message
    listener.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform("base_footprint", "odom", ros::Time(0), start_transform);

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 1;
    if (clockwise)
        base_cmd.angular.z = -base_cmd.angular.z;

    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise)
        desired_turn_axis = -desired_turn_axis;

    ros::Rate rate(10.0);
    bool done = false;
    while (!done && ros::ok()) {
        //send the drive command
        cmd_vel_publisher.publish(base_cmd);
        //rate.sleep();
        //get the current transform
        try {
            listener.lookupTransform("base_footprint", "tf", ros::Time(0), current_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            break;
        }

        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();

        double angle_turned = relative_transform.getRotation().getAngle();



        //tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3 m(current_transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        std::cout << "Yaw: " << yaw << " Degrees: " << (yaw / M_PI * 180) << std::endl;


        tf::Matrix3x3 m2(start_transform.getRotation());
        double roll2, pitch2, yaw2;
        m2.getRPY(roll2, pitch2, yaw2);
        std::cout << "Yaw: " << yaw2 << " Degrees: " << (yaw2 / M_PI * 180) << std::endl;

        double angleRotated = yaw - yaw2;

        ROS_INFO_STREAM("ROTATED ####: " << angleRotated);


        //angle_turned = yaw;



        ROS_INFO_STREAM("angle_turned: " << angle_turned);

        if(fabs(angle_turned) < 1.0e-2)
            continue;

        if(actual_turn_axis.dot( desired_turn_axis ) < 0)
            angle_turned = 2 * M_PI - angle_turned;

            if (angle_turned > radians)
                done = true;
    }

    if (done)
        return true;
    return false;*/
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
