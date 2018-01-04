#include "drive.h"

Drive::Drive(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
    rotation = new Rotation();
    rotation->reset();
	velocityPublisher = nodeHandle->advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	obstaclePublisher = nodeHandle->advertise<kobuki_mapper::Obstacle>("/obstacle", 1);

	ROS_INFO_STREAM("Drive:: Module initialized.");
}

void Drive::read() {

	autonomousDriving();
	ROS_INFO_STREAM("Drive:: read.");
}

void Drive::autonomousDriving() {
	if(!(static_cast<Button *>(ModuleLoader::get("button")))->isActive())
	    return;

    bool canDriveForward = ((static_cast<Ultrasonic *>(ModuleLoader::get("ultrasonic")))->getSensorDistance(0) > WALL_DISTANCE);

    if(canDriveForward && newDriveDirection == -1) {
        forward();
    } else {
        if(newDriveDirection == -1)
            newDriveDirection = findDirection();

        if(rotateTo(newDriveDirection)) {
            newDriveDirection = -1;
            ROS_INFO_STREAM("Drive:: >>>>>>>>>>>>>>>> RESET!!!!!!!!!!");
        }
    }
}

void Drive::forward() {
    geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

    cmd_vel_msg_ptr->linear.x = LINEAR;
    velocityPublisher.publish(cmd_vel_msg_ptr);
}

void Drive::stop() {
    geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());
    velocityPublisher.publish(cmd_vel_msg_ptr);
}

int Drive::findDirection() {
    rotation->reset();

    checkUltrasonicSensors();
    checkBumperSensors();
    checkAlreadyVisitedLocations();

    rotation->print();
    publishObstacleData();

    return rotation->getDirection();
}

void Drive::publishObstacleData() {
    obstaclePublisher.publish(rotation->getRotationPossibilitiesObstacle());
}

void Drive::checkAlreadyVisitedLocations() {
    enum directions direction;
    Location * location = (static_cast<Location *>(ModuleLoader::get("location")));

    for(int direction = FRONT; direction <= LEFT; direction++) {

        if(location->getMap()->checkTileDirection((enum directions)direction, location->getDegrees(), location->getCurrentX(), location->getCurrentY())) {
            int degreestToBlockFrom = 315 + (90 * direction);

            if(degreestToBlockFrom > 359)
                degreestToBlockFrom -= 360;

            rotation->decrease(degreestToBlockFrom, 90, 1);
        }
    }
}

void Drive::checkUltrasonicSensors() {
    Ultrasonic * ultrasonic = (static_cast<Ultrasonic *>(ModuleLoader::get("ultrasonic")));

    int front   = ultrasonic->getSensorDistance(0);
    int back    = ultrasonic->getSensorDistance(2);
    int left    = ultrasonic->getSensorDistance(3);
    int right   = ultrasonic->getSensorDistance(1);

    bool canRideForward     = (front > WALL_DISTANCE);
    bool canRideBackward    = (back > WALL_DISTANCE);
    bool canTurnLeft        = (left > WALL_DISTANCE);
    bool canTurnRight       = (right > WALL_DISTANCE);

    // Check if we can go any direction based on ultrasonic sensors
    if(!canTurnRight)
        rotation->decrease(225, 90, 2);
    if(!canTurnLeft)
        rotation->decrease(45, 90, 2);
    if(!canRideForward)
        rotation->decrease(315, 90, 2);
    if(!canRideBackward)
        rotation->decrease(135, 90, 2);
}

void Drive::checkBumperSensors() {
    Bumper * bumper = (static_cast<Bumper *>(ModuleLoader::get("bumper")));

    // Check bumper states and subtract 1 from the rotation possibilities
    if(bumper->getBumperState(0))
        rotation->decrease(225, 90, 1);
    if(bumper->getBumperState(1))
        rotation->decrease(315, 90, 1);
    if(bumper->getBumperState(2))
        rotation->decrease(45, 90, 1);
}






bool Drive::rotateTo(int degrees) {
    return rotateTo(degrees, true); // default fix numbers?
}

bool Drive::rotateTo(int degrees, bool fixDegrees) {

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

	double currentDegrees = (static_cast<Location *>(ModuleLoader::get("location")))->getDegrees();

	if(rotateToStartDegrees == -1)
		rotateToStartDegrees = currentDegrees;

	ROS_INFO_STREAM("Drive:: start: " << rotateToStartDegrees << " current: " << currentDegrees << " degrees: " << degrees);


	float absoluteDistanceStart = (((((int)degrees - (int)rotateToStartDegrees) % 360) + 540) % 360) - 180;
	float absoluteDistanceCurrent = (((((int)degrees - (int)currentDegrees) % 360) + 540) % 360) - 180;

	bool clockwise = (absoluteDistanceStart >= 0);

	ROS_INFO_STREAM("Drive:: clockwise: " << clockwise);


	float distance = abs(absoluteDistanceCurrent) / abs(absoluteDistanceStart);
	float speed = ANGULAR * distance;
	ROS_INFO_STREAM("Drive:: speed: " << speed);

	if(speed < 0.2)
		speed = 0.2;
	if(speed > 1)
		speed = 1;

	ROS_INFO_STREAM("Drive:: distance: " << distance);

	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.linear.y = 0.0;
	base_cmd.angular.z = speed;
	if (clockwise)
		base_cmd.angular.z = -base_cmd.angular.z;

	velocityPublisher.publish(base_cmd);

	if(abs(absoluteDistanceCurrent) < 10) {
		if(clockwise) {
			if(currentDegrees >= degrees) {
				isDone = true;
				rotateToStartDegrees = -1;
			}
		} else {
			if(currentDegrees <= degrees) {
				isDone = true;
				rotateToStartDegrees = -1;
			}
		}

		if(!isDone && abs(absoluteDistanceCurrent) < 1) {
			isDone = true;
			rotateToStartDegrees = -1;
		}
	}

	if(isDone) {
		stop();
	}

    ROS_INFO_STREAM("Drive:: isDone " << isDone);

    return isDone;
}

bool Drive::rotateBy(int degrees, bool clockwise) {
    double currentDegrees = (static_cast<Location *>(ModuleLoader::get("location")))->getDegrees();
    double positionToRotateTo = 0;

    if(rotateByStartDegrees == -1)
        rotateByStartDegrees = currentDegrees;

    if(clockwise){
        positionToRotateTo = rotateByStartDegrees + degrees;
    } else {
        positionToRotateTo = rotateByStartDegrees - degrees;
    }

    if(positionToRotateTo > 360)
        positionToRotateTo = (int)positionToRotateTo % 360;
    if(positionToRotateTo < 0)
        positionToRotateTo +=360;

    ROS_INFO_STREAM("Drive:: currentDegrees: " << currentDegrees << " positionToRotateTo: " << positionToRotateTo << " degrees: " << degrees << " clockwise: " << clockwise << " startRelativeDegrees: " << rotateByStartDegrees);

    if(rotateTo(positionToRotateTo)){
        rotateByStartDegrees = -1;
        return true;
    }
    return false;
}