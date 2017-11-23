#include "autonome.h"

void Autonome::init() {
    // Set default values
    left_bumper_pressed = false;
    right_bumper_pressed = false;
    center_bumper_pressed = false;

    change_direction = false;
    is_turning = false;
    turning_direction = 1;

    last_pressed_bumper = B_NONE;
    rotateTo = 0;
    currentRotation = 0;
    isReached = false;
    isPositionSet = false;
    degrees = 0;
    cameraDistance = -1.0;
    changeDegrees = false;
    camera_distance_set = 0;

    ultra_front = 0;
    ultra_right = 0;

    status = 0;
    degrees_start = -1;
    highestValue = 0;
    highestIndex = 0;

    rotationCounter = 0;

    memset(degreesDistance, 0, sizeof(degreesDistance));

    degreesToRideTo = 0;

    degreesFrom = -1;

    // Start publishers/subcribers
    ROS_INFO("Autonome:: initalization started.");

    bumper_event_subscriber = nodeHandler.subscribe("/mobile_base/events/bumper", 10, &Autonome::bumperEvent, this);
    velocity_publisher = nodeHandler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    grid_publisher = nodeHandler.advertise<kobuki_mapper::GridPoint>("/grid", 10);
    odom_subscriber = nodeHandler.subscribe("/odom", 100, &Autonome::odomCallback, this);
    cameraPoints = nodeHandler.subscribe("/camera_points", 100, &Autonome::cameraPointsCallback, this);
    ultraPoints = nodeHandler.subscribe("/ultrasone_sensors", 100, &Autonome::ultrasoneSensorsCallback, this);

    ROS_INFO("Autonome:: initalization done.");

}

void Autonome::makeThreeSixty(){

    geometry_msgs::Twist vel;

        vel.angular.z = 1;
        velocity_publisher.publish(vel);
        ROS_INFO_STREAM("degrees = " << degrees);

}


void Autonome::cameraPointsCallback(const kobuki_depth::CameraPointConstPtr& msg) {
    cameraDistance = msg->z;


    if(status == 2) {
        degreesDistance[(int)degrees] = cameraDistance;
    }
}


void Autonome::ultrasoneSensorsCallback(const kobuki_ultrasone::UltrasoneSensorsConstPtr& msg) {
    ultra_right = msg->sensor2_distance;
    ultra_front = msg->sensor1_distance;

    /*if(!is_turning) {
        if(ultra_right < 9) {
            change_direction = true;
            turning_direction = 1*//* * (ultra_right / 10)*//*;
        } else if(ultra_right > 12) {
            change_direction = true;
            turning_direction = -1*//* * (ultra_right / 10)*//*;
        } else {
            change_direction = false;
        }
    }*/
}


void Autonome::odomCallback(const nav_msgs::OdometryConstPtr& msg) {

    float positionX = msg->pose.pose.position.x;
    float positionY = msg->pose.pose.position.y;
    float rotation = msg->pose.pose.orientation.z;

    positionXGrid = (int)(-positionY * 10);
    positionYGrid = (int)(positionX * 10);

    currentRotation = rotation;

    // get 2 decimals from the current rotation
    degrees = (float)((int)(rotation * 100)) / 100;
    // format rotation into degrees
    degrees = (degrees > 0 ? 360 - (360 * degrees / 2) : 360 * std::abs(degrees) / 2);

}

void Autonome::bumperEvent(const kobuki_msgs::BumperEventConstPtr msg) {

}

void Autonome::run() {
    ros::Rate spin_rate(10);

    while (ros::ok() && !emergency_stop) {
        ros::spinOnce();
        this->spin();
        spin_rate.sleep();
    }

}

void Autonome::spin() {

    ROS_INFO_STREAM("Status: " << status);

    if(status == 0 && camera_distance_set == 0) {
        if(cameraDistance > 0) {
            camera_distance_set = cameraDistance;

            ROS_INFO_STREAM("cameraDistance: " << cameraDistance);
            status = 1;
        }
    }

    if(status == 1 && camera_distance_set > 0) {

        if(cameraDistance > (camera_distance_set / 2)) {

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->linear.x = 0.2;
            velocity_publisher.publish(cmd_vel_msg_ptr);

        } else {
            status = 2;
        }
    }

    if(status == 2) {

        if(rotationCounter < 95) {

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->angular.z = 0.3;
            velocity_publisher.publish(cmd_vel_msg_ptr);

            ROS_INFO_STREAM("val " << degreesDistance[(int)degrees]);
            if(degreesDistance[(int)degrees] == 0) {
                degreesDistance[(int)degrees] = cameraDistance;
                rotationCounter++;


                ROS_INFO_STREAM("degrees: " << (int)degrees << " start: " << rotationCounter);
            }

        } else {
            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->angular.z = 0;
            velocity_publisher.publish(cmd_vel_msg_ptr);

            status = 3;
        }

        if(degrees_start == -1)
            degrees_start = degrees + 10;
    }

    if(status == 3) {

        int i;
        for(i = 0; i < 361; i++) {
            float degreesValue = degreesDistance[i];

            if (isnan(degreesValue)) {
                nanNumbers.push_back(i);
            }else if(degreesValue > highestValue) {
                highestValue = degreesValue;
                highestIndex = i;
            }
        }

        ROS_INFO_STREAM("nanNumbers length: " << nanNumbers.size());
        ROS_INFO_STREAM("highest index: " << highestIndex);
        ROS_INFO_STREAM("highest value: " << highestValue);

        if(nanNumbers.size() > 0) {
            degreesToRideTo = degreesDistance[nanNumbers.at((int)(nanNumbers.size() / 2))];
        } else {
            degreesToRideTo = highestIndex;
        }

        status = 4;
    }

    if(status == 4) {

       geometry_msgs::TwistPtr cmd_vel_msg_ptr;
       cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

       cmd_vel_msg_ptr->angular.z = 1;
       velocity_publisher.publish(cmd_vel_msg_ptr);

       ROS_INFO_STREAM("degrees: " << degrees << " rideTo: " << degreesToRideTo);

       if((int)degrees == degreesToRideTo) {
           geometry_msgs::TwistPtr cmd_vel_msg_ptr;
           cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

           cmd_vel_msg_ptr->angular.z = 0;
           velocity_publisher.publish(cmd_vel_msg_ptr);

           status = 5;
       }

    }

    if(status == 5) {
        if(degreesFrom == -1) {
            degreesFrom = degreesToRideTo + 180;
            if(degreesFrom > 360)
                degreesFrom = 360 - degreesFrom;
        }

        if(cameraDistance > 0.6) {

            geometry_msgs::TwistPtr cmd_vel_msg_ptr;
            cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

            cmd_vel_msg_ptr->linear.x = 0.2;
            velocity_publisher.publish(cmd_vel_msg_ptr);

        } else {
            degrees_start = -1;
            memset(degreesDistance, 0, sizeof(degreesDistance));
            highestValue = 0;
            highestIndex = 0;

            status = 2;
        }

    }


















    /*geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

    if (change_direction) {
        change_direction = false;

        turning_duration = ros::Duration((M_PI / 0.2));

        turning_start = ros::Time::now();
        is_turning = true;

        ROS_INFO_STREAM("Will rotate " << turning_direction * turning_duration.toSec() * 0.3 / M_PI * 180 << " degrees.");
    }


    if (is_turning) {
        if ((ros::Time::now() - turning_start) < turning_duration && !(ultra_right >= 9 && ultra_right <= 12)) {
            cmd_vel_msg_ptr->angular.z = turning_direction * 0.2;
            cmd_vel_msg_ptr->linear.x = 0.2;
            velocity_publisher.publish(cmd_vel_msg_ptr);
        } else {
            is_turning = false;
        }
    } else {
        cmd_vel_msg_ptr->linear.x = 0.2;
        velocity_publisher.publish(cmd_vel_msg_ptr);
    }
*/





    /*float distance = (cameraDistance > 0.6 ? cameraDistance : (ultra_front / 100));

    ROS_INFO_STREAM("distance " << distance << " cameraDistance: " << cameraDistance << " type: " << (cameraDistance > 0.6 ? "camera" : "ultrasone"));

    if(!is_turning) {
        if(distance > 0.1) {
            vel.linear.x = 0.1;
        } else {
            vel.linear.x = 0;

            //is_turning = true;
            //turning_direction = 1;
        }
    }

    velocity_publisher.publish(vel);*/













    /*

    geometry_msgs::Twist vel;
    float distance = cameraDistance;

    if(isPositionSet == false) positionToReach = distance / 2; isPositionSet = true;

    if(isReached){
        vel.linear.x = 0;
        velocity_publisher.publish(vel);

        Autonome::makeThreeSixty();
        isReached = false;
    }
    else {
        if(distance > (positionToReach + 0.03) || distance < (positionToReach - 0.03)){
                    vel.linear.x = distance / 5;
                    velocity_publisher.publish(vel);

        }
        else{
            isReached = true;
            changeDegrees = true;
        }
    }

ROS_INFO("distance = %f, positionToReach = %f", distance, positionToReach);*/


}






// main program

int main(int argc, char **argv) {
	ROS_INFO("Starting the node");

	ros::init(argc, argv, "autonome");
	ROS_INFO("ros::init done");

	Autonome driving;

	driving.init();
	driving.run();

	ROS_INFO("Exiting the node");
	return 0;
}

