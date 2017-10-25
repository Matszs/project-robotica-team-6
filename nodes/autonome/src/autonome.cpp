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

    // Start publishers/subcribers
    ROS_INFO("Autonome:: initalization started.");

    bumper_event_subscriber = nodeHandler.subscribe("/mobile_base/events/bumper", 10, &Autonome::bumperEvent, this);
    velocity_publisher = nodeHandler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    grid_publisher = nodeHandler.advertise<kobuki_mapper::GridPoint>("/grid", 10);
    odom_subscriber = nodeHandler.subscribe("/odom", 100, &Autonome::odomCallback, this);

    ROS_INFO("Autonome:: initalization done.");

}

void Autonome::odomCallback(const nav_msgs::OdometryConstPtr& msg) {

    float positionX = msg->pose.pose.position.x;
    float positionY = msg->pose.pose.position.y;
    float rotation = msg->pose.pose.orientation.z;

    positionXGrid = (int)(positionX * 10);
    positionYGrid = (int)(positionY * 10);

    currentRotation = rotation;

}

void Autonome::bumperEvent(const kobuki_msgs::BumperEventConstPtr msg) {

    //ROS_INFO("Autonome:: bumperEvent trigger.");
    //ROS_INFO("Autonome:: state: %d", msg->state);

    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
        switch (msg->bumper) {
            case kobuki_msgs::BumperEvent::LEFT:
                if(!left_bumper_pressed) {
                    ROS_INFO("Autonome:: LEFT!");
                    left_bumper_pressed = true;
                    change_direction = true;
                    last_pressed_bumper = B_LEFT;
                }
            break;

            case kobuki_msgs::BumperEvent::CENTER:
                if(!center_bumper_pressed) {
                    ROS_INFO("Autonome:: CENTER!");
                    center_bumper_pressed = true;
                    change_direction = true;
                    //last_pressed_bumper = center;
                }
            break;

            case kobuki_msgs::BumperEvent::RIGHT:
                if(!right_bumper_pressed) {
                    ROS_INFO("Autonome:: RIGHT!");
                    right_bumper_pressed = true;
                    change_direction = true;
                    last_pressed_bumper = B_RIGHT;
                }
            break;
        }

    } else {
        switch (msg->bumper) {
            case kobuki_msgs::BumperEvent::LEFT:
                left_bumper_pressed = false;
            break;

            case kobuki_msgs::BumperEvent::CENTER:
                center_bumper_pressed = false;
            break;

            case kobuki_msgs::BumperEvent::RIGHT:
                right_bumper_pressed = false;
            break;

        }
    }
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
    geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

    if(stop) { // first check if we have to stop driving
        velocity_publisher.publish(cmd_vel_msg_ptr);
        return;
    }

    if(change_direction) {

        // add current location as a wall to the grid.
        kobuki_mapper::GridPoint gridPoint;
        gridPoint.x = positionXGrid;
        gridPoint.y = positionYGrid;
        gridPoint.type = 0;
        grid_publisher.publish(gridPoint);

        // drive a bit backwards
        cmd_vel_msg_ptr->linear.x = -SPEED;
        velocity_publisher.publish(cmd_vel_msg_ptr);

        // Get out of this if-statement after
        change_direction = false;

        // Find out which direction to rotate to.
        if(last_pressed_bumper == B_LEFT) {
            turning_direction = -1;
        } else if(last_pressed_bumper == B_RIGHT) {
            turning_direction = 1;
        }

        // set rotation
        rotateTo = currentRotation + (turning_direction * 0.5);
        //rotateTo = -currentRotation; // test inverting
        ROS_INFO_STREAM("currentRotation: " << currentRotation);
        ROS_INFO_STREAM("rotateTo: " << rotateTo);

        if(rotateTo > 1)
            rotateTo = -(2 - rotateTo);
        if(rotateTo < -1)
            rotateTo = (2 + rotateTo);

        ROS_INFO_STREAM("rotateTo: " << rotateTo);
        ROS_INFO_STREAM("direction: " << turning_direction);

        // Set the variable so the robot knows it has to rotate.
        is_turning = true;
    }

    if(is_turning) {
        // check if we are in range of the 'destination' rotation.
        if(currentRotation > (rotateTo - 0.1) && currentRotation < (rotateTo + 0.1)) {
            is_turning = false;
            ROS_INFO_STREAM("Rotation done.");
        } else {
            cmd_vel_msg_ptr->angular.z = turning_direction * ANGLE;
            velocity_publisher.publish(cmd_vel_msg_ptr);

            ROS_INFO_STREAM("rotate " << turning_direction * ANGLE);
            ROS_INFO_STREAM("currentRotation #2: " << currentRotation);
        }

    } else { // drive forward
        cmd_vel_msg_ptr->linear.x = SPEED;
        velocity_publisher.publish(cmd_vel_msg_ptr);
    }
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

