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

    // Start publishers/subcribers
    ROS_INFO("Autonome:: initalization started.");

    bumper_event_subscriber = nodeHandler.subscribe("/mobile_base/events/bumper", 10, &Autonome::bumperEvent, this);
    tracked_position_event_subscriber = nodeHandler.subscribe("/vision/tracked_position", 10, &Autonome::trackedPositionEvent, this);
    velocity_publisher = nodeHandler.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);

    ROS_INFO("Autonome:: initalization done.");

}

void Autonome::cliffEvent(const kobuki_msgs::CliffEventConstPtr msg) {
    ROS_INFO("Autonome:: cliffEvent trigger.");
    ROS_INFO("Autonome:: state: %d", msg->state);
}

void Autonome::trackedPositionEvent(const vision::TrackedPositionConstPtr& msg) {

    ROS_INFO("Autonome:: trackedPosition trigger.");
    ROS_INFO("x: [%f], y: [%f], z: [%f], ", -msg->x * 2, -msg->y, -msg->z);

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
        ROS_INFO("Autonome:: NO PRESSURE!");
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
        change_direction = false;

        turning_duration = ros::Duration(((double)std::rand() / (double)RAND_MAX) * (M_PI / ANGLE));

        /*if (((double)std::rand() / (double)RAND_MAX) >= 0.5) {
            turning_direction = 1;
        } else {
            turning_direction = -1;
        }*/

        ROS_INFO_STREAM("Bumper: " << (last_pressed_bumper == B_LEFT ? "left" : "not left"));
        ROS_INFO_STREAM("Bumper: " << (last_pressed_bumper == B_RIGHT ? "right" : "not right"));

        if(last_pressed_bumper == B_LEFT) {
            turning_direction = 1;
        } else if(last_pressed_bumper == B_RIGHT) {
            turning_direction = -1;
        }

        turning_start = ros::Time::now();

        is_turning = true;
        ROS_INFO_STREAM("Will rotate " << turning_direction * turning_duration.toSec() * ANGLE / M_PI * 180 << " degrees.");
    }

    if(is_turning) {
        if ((ros::Time::now() - turning_start) < turning_duration) {
            cmd_vel_msg_ptr->angular.z = turning_direction * ANGLE;
            velocity_publisher.publish(cmd_vel_msg_ptr);

            ROS_INFO_STREAM("rotate " << turning_direction * ANGLE);
        } else {
            is_turning = false;
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

