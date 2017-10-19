#include "walker_controller.h"

bool WalkerController::init() {

    enable_controller_subscriber = nh_priv.subscribe("enable", 10, &WalkerController::enableCB, this);
    disable_controller_subscriber = nh_priv.subscribe("disable", 10, &WalkerController::disableCB, this);
    bumper_event_subscriber = nh_priv.subscribe("events/bumper", 10, &WalkerController::bumperEventCB, this);
    cliff_event_subscriber = nh_priv.subscribe("events/cliff", 10, &WalkerController::cliffEventCB, this);
    wheel_drop_event_subscriber = nh_priv.subscribe("events/wheel_drop", 10,
                                                  &WalkerController::wheelDropEventCB, this);
    cmd_vel_publisher_ = nh_priv.advertise<geometry_msgs::Twist>("commands/velocity", 10);
    led1_publisher_ = nh_priv.advertise<kobuki_msgs::Led>("commands/led1", 10);
    led2_publisher_ = nh_priv.advertise<kobuki_msgs::Led> ("commands/led2", 10);

    nh_priv.param("linear_velocity", vel_lin_, 0.5);
    nh_priv.param("angular_velocity", vel_ang_, 0.1);
    ROS_INFO_STREAM("Velocity parameters: linear velocity = " << vel_lin << ", angular velocity = " << vel_ang << " [" << name <<"]");
    std::srand(std::time(0));

    this->enable(); // enable controller

    return true;

};

void WalkerController::enableCB(const std_msgs::EmptyConstPtr msg) {
    if (this->enable()) {
        ROS_INFO_STREAM("Controller has been enabled. [" << name << "]");
    } else {
        ROS_INFO_STREAM("Controller was already enabled. [" << name <<"]");
    }
};

void WalkerController::disableCB(const std_msgs::EmptyConstPtr msg) {
    if (this->disable()) {
        ROS_INFO_STREAM("Controller has been disabled. [" << name <<"]");
    } else {
        ROS_INFO_STREAM("Controller was already disabled. [" << name <<"]");
    }
};

void WalkerController::bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg) {
    if (this->getState()) { // check, if the controller is active
        if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
            switch (msg->bumper) {
                case kobuki_msgs::BumperEvent::LEFT:
                    if (!bumper_left_pressed) {
                        bumper_left_pressed = true;
                        change_direction = true;
                    }
                break;
                case kobuki_msgs::BumperEvent::CENTER:
                    if (!bumper_center_pressed) {
                        bumper_center_pressed = true;
                        change_direction = true;
                    }
                break;
                case kobuki_msgs::BumperEvent::RIGHT:
                    if (!bumper_right_pressed) {
                        bumper_right_pressed = true;
                        change_direction = true;
                    }
                break;
            }
        } else { // kobuki_msgs::BumperEvent::RELEASED
            switch (msg->bumper) {
                case kobuki_msgs::BumperEvent::LEFT:
                    bumper_left_pressed = false;
                break;
                case kobuki_msgs::BumperEvent::CENTER:
                    bumper_center_pressed = false;
                break;
                case kobuki_msgs::BumperEvent::RIGHT:
                    bumper_right_pressed = false;
                break;
            }
        }

        if (!led_bumper_on && (bumper_left_pressed || bumper_center_pressed || bumper_right_pressed)) {
            kobuki_msgs::LedPtr led_msg_ptr;
            led_msg_ptr.reset(new kobuki_msgs::Led());
            led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
            led1_publisher.publish(led_msg_ptr);
            led_bumper_on = true;
        } else if (led_bumper_on && (!bumper_left_pressed && !bumper_center_pressed && !bumper_right_pressed)) {
            kobuki_msgs::LedPtr led_msg_ptr;
            led_msg_ptr.reset(new kobuki_msgs::Led());
            led_msg_ptr->value = kobuki_msgs::Led::BLACK;
            led1_publisher.publish(led_msg_ptr);
            led_bumper_on = false;
        }
        if (change_direction) {
            ROS_INFO_STREAM("Bumper pressed. Changing direction. [" << name_ << "]");
        }
    }
};

void WalkerController::cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg) {
    if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {
        switch (msg->sensor) {
            case kobuki_msgs::CliffEvent::LEFT:
                if (!cliff_left_detected) {
                    cliff_left_detected = true;
                    change_direction = true;
                }
            break;
            case kobuki_msgs::CliffEvent::CENTER:
                if (!cliff_center_detected) {
                    cliff_center_detected = true;
                    change_direction = true;
                }
            break;
            case kobuki_msgs::CliffEvent::RIGHT:
                if (!cliff_right_detected) {
                    cliff_right_detected = true;
                    change_direction = true;
                }
            break;
        }
    } else { // kobuki_msgs::BumperEvent::FLOOR
        switch (msg->sensor) {
            case kobuki_msgs::CliffEvent::LEFT:
                cliff_left_detected = false;
            break;
            case kobuki_msgs::CliffEvent::CENTER:
                cliff_center_detected = false;
            break;
            case kobuki_msgs::CliffEvent::RIGHT:
                cliff_right_detected = false;
            break;
        }
    }
    if (!led_cliff_on && (cliff_left_detected || cliff_center_detected || cliff_right_detected)) {
        kobuki_msgs::LedPtr led_msg_ptr;
        led_msg_ptr.reset(new kobuki_msgs::Led());
        led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
        led2_publisher.publish(led_msg_ptr);
        led_cliff_on = true;
    } else if (led_cliff_on && (!cliff_left_detected && !cliff_center_detected && !cliff_right_detected)) {
        kobuki_msgs::LedPtr led_msg_ptr;
        led_msg_ptr.reset(new kobuki_msgs::Led());
        led_msg_ptr->value = kobuki_msgs::Led::BLACK;
        led2_publisher.publish(led_msg_ptr);
        led_cliff_on = false;
    }

    if (change_direction) {
        ROS_INFO_STREAM("Cliff detected. Changing direction. [" << name << "]");
    }
};

void WalkerController::wheelDropEventCB(const kobuki_msgs::WheelDropEventConstPtr msg) {
    if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED) {
        switch (msg->wheel) {
            case kobuki_msgs::WheelDropEvent::LEFT:
                if (!wheel_drop_left_detected) {
                    wheel_drop_left_detected = true;
                }
            break;
            case kobuki_msgs::WheelDropEvent::RIGHT:
                if (!wheel_drop_right_detected) {
                    wheel_drop_right_detected = true;
                }
            break;
        }
    } else { // kobuki_msgs::WheelDropEvent::RAISED
        switch (msg->wheel) {
            case kobuki_msgs::WheelDropEvent::LEFT:
                wheel_drop_left_detected = false;
            break;
            case kobuki_msgs::WheelDropEvent::RIGHT:
                wheel_drop_right_detected = false;
            break;
        }
    }

    if (!led_wheel_drop_on && (wheel_drop_left_detected || wheel_drop_right_detected)) {
        kobuki_msgs::LedPtr led_msg_ptr;
        led_msg_ptr.reset(new kobuki_msgs::Led());
        led_msg_ptr->value = kobuki_msgs::Led::RED;
        led1_publisher.publish(led_msg_ptr);
        led2_publisher.publish(led_msg_ptr);
        stop = true;
        led_wheel_drop_on = true;
    } else if (led_wheel_drop_on && (!wheel_drop_left_detected && !wheel_drop_right_detected)) {
        kobuki_msgs::LedPtr led_msg_ptr;
        led_msg_ptr.reset(new kobuki_msgs::Led());
        led_msg_ptr->value = kobuki_msgs::Led::BLACK;
        led1_publisher.publish(led_msg_ptr);
        led2_publisher.publish(led_msg_ptr);
        stop = false;
        led_wheel_drop_on = false;
    }
    if (change_direction) {
        ROS_INFO_STREAM("Wheel(s) dropped. Stopping. [" << name << "]");
    }
};

void WalkerController::spin() {
    if (this->getState()) { // check, if the controller is active
        // Velocity commands
        geometry_msgs::TwistPtr cmd_vel_msg_ptr;
        cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

        if (stop) {
            cmd_vel_publisher.publish(cmd_vel_msg_ptr); // will be all zero when initialised
            return;
        }

        if (change_direction) {
            change_direction = false;
            // calculate a random turning angle (-180 ... +180) based on the set angular velocity
            // time for turning 180 degrees in seconds = M_PI / angular velocity
            turning_duration = ros::Duration(((double)std::rand() / (double)RAND_MAX) * (M_PI / vel_ang));
            // randomly chosen turning direction
            if (((double)std::rand() / (double)RAND_MAX) >= 0.5) {
                turning_direction = 1;
            } else {
                turning_direction = -1;
            }

            turning_start = ros::Time::now();
            turning = true;

            ROS_INFO_STREAM("Will rotate " << turning_direction * turning_duration.toSec() * vel_ang_ / M_PI * 180 << " degrees. [" << name << "]");
        }

        if (turning) {
            if ((ros::Time::now() - turning_start) < turning_duration) {
                cmd_vel_msg_ptr->angular.z = turning_direction * vel_ang;
                cmd_vel_publisher_.publish(cmd_vel_msg_ptr);
            } else {
                turning = false;
            }
        } else {
            cmd_vel_msg_ptr->linear.x = vel_lin_;
            cmd_vel_publisher.publish(cmd_vel_msg_ptr);
        }
    }
};