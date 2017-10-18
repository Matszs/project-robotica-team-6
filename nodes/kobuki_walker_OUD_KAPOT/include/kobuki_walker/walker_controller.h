#ifndef WALKER_CONTROLLER_H
#define WALKER_CONTROLLER_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <yocs_controllers/default_controller.hpp>

namespace kobuki {

    class WalkerController : public yocs::Controller {

        private:
            ros::NodeHandle nh_priv;
            std::string name;
            ros::Subscriber enable_controller_subscriber;
            ros::Subscriber disable_controller_subscriber;
            ros::Subscriber bumper_event_subscriber;
            ros::Subscriber cliff_event_subscriber;
            ros::Subscriber wheel_drop_event_subscriber;

            ros::Publisher cmd_vel_publisher;
            ros::Publisher led1_publisher;
            ros::Publisher led2_publisher;

            bool change_direction;
            bool stop;
            bool bumper_left_pressed;
            bool bumper_center_pressed;
            bool bumper_right_pressed;
            bool cliff_left_detected;
            bool cliff_center_detected;
            bool cliff_right_detected;
            bool wheel_drop_left_detected;
            bool wheel_drop_right_detected;
            bool led_bumper_on;
            bool led_cliff_on;
            bool led_wheel_drop_on;
            double vel_lin;
            double vel_ang;
            ros::Duration turning_duration;
            int turning_direction;
            ros::Time turning_start;
            bool turning;

        public:
            WalkerController(ros::NodeHandle& nh_priv, std::string& name) : Controller(),
                                                                                       nh_priv(nh_priv),
                                                                                       name(name),
                                                                                       change_direction(false),
                                                                                       stop(false),
                                                                                       bumper_left_pressed(false),
                                                                                       bumper_center_pressed(false),
                                                                                       bumper_right_pressed(false),
                                                                                       cliff_left_detected(false),
                                                                                       cliff_center_detected(false),
                                                                                       cliff_right_detected(false),
                                                                                       led_bumper_on(false),
                                                                                       led_cliff_on(false),
                                                                                       led_wheel_drop_on(false),
                                                                                       turning(false),
                                                                                       turning_direction(1)
                                                                                       {};

            ~WalkerController();
            bool init();
            void spin();

            /**
            * @brief ROS logging output for enabling the controller
            * @param msg incoming topic message
            */
            void enableCB(const std_msgs::EmptyConstPtr msg);

            /**
            * @brief ROS logging output for disabling the controller
            * @param msg incoming topic message
            */
            void disableCB(const std_msgs::EmptyConstPtr msg);

            /**
            * @brief Trigger direction change and LED blink, when a bumper is pressed
            * @param msg bumper event
            */
            void bumperEventCB(const kobuki_msgs::BumperEventConstPtr msg);

            /**
            * @brief Trigger direction change and LED blink, when a cliff is detected
            * @param msg cliff event
            */
            void cliffEventCB(const kobuki_msgs::CliffEventConstPtr msg);

            /**
            * @brief Trigger stopping and LED blink, when a wheel drop is detected
            * @param msg wheel drop event
            */
            void wheelDropEventCB(const kobuki_msgs::WheelDropEventConstPtr msg);

    };


}

#endif
