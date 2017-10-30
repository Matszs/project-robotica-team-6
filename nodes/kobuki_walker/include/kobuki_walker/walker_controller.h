#ifndef WALKER_CONTROLLER_H_
#define WALKER_CONTROLLER_H_

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
            /// Private ROS handle
            ros::NodeHandle nh_priv;
            /// Node(let) name
            std::string name;
            /// Subscribers
            ros::Subscriber enable_controller_subscriber, disable_controller_subscriber;
            /// Subscribers
            ros::Subscriber bumper_event_subscriber, cliff_event_subscriber, wheel_drop_event_subscriber;
            /// Publishers
            ros::Publisher cmd_vel_publisher, led1_publisher, led2_publisher;
            /// Flag for changing direction
            bool change_direction;
            /// Flag for stopping
            bool stop;
            /// Flag for left bumper's state
            bool bumper_left_pressed;
            /// Flag for center bumper's state
            bool bumper_center_pressed;
            /// Flag for right bumper's state
            bool bumper_right_pressed;
            /// Flag for left cliff sensor's state
            bool cliff_left_detected;
            /// Flag for center cliff sensor's state
            bool cliff_center_detected;
            /// Flag for right cliff sensor's state
            bool cliff_right_detected;
            /// Flag for left wheel drop sensor's state
            bool wheel_drop_left_detected;
            /// Flag for right wheel drop sensor's state
            bool wheel_drop_right_detected;
            /// Flag for bumper LED's state
            bool led_bumper_on;
            /// Flag for cliff sensor LED's state
            bool led_cliff_on;
            /// Flag for wheel drop sensor LED's state
            bool led_wheel_drop_on;
            /// Linear velocity for moving straight
            double vel_lin;
            /// Angular velocity for rotating
            double vel_ang;
            /// Randomly chosen turning duration
            ros::Duration turning_duration;
            /// Randomly chosen turning direction
            int turning_direction;
            /// Start time of turning
            ros::Time turning_start;
            /// Flag for turning state
            bool turning;

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

            ~WalkerController(){};

            bool init();
            void spin();



    };

}

#endif