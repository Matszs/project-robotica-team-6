#include "arm.h"

Arm::Arm(ros::NodeHandle * nodeHandle) : Module(nodeHandle) {
    armPublisher = nodeHandle->advertise<sensor_msgs::JointState>("/phantomx_reactor_controller/joint_command", 1000);

	ROS_INFO_STREAM("Arm:: Module initialized.");

	//setStartPosition();
}

void Arm::read() {

    //setGripperState(100);

    //setStartPosition();

	ROS_INFO_STREAM("Arm:: read.");
}


void Arm::setStartPosition() {

    sensor_msgs::JointState jointState;
    jointState.name.push_back(ARM_BASE);
    jointState.position.push_back(1.57); // 0 => 90 graden, 1.57 => 0 graden
    armPublisher.publish(jointState);

    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState5;

    jointState5.name.push_back(ARM_ELBOW);
    jointState5.position.push_back(0.7); // 0 => 'recht naar beneden', 1.57 'recht vooruit'

    jointState5.name.push_back(ARM_WRIST);
    jointState5.position.push_back(0.7); // 1.57 => 'recht naar voren', 0 => 'naar beneden'

    armPublisher.publish(jointState5);


    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState1;
    jointState1.name.push_back(ARM_SHOULDER);
    jointState1.position.push_back(1.57); // 1.57 => 'recht naar beneden'
    armPublisher.publish(jointState1);

    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState2;
    jointState2.name.push_back(ARM_ELBOW);
    jointState2.position.push_back(0.01); // 0 => 'recht naar beneden', 1.57 'recht vooruit'
    armPublisher.publish(jointState2);

    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState3;
    jointState3.name.push_back(ARM_WRIST);
    jointState3.position.push_back(1.57); // 1.57 => 'recht naar voren', 0 => 'naar beneden'
    armPublisher.publish(jointState3);

    /*ros::Duration(1.0).sleep();

    sensor_msgs::JointState jointState4;
    jointState4.name.push_back(ARM_HAND);
    jointState4.position.push_back(0.01);
    armPublisher.publish(jointState4);

    ros::Duration(1.0).sleep();*/

}



void Arm::resetPosition() {


    sensor_msgs::JointState jointState1;
    jointState1.name.push_back(ARM_SHOULDER);
    jointState1.position.push_back(-1.57); // 1.57 => 'recht naar beneden'
    armPublisher.publish(jointState1);

    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState5;

    jointState5.name.push_back(ARM_ELBOW);
    jointState5.position.push_back(-1.57); // 0 => 'recht naar beneden', 1.57 'recht vooruit'

    jointState5.name.push_back(ARM_WRIST);
    jointState5.position.push_back(1.57); // 1.57 => 'recht naar voren', 0 => 'naar beneden'

    armPublisher.publish(jointState5);



    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState2;
    jointState2.name.push_back(ARM_ELBOW);
    jointState2.position.push_back(0.01); // 0 => 'recht naar beneden', 1.57 'recht vooruit'
    armPublisher.publish(jointState2);

    ros::Duration(2.0).sleep();

    sensor_msgs::JointState jointState3;
    jointState3.name.push_back(ARM_WRIST);
    jointState3.position.push_back(1.57); // 1.57 => 'recht naar voren', 0 => 'naar beneden'
    armPublisher.publish(jointState3);


    ros::Duration(2.0).sleep();


    sensor_msgs::JointState jointState;
    jointState.name.push_back(ARM_BASE);
    jointState.position.push_back(0.7); // 0 => 90 graden, 1.57 => 0 graden
    armPublisher.publish(jointState);

    /*ros::Duration(1.0).sleep();

    sensor_msgs::JointState jointState4;
    jointState4.name.push_back(ARM_HAND);
    jointState4.position.push_back(0.01);
    armPublisher.publish(jointState4);

    ros::Duration(1.0).sleep();*/

}


/*
    0 = open
    100 = dicht
*/
void Arm::setGripperState(int level) {

    float gripState = -1.57 + (1.57 * ((float)level / 100)) + 0.01; // + 0.01 fix because 0.0 is not allowed

    sensor_msgs::JointState jointState;
    jointState.name.push_back(ARM_HAND);
    jointState.position.push_back(gripState);
    jointState.velocity.push_back(0.0);
    jointState.effort.push_back(0.0);
    armPublisher.publish(jointState);

}