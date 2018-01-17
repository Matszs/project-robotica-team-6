#include <cstdio>

#include "ros/ros.h"

#include "sensor_msgs/JointState.h"

#include "robot_arm/SetArmPosition.h"
#include "robot_arm/GetArmPosition.h"
#include "robot_arm/SetHandState.h"
#include "robot_arm/GetHandState.h"

#include "arm.h"
#include "math_helpers.h"

#define MAX_ITERATIONS 100

using namespace std;
using namespace ros;
using namespace robot_arm;

Arm::Arm() {
  NodeHandle n;
  this->armPublisher = n.advertise<sensor_msgs::JointState>(
      "/phantomx_reactor_controller/joint_command", 1000);

  this->root = NULL;

  this->clawState = -0.6;
  this->baseRotation = 0;
  this->addArmSegment(new ArmSegment(150, RAD(-90), RAD(90), ARM_SHOULDER));
  this->addArmSegment(new ArmSegment(150, RAD(0), RAD(175), ARM_ELBOW));
  this->addArmSegment(new ArmSegment(140, RAD(-90), RAD(90), ARM_WRIST));

  ROS_WARN("ARM CREATED");
  this->publishArmState();
}

ArmSegment *Arm::getLastSegment() {
  ArmSegment *last = this->root;

  while (last->getChild() != NULL)
    last = last->getChild();

  return last;
}

void Arm::addArmSegment(ArmSegment *arm_segment) {
  if (this->root == NULL) {
    this->root = arm_segment;
    return;
  }

  arm_segment->setParent(this->getLastSegment());
}

void Arm::setTarget(Position *target_position_abs) {
  Position target_position_rel;

  this->target = *target_position_abs;
  this->baseRotation = atan(target_position_abs->x / target_position_abs->z);

  target_position_rel.x = 0;
  target_position_rel.y = target_position_abs->y;
  target_position_rel.z = hypot(target_position_abs->x, target_position_abs->z);

  this->calculateArmPositions(&target_position_rel);

  double distance = hypot(target_position_rel.y, target_position_rel.z);

  this->publishArmState();
}

void Arm::calculatePosition(Position *position_abs) {
  Position position_rel;

  ArmSegment *last = this->getLastSegment();
  last->calculateTopPosition(&position_rel);

  position_abs->x = position_rel.z * sin(this->baseRotation);
  position_abs->y = position_rel.y;
  position_abs->z = position_rel.z * cos(this->baseRotation);
}

void Arm::calculateArmPositions(Position *target_position_rel) {
  ArmSegment *segment;
  Position current_position_rel;

  this->resetArmSegments();

  int iteration = 0;
  while (iteration <= MAX_ITERATIONS) {
    segment = this->getLastSegment();
    segment->calculateTopPosition(&current_position_rel);

    double distance = hypot((target_position_rel->y - current_position_rel.y),
                            (target_position_rel->z - current_position_rel.z));

    if (distance < 1)
      break;

    while (segment != NULL) {
      segment->setTarget(target_position_rel);
      segment = segment->getParent();
    }

    if (MAX_ITERATIONS != 0)
      iteration++;
  }
}

void Arm::resetArmSegments() {
  ArmSegment *segment = this->root;
  while (segment != NULL) {
    segment->setAngle(0);
    segment = segment->getChild();
  }
}

void Arm::setClawState(bool open) {
  if (open) {
    printf("Now opening\n");
    this->clawState = 0;
  } else {
    printf("Now closing\n");
    this->clawState = -0.6;
  }

  this->publishArmState();
}

bool Arm::getClawState() { return this->clawState == 0 ? true : false; }

void Arm::publishArmState() {
  sensor_msgs::JointState states;

  double angle;
  const char *name;
  ArmSegment *segment = this->root;
  while (segment != NULL) {
    name = segment->getSegmentName();
    angle = segment->getAngle();

    if (strcmp(name, ARM_SHOULDER) == 0) {
      angle -= RAD(15.46);
    } else if (strcmp(name, ARM_ELBOW) == 0) {
      angle -= RAD((90 - 15.46));
      angle *= -1;
    } else if (strcmp(name, ARM_WRIST) == 0) {
      angle *= -1;
    }

    if (!isnan(angle)) {
      states.name.push_back(name);
      states.position.push_back(angle);
    }

    segment = segment->getChild();
  }

  states.name.push_back(ARM_BASE);
  states.position.push_back(-(this->baseRotation - RAD(135)));

  states.name.push_back(ARM_WRIST_ROT);
  states.position.push_back(0);

  states.name.push_back(ARM_HAND);
  states.position.push_back(this->clawState);

  this->armPublisher.publish(states);
}

namespace arm {
Arm *arm;

bool set_arm_position(SetArmPosition::Request &req,
                      SetArmPosition::Response &res) {
  ROS_WARN("SET ARM POSITION");

  arm->setTarget(&req.position);
  return true;
}

bool get_arm_position(GetArmPosition::Request &req,
                      GetArmPosition::Response &res) {

  arm->calculatePosition(&res.position);

  return true;
}

bool set_hand_state(SetHandState::Request &req, SetHandState::Response &res) {

  arm->setClawState(req.open);
  return true;
}

bool get_hand_state(GetHandState::Request &req, GetHandState::Response &res) {
  res.open = arm->getClawState();
  return true;
}

int start(int argc, char **argv) {
  init(argc, argv, "robot_arm");
  NodeHandle n;

  arm = new Arm();

  ServiceServer set_arm_position_server =
      n.advertiseService("set_arm_position", set_arm_position);
  ServiceServer get_arm_position_server =
      n.advertiseService("get_arm_position", get_arm_position);

  ServiceServer set_hand_state_server =
      n.advertiseService("set_hand_state", set_hand_state);
  ServiceServer get_hand_state_server =
      n.advertiseService("get_hand_state", get_hand_state);

  arm->publishArmState();

  spin();

  return 0;
}
}

int main(int argc, char **argv) { return arm::start(argc, argv); }

/* vim: set ts=8 sw=2 tw=0 et :*/
