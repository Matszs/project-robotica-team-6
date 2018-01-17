#ifndef ARM_H
#define ARM_H

#include "robot_arm/Position.h"

#include "arm_segment.h"

#define ARM_BASE "j1"
#define ARM_SHOULDER "j2"
#define ARM_ELBOW "j3"
#define ARM_WRIST "j4"
#define ARM_WRIST_ROT "j5"
#define ARM_HAND "gripper"

using namespace ros;
using namespace robot_arm;

class Arm {
private:
  Publisher armPublisher;

  double clawState;
  double wristRotation;
  double baseRotation;

  ArmSegment *root;
  Position target;

public:
  Arm();

  ArmSegment *getLastSegment();
  void addArmSegment(ArmSegment *arm_segment);

  double getDistanceFromTarget();

  void setTarget(Position *position);
  void calculatePosition(Position *position_abs);
  void calculateArmPositions(Position *target_position_rel);

  void resetArmSegments();
  void setClawState(bool open);
  bool getClawState();

  void publishArmState();
};

#endif
/* vim: set ts=8 sw=2 tw=0 et :*/
