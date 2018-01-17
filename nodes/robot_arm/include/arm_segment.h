#ifndef ARM_SEGMENT_H
#define ARM_SEGMENT_H

#include <math.h>

#include "robot_arm/Position.h"

#include "math_helpers.h"

using namespace robot_arm;

class ArmSegment {
private:
  const char *segmentName;

  ArmSegment *parent;
  ArmSegment *child;

  double length;
  double angle;

  double maxAngle;
  double minAngle;

public:
  static double calculateAngleBetween(Position *vector1, Position *vector2);

  ArmSegment(double length, double min_angle, double max_angle,
             const char *segment_name);

  const char *getSegmentName();

  void setParent(ArmSegment *parent);
  ArmSegment *getParent();
  ArmSegment *getChild();

  void setAngle(double angle);
  double getAngle();
  double getHeading();

  void calculateVector(Position *position, Position *vector);
  void calculateTopPosition(Position *top);
  void calculateBottomPosition(Position *bottom);

  void setTarget(Position *position);
};

#endif

/* vim: set ts=8 sw=2 tw=0 et :*/
