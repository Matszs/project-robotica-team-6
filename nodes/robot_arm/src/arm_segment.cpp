#include "arm_segment.h"

#define BETWEEN(x, mn, mx) (std::min(std::max(x, mn), mx))

double ArmSegment::calculateAngleBetween(Position *vector1, Position *vector2) {
  double length_vector1 = hypot(vector1->y, vector1->z);
  double length_vector2 = hypot(vector2->y, vector2->z);

  double dot_product = ((vector1->y * vector2->y) + (vector1->z * vector2->z));

  double angle = acos(dot_product / (length_vector1 * length_vector2));

  double cross_product =
      ((vector1->y * vector2->z) - (vector1->z * vector2->y));

  int sign = (cross_product > 0) - (cross_product < 0);

  return sign * angle;
}

ArmSegment::ArmSegment(double length, double min_angle, double max_angle,
                       const char *segment_name) {
  this->segmentName = segment_name;

  this->parent = NULL;
  this->child = NULL;

  this->length = length;
  this->angle = 0;

  this->minAngle = min_angle;
  this->maxAngle = max_angle;
}

const char *ArmSegment::getSegmentName() { return this->segmentName; }

void ArmSegment::setParent(ArmSegment *parent) {
  this->parent = parent;
  this->parent->child = this;
}

ArmSegment *ArmSegment::getParent() { return this->parent; }

ArmSegment *ArmSegment::getChild() { return this->child; }

void ArmSegment::setAngle(double angle) {
  this->angle = BETWEEN(angle, this->minAngle, this->maxAngle);
}

double ArmSegment::getAngle() { return this->angle; }

double ArmSegment::getHeading() {
  double base_heading = this->parent != NULL ? this->parent->getHeading() : 0;
  return base_heading + this->angle;
}

void ArmSegment::calculateVector(Position *position, Position *vector) {
  Position bottom;
  this->calculateBottomPosition(&bottom);

  vector->x = position->x - bottom.x;
  vector->y = position->y - bottom.y;
  vector->z = position->z - bottom.z;
}

void ArmSegment::calculateTopPosition(Position *top) {
  Position bottom;
  this->calculateBottomPosition(&bottom);

  top->x = 0;
  top->y = bottom.y + this->length * cos(this->getHeading());
  top->z = bottom.z + this->length * sin(this->getHeading());
}

void ArmSegment::calculateBottomPosition(Position *bottom) {
  if (this->parent != NULL) {
    Position parent_top;
    this->parent->calculateTopPosition(&parent_top);

    bottom->x = parent_top.x;
    bottom->y = parent_top.y;
    bottom->z = parent_top.z;
  } else {
    bottom->x = 0;
    bottom->y = 0;
    bottom->z = 0;
  }
}

void ArmSegment::setTarget(Position *position) {
  ArmSegment *last = this;
  while (last->getChild() != NULL)
    last = last->getChild();

  Position current;
  last->calculateTopPosition(&current);

  Position origin_vector, target_vector;
  this->calculateVector(&current, &origin_vector);
  this->calculateVector(position, &target_vector);

  double new_angle = this->angle + ArmSegment::calculateAngleBetween(
                                       &origin_vector, &target_vector);

  if (isnan(new_angle))
    this->setAngle(0);
  else if (new_angle > M_PI)
    this->setAngle(new_angle - (2 * M_PI));
  else if (new_angle < -M_PI)
    this->setAngle((2 * M_PI) - new_angle);
  else
    this->setAngle(new_angle);
}

/* vim: set ts=8 sw=2 tw=0 et :*/
