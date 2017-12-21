#include "rotation.h"

Rotation:init(Robot * robot) {
    this->robot = &robot;
}

void Rotation::reset() {
    fill(begin(rotationPossibilities), end(rotationPossibilities), DEFAULT_ROTATION_VALUE); // set all values to 5.
}

void Rotation::update(int index, int length, int math) {
    for( unsigned int i = index; i < (index + length); i++ ) {
        int degreesIndex = i;
        if(i >= (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])))
            degreesIndex -= 360;

        if(math == 0) {
            rotationPossibilities[degreesIndex] = 0;
        } else {
            rotationPossibilities[degreesIndex] += math;
        }

    }
}

void Rotation::decrease(int index, int length, int steps) {
    update(index, length, 1 * steps);
}

void Rotation::increase(int index, int length, int steps) {
    update(index, length, -1 * steps);
}

void Rotation::print() {
    for( unsigned int i = 0; i < (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])); i++ ) {
        ROS_INFO_STREAM("rotationPossibilities " << i << " = " << rotationPossibilities[i]);
    }
}

int Rotation::getDirection() {
    std::vector<int> rotationDegrees;
    int highestValue = 0;

    for( unsigned int i = 0; i < (sizeof(rotationPossibilities) / sizeof(rotationPossibilities[0])); i++ ) {
        int degreesIndex = i + 1;
        int value = rotationPossibilities[i];

        if(value > highestValue) {
            rotationDegrees.clear();
            rotationDegrees.push_back(degreesIndex);
            highestValue = value;
        } else if(value == highestValue) {
            rotationDegrees.push_back(degreesIndex);
        }
    }

    if(rotationDegrees.size() == 0)
        return -1;

    if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 0) != rotationDegrees.end() )
        return 0;
    if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 90) != rotationDegrees.end() )
        return 90;
    else if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 180) != rotationDegrees.end() )
        return 180;
    else if ( std::find(rotationDegrees.begin(), rotationDegrees.end(), 270) != rotationDegrees.end() )
        return 270;

    if(rotationDegrees.size() > 90)
        return rotationDegrees.at(rotationDegrees.size() / 4);
    else
        return rotationDegrees.at(rotationDegrees.size() / 2);
}