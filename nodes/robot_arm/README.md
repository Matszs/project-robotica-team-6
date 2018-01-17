# Robot Arm
This package is responsible for controlling the PhantomX Reactor Robot Arm.
It allows nodes to send a request for the robot arm to move to a certain
position using x, y, z coordinates.
This package uses inverse kinematics to calculate the correct angles for each
joint of the robot arm and it sets the robot arm in the correct position using
the calculated angles.
