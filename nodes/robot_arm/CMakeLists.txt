cmake_minimum_required(VERSION 2.8.3)
project(robot_arm)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
)

add_message_files(
  FILES
  Position.msg
)

add_service_files(
  FILES
  SetArmPosition.srv
  GetArmPosition.srv
  SetHandState.srv
  GetHandState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES navigation
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS system_lib
)

include_directories(
	include/
	${catkin_INCLUDE_DIRS})

add_executable(arm src/arm.cpp src/arm_segment.cpp)
target_link_libraries(arm ${catkin_LIBRARIES})

