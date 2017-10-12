cmake_minimum_required(VERSION 2.8.3)
project(mqtt_controller)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)


include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)

add_executable(mqtt_controller src/mqtt_controller.cpp)
target_link_libraries(mqtt_controller ${catkin_LIBRARIES})

