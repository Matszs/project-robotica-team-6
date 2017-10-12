cmake_minimum_required(VERSION 2.8.3)
project(walker)

find_package(catkin REQUIRED COMPONENTS
        ecl_threads
        geometry_msgs
        kobuki_msgs
        roscpp
        std_msgs
        yocs_controllers
)

add_message_files(
        FILES
        TrackedPosition.msg
        SetTrackingColours.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS ecl_threads
                              geometry_msgs
                              kobuki_msgs
                              roscpp
                              std_msgs
                              yocs_controllers)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)

add_executable(walker src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})

