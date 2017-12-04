cmake_minimum_required(VERSION 2.8.3)
project(kobuki_mapper)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        genmsg
)

add_message_files(
        FILES
        GridPoint.msg
)

find_package(PCL REQUIRED COMPONENTS)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs message_runtime
)

add_compile_options(-std=c++11)

add_executable(kobuki_mapper src/kobuki_mapper.cpp src/robot.cpp src/square.cpp)
target_link_libraries(kobuki_mapper ${catkin_LIBRARIES})

