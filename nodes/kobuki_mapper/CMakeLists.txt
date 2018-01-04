cmake_minimum_required(VERSION 2.8.3)
project(kobuki_mapper)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        genmsg
        tf
)

add_message_files(
        FILES
        GridPoint.msg
        Info.msg
        Obstacle.msg
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

add_executable(kobuki_mapper
	src/kobuki_mapper.cpp
	src/module_loader.cpp
	src/modules/module.cpp
	src/modules/button.cpp
	src/modules/object_tracker.cpp
	src/modules/location.cpp
	src/modules/battery.cpp
	src/modules/bumper.cpp
	src/modules/ultrasonic.cpp
	src/modules/info_publisher.cpp
	src/modules/drive.cpp
	src/rotation.cpp
	src/map.cpp)
target_link_libraries(kobuki_mapper ${catkin_LIBRARIES})

