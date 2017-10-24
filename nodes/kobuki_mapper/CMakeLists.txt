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

#add_dependencies(kobuki_mapper kobuki_mapper_generate_messages_cpp)
#add_dependencies(mapper ${catkin_EXPORTED_TARGETS})

add_executable(kobuki_mapper src/kobuki_mapper.cpp)
target_link_libraries(kobuki_mapper ${catkin_LIBRARIES})

