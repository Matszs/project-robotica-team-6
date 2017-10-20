cmake_minimum_required(VERSION 2.8.3)
project(kobuki_mapper)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
)


find_package(PCL REQUIRED COMPONENTS)



include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs
)

add_executable(kobuki_mapper src/kobuki_mapper.cpp)
target_link_libraries(kobuki_mapper ${catkin_LIBRARIES})

