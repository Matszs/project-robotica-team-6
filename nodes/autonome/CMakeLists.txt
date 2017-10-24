cmake_minimum_required(VERSION 2.8.3)
project(autonome)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        geometry_msgs
        kobuki_mapper
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
)

add_executable(autonome src/autonome.cpp)
target_link_libraries(autonome ${catkin_LIBRARIES})

add_dependencies(autonome ${catkin_EXPORTED_TARGETS})

