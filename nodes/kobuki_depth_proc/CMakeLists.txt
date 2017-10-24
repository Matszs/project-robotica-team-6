cmake_minimum_required(VERSION 2.8.3)
project(kobuki_depth_proc)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        message_generation
        genmsg
        image_transport
        cv_bridge
        pcl_msgs
)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_msgs DEPENDS PCL
)

add_executable(kobuki_depth_proc src/kobuki_depth_proc.cpp)
target_link_libraries(kobuki_depth_proc ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

