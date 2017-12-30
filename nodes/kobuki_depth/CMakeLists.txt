cmake_minimum_required(VERSION 2.8.3)
project(kobuki_depth)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        message_generation
        genmsg
        image_transport
        cv_bridge
        pcl_msgs
        kobuki_mapper
)

add_message_files(
        FILES
        CameraPoint.msg
)


find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)


include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_msgs message_runtime DEPENDS PCL
)

add_executable(kobuki_depth src/kobuki_depth.cpp)
add_dependencies(kobuki_depth kobuki_depth_generate_messages_cpp)
target_link_libraries(kobuki_depth ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

