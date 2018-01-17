cmake_minimum_required(VERSION 2.8.3)
project(vision)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        genmsg
        image_transport
        cv_bridge
        pcl_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)

add_message_files(
        FILES
        TrackedPosition.msg
        SetTrackingColours.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs message_runtime pcl_msgs DEPENDS PCL
)

add_executable(tracker src/vision.cpp)
target_link_libraries(tracker ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

