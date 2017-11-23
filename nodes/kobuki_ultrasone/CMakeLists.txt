cmake_minimum_required(VERSION 2.8.3)
project(kobuki_ultrasone)

# Find catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        genmsg
        kobuki_mapper
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_message_files(
    FILES
    UltrasoneSensors.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS message_runtime rt pthread
)

## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)

list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

add_compile_options(-std=c++11)

## Add serial library
add_library(kobuki_ultrasone_serial ${serial_SRCS})
target_link_libraries(kobuki_ultrasone_serial rt pthread)

## Uncomment for example
add_executable(kobuki_ultrasone src/kobuki_ultrasone.cpp)
add_dependencies(kobuki_ultrasone ${PROJECT_NAME})
target_link_libraries(kobuki_ultrasone kobuki_ultrasone_serial ${catkin_LIBRARIES})

## Include headers
include_directories(include)

## Install executable
install(TARGETS kobuki_ultrasone_serial
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)