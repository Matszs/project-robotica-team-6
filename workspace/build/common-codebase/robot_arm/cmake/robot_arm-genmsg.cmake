# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_arm: 1 messages, 4 services")

set(MSG_I_FLAGS "-Irobot_arm:/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_arm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_custom_target(_robot_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_arm" "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" ""
)

get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_custom_target(_robot_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_arm" "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" "robot_arm/Position"
)

get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_custom_target(_robot_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_arm" "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" "robot_arm/Position"
)

get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_custom_target(_robot_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_arm" "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" ""
)

get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_custom_target(_robot_arm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_arm" "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
)

### Generating Services
_generate_srv_cpp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
)
_generate_srv_cpp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
)
_generate_srv_cpp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
)
_generate_srv_cpp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
)

### Generating Module File
_generate_module_cpp(robot_arm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_arm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_arm_generate_messages robot_arm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_cpp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_cpp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_cpp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_cpp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_dependencies(robot_arm_generate_messages_cpp _robot_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_arm_gencpp)
add_dependencies(robot_arm_gencpp robot_arm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_arm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
)

### Generating Services
_generate_srv_eus(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
)
_generate_srv_eus(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
)
_generate_srv_eus(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
)
_generate_srv_eus(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
)

### Generating Module File
_generate_module_eus(robot_arm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_arm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_arm_generate_messages robot_arm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_eus _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_eus _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_eus _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_eus _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_dependencies(robot_arm_generate_messages_eus _robot_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_arm_geneus)
add_dependencies(robot_arm_geneus robot_arm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_arm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
)

### Generating Services
_generate_srv_lisp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
)
_generate_srv_lisp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
)
_generate_srv_lisp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
)
_generate_srv_lisp(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
)

### Generating Module File
_generate_module_lisp(robot_arm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_arm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_arm_generate_messages robot_arm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_lisp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_lisp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_lisp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_lisp _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_dependencies(robot_arm_generate_messages_lisp _robot_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_arm_genlisp)
add_dependencies(robot_arm_genlisp robot_arm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_arm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
)

### Generating Services
_generate_srv_nodejs(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
)
_generate_srv_nodejs(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
)
_generate_srv_nodejs(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
)
_generate_srv_nodejs(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
)

### Generating Module File
_generate_module_nodejs(robot_arm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_arm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_arm_generate_messages robot_arm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_nodejs _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_nodejs _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_nodejs _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_nodejs _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_dependencies(robot_arm_generate_messages_nodejs _robot_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_arm_gennodejs)
add_dependencies(robot_arm_gennodejs robot_arm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_arm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
)

### Generating Services
_generate_srv_py(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
)
_generate_srv_py(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
)
_generate_srv_py(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
)
_generate_srv_py(robot_arm
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv"
  "${MSG_I_FLAGS}"
  "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
)

### Generating Module File
_generate_module_py(robot_arm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_arm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_arm_generate_messages robot_arm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_py _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_py _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/SetArmPosition.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_py _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/srv/GetHandState.srv" NAME_WE)
add_dependencies(robot_arm_generate_messages_py _robot_arm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/turtlebot6/workspace/src/common-codebase/robot_arm/msg/Position.msg" NAME_WE)
add_dependencies(robot_arm_generate_messages_py _robot_arm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_arm_genpy)
add_dependencies(robot_arm_genpy robot_arm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_arm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_arm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_arm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_arm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_arm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_arm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_arm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_arm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_arm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_arm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_arm_generate_messages_py std_msgs_generate_messages_py)
endif()
