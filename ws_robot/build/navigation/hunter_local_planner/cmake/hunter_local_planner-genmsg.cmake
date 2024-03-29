# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hunter_local_planner: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ihunter_local_planner:/home/am/ws_robot/src/navigation/hunter_local_planner/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Icostmap_converter:/opt/ros/melodic/share/costmap_converter/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hunter_local_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_custom_target(_hunter_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hunter_local_planner" "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" "geometry_msgs/Vector3:geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Twist:geometry_msgs/Point"
)

get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_custom_target(_hunter_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hunter_local_planner" "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" "hunter_local_planner/TrajectoryPointMsg:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_custom_target(_hunter_local_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hunter_local_planner" "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" "hunter_local_planner/TrajectoryPointMsg:costmap_converter/ObstacleArrayMsg:geometry_msgs/Point32:geometry_msgs/Polygon:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:hunter_local_planner/TrajectoryMsg:costmap_converter/ObstacleMsg:std_msgs/Header:geometry_msgs/TwistWithCovariance:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_cpp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_cpp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleArrayMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleMsg.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(hunter_local_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hunter_local_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hunter_local_planner_generate_messages hunter_local_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_cpp _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_cpp _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_cpp _hunter_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hunter_local_planner_gencpp)
add_dependencies(hunter_local_planner_gencpp hunter_local_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hunter_local_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_eus(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_eus(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleArrayMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleMsg.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(hunter_local_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hunter_local_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hunter_local_planner_generate_messages hunter_local_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_eus _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_eus _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_eus _hunter_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hunter_local_planner_geneus)
add_dependencies(hunter_local_planner_geneus hunter_local_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hunter_local_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_lisp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_lisp(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleArrayMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleMsg.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(hunter_local_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hunter_local_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hunter_local_planner_generate_messages hunter_local_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_lisp _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_lisp _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_lisp _hunter_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hunter_local_planner_genlisp)
add_dependencies(hunter_local_planner_genlisp hunter_local_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hunter_local_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_nodejs(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_nodejs(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleArrayMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleMsg.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(hunter_local_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hunter_local_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hunter_local_planner_generate_messages hunter_local_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_nodejs _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_nodejs _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_nodejs _hunter_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hunter_local_planner_gennodejs)
add_dependencies(hunter_local_planner_gennodejs hunter_local_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hunter_local_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_py(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner
)
_generate_msg_py(hunter_local_planner
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleArrayMsg.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Polygon.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg;/opt/ros/melodic/share/costmap_converter/cmake/../msg/ObstacleMsg.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner
)

### Generating Services

### Generating Module File
_generate_module_py(hunter_local_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hunter_local_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hunter_local_planner_generate_messages hunter_local_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryPointMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_py _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/TrajectoryMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_py _hunter_local_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/am/ws_robot/src/navigation/hunter_local_planner/msg/FeedbackMsg.msg" NAME_WE)
add_dependencies(hunter_local_planner_generate_messages_py _hunter_local_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hunter_local_planner_genpy)
add_dependencies(hunter_local_planner_genpy hunter_local_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hunter_local_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hunter_local_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hunter_local_planner_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hunter_local_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET costmap_converter_generate_messages_cpp)
  add_dependencies(hunter_local_planner_generate_messages_cpp costmap_converter_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hunter_local_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hunter_local_planner_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hunter_local_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET costmap_converter_generate_messages_eus)
  add_dependencies(hunter_local_planner_generate_messages_eus costmap_converter_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hunter_local_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hunter_local_planner_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hunter_local_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET costmap_converter_generate_messages_lisp)
  add_dependencies(hunter_local_planner_generate_messages_lisp costmap_converter_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hunter_local_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hunter_local_planner_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hunter_local_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET costmap_converter_generate_messages_nodejs)
  add_dependencies(hunter_local_planner_generate_messages_nodejs costmap_converter_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hunter_local_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hunter_local_planner_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hunter_local_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET costmap_converter_generate_messages_py)
  add_dependencies(hunter_local_planner_generate_messages_py costmap_converter_generate_messages_py)
endif()
