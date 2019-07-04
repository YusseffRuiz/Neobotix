# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cob_srvs: 0 messages, 10 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cob_srvs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" "std_msgs/String"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" "std_msgs/String:std_msgs/Bool"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" "std_msgs/String:std_msgs/Bool"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" "std_msgs/String"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" "std_msgs/String"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" "std_msgs/String:std_msgs/Bool"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" "trajectory_msgs/JointTrajectory:std_msgs/String:trajectory_msgs/JointTrajectoryPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" "std_msgs/String:std_msgs/Bool"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" "std_msgs/String"
)

get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_custom_target(_cob_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cob_srvs" "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)
_generate_srv_cpp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
)

### Generating Module File
_generate_module_cpp(cob_srvs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cob_srvs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cob_srvs_generate_messages cob_srvs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_cpp _cob_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cob_srvs_gencpp)
add_dependencies(cob_srvs_gencpp cob_srvs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_srvs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)
_generate_srv_eus(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
)

### Generating Module File
_generate_module_eus(cob_srvs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cob_srvs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cob_srvs_generate_messages cob_srvs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_eus _cob_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cob_srvs_geneus)
add_dependencies(cob_srvs_geneus cob_srvs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_srvs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)
_generate_srv_lisp(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
)

### Generating Module File
_generate_module_lisp(cob_srvs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cob_srvs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cob_srvs_generate_messages cob_srvs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_lisp _cob_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cob_srvs_genlisp)
add_dependencies(cob_srvs_genlisp cob_srvs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_srvs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)
_generate_srv_nodejs(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
)

### Generating Module File
_generate_module_nodejs(cob_srvs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cob_srvs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cob_srvs_generate_messages cob_srvs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_nodejs _cob_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cob_srvs_gennodejs)
add_dependencies(cob_srvs_gennodejs cob_srvs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_srvs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectory.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/JointTrajectoryPoint.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)
_generate_srv_py(cob_srvs
  "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Bool.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
)

### Generating Module File
_generate_module_py(cob_srvs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cob_srvs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cob_srvs_generate_messages cob_srvs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetFloat.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetDefaultVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointStiffness.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetInt.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetString.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetOperationMode.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetJointTrajectory.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/SetMaxVel.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/yusseff/Neobotix/src/cob_common/cob_srvs/srv/GetPoseStampedTransformed.srv" NAME_WE)
add_dependencies(cob_srvs_generate_messages_py _cob_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cob_srvs_genpy)
add_dependencies(cob_srvs_genpy cob_srvs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cob_srvs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cob_srvs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(cob_srvs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(cob_srvs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(cob_srvs_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cob_srvs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(cob_srvs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(cob_srvs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(cob_srvs_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cob_srvs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(cob_srvs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(cob_srvs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(cob_srvs_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cob_srvs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(cob_srvs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(cob_srvs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(cob_srvs_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cob_srvs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(cob_srvs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(cob_srvs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(cob_srvs_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
