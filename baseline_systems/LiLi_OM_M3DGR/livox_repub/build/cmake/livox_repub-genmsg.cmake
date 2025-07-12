# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "livox_repub: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilivox_repub:/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(livox_repub_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_custom_target(_livox_repub_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_repub" "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" "std_msgs/Header:livox_repub/CustomPoint"
)

get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_custom_target(_livox_repub_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_repub" "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_repub
)
_generate_msg_cpp(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_repub
)

### Generating Services

### Generating Module File
_generate_module_cpp(livox_repub
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_repub
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(livox_repub_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(livox_repub_generate_messages livox_repub_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_cpp _livox_repub_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_cpp _livox_repub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_repub_gencpp)
add_dependencies(livox_repub_gencpp livox_repub_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_repub_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_repub
)
_generate_msg_eus(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_repub
)

### Generating Services

### Generating Module File
_generate_module_eus(livox_repub
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_repub
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(livox_repub_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(livox_repub_generate_messages livox_repub_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_eus _livox_repub_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_eus _livox_repub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_repub_geneus)
add_dependencies(livox_repub_geneus livox_repub_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_repub_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_repub
)
_generate_msg_lisp(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_repub
)

### Generating Services

### Generating Module File
_generate_module_lisp(livox_repub
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_repub
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(livox_repub_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(livox_repub_generate_messages livox_repub_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_lisp _livox_repub_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_lisp _livox_repub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_repub_genlisp)
add_dependencies(livox_repub_genlisp livox_repub_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_repub_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_repub
)
_generate_msg_nodejs(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_repub
)

### Generating Services

### Generating Module File
_generate_module_nodejs(livox_repub
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_repub
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(livox_repub_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(livox_repub_generate_messages livox_repub_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_nodejs _livox_repub_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_nodejs _livox_repub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_repub_gennodejs)
add_dependencies(livox_repub_gennodejs livox_repub_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_repub_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub
)
_generate_msg_py(livox_repub
  "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub
)

### Generating Services

### Generating Module File
_generate_module_py(livox_repub
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(livox_repub_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(livox_repub_generate_messages livox_repub_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomMsg.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_py _livox_repub_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zjj/Elements/CQU_ZJJ/lili-om/livox_repub/msg/CustomPoint.msg" NAME_WE)
add_dependencies(livox_repub_generate_messages_py _livox_repub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_repub_genpy)
add_dependencies(livox_repub_genpy livox_repub_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_repub_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_repub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_repub
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(livox_repub_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(livox_repub_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_repub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_repub
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(livox_repub_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(livox_repub_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_repub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_repub
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(livox_repub_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(livox_repub_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_repub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_repub
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(livox_repub_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(livox_repub_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub)
  install(CODE "execute_process(COMMAND \"/media/zjj/Elements/anaconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_repub
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(livox_repub_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(livox_repub_generate_messages_py sensor_msgs_generate_messages_py)
endif()
