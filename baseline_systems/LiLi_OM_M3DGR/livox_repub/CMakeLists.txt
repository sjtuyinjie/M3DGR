cmake_minimum_required(VERSION 3.0.2)
project(livox_repub)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    message_generation
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    eigen_conversions
    tf_conversions
    roslib
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CustomMsg.msg
  CustomPoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
)

add_executable(livox_repub_offline_node node/livox_repub_offline_node.cpp)
#target_link_libraries(livox_repub_offline_node ${thirdparty_libraries})
add_dependencies(livox_repub_offline_node  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(livox_repub_online_node node/livox_repub_online_node.cpp)
target_link_libraries(livox_repub_online_node ${thirdparty_libraries})
add_dependencies(livox_repub_online_node  ${PROJECT_NAME}_generate_messages_cpp)
