cmake_minimum_required(VERSION 3.0.2)
project(dmvio_ros)

set(CMAKE_CXX_STANDARD 14)

IF(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF()
set(BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  sensor_msgs
  message_generation
  rosbag
)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer regex)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Pangolin 0.2 REQUIRED)
find_package(OpenCV REQUIRED)

# IMPORTANT: For compiling this project you need to set the environment variable $DMVIO_BUILD
# to point to the build folder of the main DM-VIO project.
# We expect that the build folder is a direct subfolder of the DM-VIO project folder.
set(DMVIO_BUILD $ENV{DMVIO_BUILD})
message("\n---- Path to DM-VIO build folder is: ${DMVIO_BUILD}\n")

set(DMVIO_ROOT ${DMVIO_BUILD}/..)

find_library(DMVIO_LIBRARY dmvio ${DMVIO_BUILD}/lib/)


IF(${Boost_VERSION} GREATER_EQUAL 106500)
 message("Building with stacktrace support.")
 set(STACKTRACE_LIBRARIES dl)
 set(STACKTRACE_DEFINES "-DSTACKTRACE -DBOOST_STACKTRACE_USE_ADDR2LINE")
ENDIF()


add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS}"
)
set(CMAKE_CXX_FLAGS_DEBUG
    "-O2 -g -fno-omit-frame-pointer -DEIGEN_INITIALIZE_MATRICES_WITH_NAN -DDEBUG ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "-O3 -g -fno-omit-frame-pointer ${STACKTRACE_DEFINES}"
)
set(CMAKE_CXX_FLAGS_RELEASE
    "-O3 -DNDEBUG"
)

add_message_files(
  FILES
  DMVIOPoseMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)


catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp sensor_msgs message_runtime
)

include_directories(
 include
  ${DMVIO_ROOT}/src
  ${DMVIO_ROOT}/src/dso
  ${EIGEN3_INCLUDE_DIR}
  ${DMVIO_ROOT}/thirdparty/Sophus
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
add_executable(${PROJECT_NAME}_node src/main.cpp src/ROSOutputWrapper.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

set(DMVIO_LINKED_LIBRARIES boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} gtsam ${YAML_CPP_LIBRARIES} ${STACKTRACE_LIBRARIES})

target_link_libraries(${PROJECT_NAME}_node
  ${DMVIO_LIBRARY}
  ${DMVIO_LINKED_LIBRARIES}
  ${catkin_LIBRARIES}
)