cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# -DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# NEW
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_models
  image_transport

  rospy

  actionlib
  darknet_ros_msgs

  nodelet

  # gnss new
  rosbag
  message_generation
  gnss_comm
)

# gnss new
add_message_files(
  DIRECTORY msg
  FILES LocalSensorExternalTrigger.msg
)

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../darknet/")
message(STATUS "Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Find CUDA
find_package(CUDA QUIET)

if(CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
message(STATUS "Searching for X11...")
find_package(X11 REQUIRED)

if(X11_FOUND)
  include_directories(${X11_INCLUDE_DIR})
  link_libraries(${X11_LIBRARIES})
  message(STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR})
  message(STATUS " X11_LIBRARIES: " ${X11_LIBRARIES})
endif(X11_FOUND)

# Find rquired packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

include_directories(
  ${DARKNET_PATH}/src
  ${DARKNET_PATH}/include
  include
  ${Boost_INCLUDE_DIRS}

  # ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
  src/YoloObjectDetector.cpp src/image_interface.cpp
)

set(DARKNET_CORE_FILES
  ${DARKNET_PATH}/src/activation_layer.c ${DARKNET_PATH}/src/im2col.c
  ${DARKNET_PATH}/src/activations.c ${DARKNET_PATH}/src/image.c
  ${DARKNET_PATH}/src/avgpool_layer.c ${DARKNET_PATH}/src/layer.c
  ${DARKNET_PATH}/src/batchnorm_layer.c ${DARKNET_PATH}/src/list.c
  ${DARKNET_PATH}/src/blas.c ${DARKNET_PATH}/src/local_layer.c
  ${DARKNET_PATH}/src/box.c ${DARKNET_PATH}/src/lstm_layer.c
  ${DARKNET_PATH}/src/col2im.c ${DARKNET_PATH}/src/matrix.c
  ${DARKNET_PATH}/src/connected_layer.c ${DARKNET_PATH}/src/maxpool_layer.c
  ${DARKNET_PATH}/src/convolutional_layer.c ${DARKNET_PATH}/src/network.c
  ${DARKNET_PATH}/src/cost_layer.c ${DARKNET_PATH}/src/normalization_layer.c
  ${DARKNET_PATH}/src/crnn_layer.c ${DARKNET_PATH}/src/option_list.c
  ${DARKNET_PATH}/src/crop_layer.c ${DARKNET_PATH}/src/parser.c
  ${DARKNET_PATH}/src/cuda.c ${DARKNET_PATH}/src/region_layer.c
  ${DARKNET_PATH}/src/data.c ${DARKNET_PATH}/src/reorg_layer.c
  ${DARKNET_PATH}/src/deconvolutional_layer.c ${DARKNET_PATH}/src/rnn_layer.c
  ${DARKNET_PATH}/src/demo.c ${DARKNET_PATH}/src/route_layer.c
  ${DARKNET_PATH}/src/detection_layer.c ${DARKNET_PATH}/src/shortcut_layer.c
  ${DARKNET_PATH}/src/dropout_layer.c ${DARKNET_PATH}/src/softmax_layer.c
  ${DARKNET_PATH}/src/gemm.c ${DARKNET_PATH}/src/tree.c
  ${DARKNET_PATH}/src/gru_layer.c ${DARKNET_PATH}/src/utils.c
  ${DARKNET_PATH}/src/upsample_layer.c ${DARKNET_PATH}/src/logistic_layer.c
  ${DARKNET_PATH}/src/l2norm_layer.c ${DARKNET_PATH}/src/yolo_layer.c
  ${DARKNET_PATH}/src/iseg_layer.c ${DARKNET_PATH}/src/image_opencv.cpp

  ${DARKNET_PATH}/examples/art.c ${DARKNET_PATH}/examples/lsd.c
  ${DARKNET_PATH}/examples/nightmare.c ${DARKNET_PATH}/examples/instance-segmenter.c
  ${DARKNET_PATH}/examples/captcha.c ${DARKNET_PATH}/examples/regressor.c
  ${DARKNET_PATH}/examples/cifar.c ${DARKNET_PATH}/examples/rnn.c
  ${DARKNET_PATH}/examples/classifier.c ${DARKNET_PATH}/examples/segmenter.c
  ${DARKNET_PATH}/examples/coco.c ${DARKNET_PATH}/examples/super.c
  ${DARKNET_PATH}/examples/darknet.c ${DARKNET_PATH}/examples/tag.c
  ${DARKNET_PATH}/examples/detector.c ${DARKNET_PATH}/examples/yolo.c
  ${DARKNET_PATH}/examples/go.c
)

set(DARKNET_CUDA_FILES
  ${DARKNET_PATH}/src/activation_kernels.cu ${DARKNET_PATH}/src/crop_layer_kernels.cu
  ${DARKNET_PATH}/src/avgpool_layer_kernels.cu ${DARKNET_PATH}/src/deconvolutional_kernels.cu
  ${DARKNET_PATH}/src/blas_kernels.cu ${DARKNET_PATH}/src/dropout_layer_kernels.cu
  ${DARKNET_PATH}/src/col2im_kernels.cu ${DARKNET_PATH}/src/im2col_kernels.cu
  ${DARKNET_PATH}/src/convolutional_kernels.cu ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  "/usr/local/include/fmt"
  # line
  ${CMAKE_CURRENT_SOURCE_DIR}/src/linefeatureTracker/line_descriptor/include
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES

  # ${PROJECT_NAME}_lib
  darknet_ros_lib
  CATKIN_DEPENDS
  cv_bridge
  roscpp
  actionlib
  rospy
  std_msgs
  darknet_ros_msgs
  image_transport
  nodelet
  DEPENDS
  Boost
)

if(CUDA_FOUND)
  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  )

  cuda_add_library(darknet_ros_lib
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
    ${DARKNET_CUDA_FILES}
  )

  target_link_libraries(darknet_ros_lib
    cuda
    cudart
    cublas
    curand
  )

  cuda_add_executable(darknet_ros
    src/yolo_object_detector_node.cpp
  )

  cuda_add_library(darknet_ros_nodelet
    src/yolo_object_detector_nodelet.cpp
  )

else()
  add_library(darknet_ros_lib
    ${PROJECT_LIB_FILES} ${DARKNET_CORE_FILES}
  )

  add_executable(darknet_ros
    src/yolo_object_detector_node.cpp
  )

  add_library(darknet_ros_nodelet
    src/yolo_object_detector_nodelet.cpp
  )
endif()

target_link_libraries(darknet_ros_lib
  m
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(darknet_ros
  darknet_ros_lib
)

target_link_libraries(darknet_ros_nodelet
  darknet_ros_lib
)

add_dependencies(darknet_ros_lib
  darknet_ros_msgs_generate_messages_cpp
)

install(TARGETS darknet_ros_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS darknet_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/darknet_ros/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch yolo_network_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Download yolov2-tiny.weights
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
set(FILE "${PATH}/yolov2-tiny.weights")
message(STATUS "Checking and downloading yolov2-tiny.weights if needed ...")

# if (NOT EXISTS "${FILE}")
# message(STATUS "... file does not exist. Downloading now ...")
# execute_process(COMMAND wget -q https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov2-tiny.weights -P ${PATH})
# endif()

# Download yolov3.weights
set(FILE "${PATH}/yolov3.weights")
message(STATUS "Checking and downloading yolov3.weights if needed ...")

# if (NOT EXISTS "${FILE}")
# message(STATUS "... file does not exist. Downloading now ...")
# execute_process(COMMAND wget -q https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov3.weights -P ${PATH})
# endif()
add_library(vins_lib
  src/estimator/parameters.cpp
  src/estimator/estimator.cpp
  src/estimator/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/pose_subset_parameterization.cpp # new
  src/factor/orientation_local_parameterization.cpp
  src/factor/orientation_subset_parameterization.cpp
  src/factor/projectionTwoFrameOneCamFactor.cpp
  src/factor/projectionTwoFrameTwoCamFactor.cpp
  src/factor/projectionOneFrameTwoCamFactor.cpp
  src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  src/utility/visualization.cpp
  src/utility/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  src/featureTracker/feature_tracker.cpp

  # line
  src/linefeatureTracker/linefeature_tracker.cpp
  src/linefeatureTracker/line_descriptor/src/binary_descriptor_custom.cpp
  src/linefeatureTracker/line_descriptor/src/binary_descriptor_matcher.cpp
  src/linefeatureTracker/line_descriptor/src/draw_custom.cpp
  src/linefeatureTracker/line_descriptor/src/LSDDetector_custom.cpp
  src/utility/line_geometry.cpp
  src/factor/line_projection_factor.cpp
  src/factor/line_parameterization.cpp
  src/factor/projection_factor.cpp # new

  # gnss new
  src/factor/gnss_psr_dopp_factor.cpp
  src/factor/gnss_dt_ddt_factor.cpp
  src/factor/gnss_dt_anchor_factor.cpp
  src/factor/gnss_ddt_smooth_factor.cpp
  src/factor/pos_vel_factor.cpp
  src/factor/pose_anchor_factor.cpp
  src/initial/gnss_vi_initializer.cpp
)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} /usr/local/lib/libSophus.so /usr/local/lib/libfmt.a)

add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib /usr/local/lib/libSophus.so
  ${CMAKE_CURRENT_SOURCE_DIR}/src/linefeatureTracker/line_descriptor/lib/liblinedesc.so
  )

# add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
# target_link_libraries(kitti_odom_test vins_lib)

# add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# target_link_libraries(kitti_gps_test vins_lib)
