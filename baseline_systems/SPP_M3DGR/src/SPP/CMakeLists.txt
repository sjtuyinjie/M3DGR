cmake_minimum_required(VERSION 3.0.2)
# project(VoxelMap)
project(gnss_spp)
#SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_BUILD_TYPE "Debug")
ADD_COMPILE_OPTIONS(-std=c++17 )


add_definitions(-DROOT_DIR="/home/supercoconut/Myfile/myslam/voxel_ws/src/VoxelMap")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++17 -g -O3 -ltbb -lboost_system -msse2 -msse3 -pthread -w") # -Wall
# 暂时修改优化选项用于debug
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++17 -g -O0 -ltbb -lboost_system -msse2 -msse3 -pthread -w") # -Wall
set(CMAKE_BUILD_PARALLEL_LEVEL 24)

# 关于当前的CPU版本设置对应的openMP的线程数
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
    include(ProcessorCount)
    ProcessorCount(N)
    message("Processer number:  ${N}")
    if(N GREATER 5)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=4)
        message("core for MP:  4")
    elseif(N GREATER 3)
        math(EXPR PROC_NUM "${N} - 2")
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
        message("core for MP:  ${PROC_NUM}")
    else()
        add_definitions(-DMP_PROC_NUM=1)
    endif()
else()
    add_definitions(-DMP_PROC_NUM=1)
endif()


message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")



find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    livox_ros_driver
    nav_msgs
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf

    message_generation
    eigen_conversions
)


find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CGAL REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
set(Sophus_LIBRARIES libSophus.so)

# set(Glog_DIR /home/supercoconut/Myfile/thirdparty/glog-0.4.0/build)
# find_package( Glog  REQUIRED PATHS /home/supercoconut/Myfile/thirdparty/glog-0.4.0 NO_DEFAULT_PATH)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        # ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${glog_INCLUDE_DIRS}
        ./include
#        ./include/voxel_mapping.hpp
#        ./include/IMU_Processing.h
#        ./include/preprocess.h
)

add_message_files(
        FILES
        Pose6D.msg
        # States.msg
)


generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
        DEPENDS EIGEN3 PCL OpenCV Sophus
        INCLUDE_DIRS
)


## 补充编译
# add_executable(voxelmap_mapping
add_executable(spp
    # src
    ./src/main.cpp
    ./src/IMU_Processing.cpp
    ./src/preprocess.cpp
    ./src/voxel_mapping.cpp
    ./src/voxel_loc.cpp
    ./src/voxel_mapping_common.cpp
    ./src/ikd_Tree.cpp
)

# target_link_libraries(voxelmap_mapping
target_link_libraries(spp
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        CGAL::CGAL
        tbb
        # glog::glog
        ${glog_LDFLAGS}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Boost_SYSTEM_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SERIALIZATION_LIBRARY} # serialization
        ${CERES_LIBRARIES}
        pcl_common
        pcl_io
        pcl_filters
        pcl_kdtree
        GL GLU glfw X11 Xrandr pthread Xi dl Xinerama Xcursor)
