cmake_minimum_required(VERSION 3.0.2)
project(myslam)

set(CMAKE_CXX_STANDARD 17)

###########################################################################################
##为后续补充的增量编译方法 -
set_source_files_properties(precompiled.h PROPERTIES COMPILE_FLAGS "-x c++-header")
add_library(precompiled INTERFACE)
target_include_directories(precompiled INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(precompiled INTERFACE "-include;precompiled.h")
##########################################################################################

add_subdirectory(livox_ros_driver/livox_ros_driver)
add_subdirectory(VoxelMap)

