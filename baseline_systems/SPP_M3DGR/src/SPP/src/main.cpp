#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <ros/ros.h>
#include <so3_math.h>
#include <thread>
#include <unistd.h>
#include "voxel_mapping.hpp"
#include "../include/tool/tools_data_io.hpp"
#include "../include/tool/tools_logger.hpp"
#include "../include/tool/tools_color_printf.hpp"
#include "../include/tool/tools_eigen.hpp"
Voxel_mapping voxel_mapping;

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    LOG(INFO) << "------------------ Starting this program ------------------";
    ros::init( argc, argv, "laserMapping" );

    voxel_mapping.init_ros_node();
    // std::thread thr_lidar = std::thread( &Voxel_mapping::service_LiDAR_update, &voxel_mapping);

    // TODO fastlivo2

    // std::thread thr_camera = std::thread( &Voxel_mapping::service_Camera_update, &voxel_mapping);
    std::thread thr_gps = std::thread( &Voxel_mapping::service_GPS_update, &voxel_mapping);
    // 等待子线程结束任务后一并退出
    // thr_lidar.join();
    // thr_camera.join();
    thr_gps.join();
    google::ShutdownGoogleLogging();
    return 0;
}
