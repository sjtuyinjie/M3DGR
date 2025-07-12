/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <cmath>
#include <math.h>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
#include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Eigen>
// #include "Estimator.h"
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)
const bool time_list(PointType &x, PointType &y); // {return (x.curvature < y.curvature);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);
  void pointBodyToWorld_li_init(PointType const * const pi, PointType * const po);

  state_output state_LI_Init;
  MD(12, 12) state_cov = MD(12, 12)::Identity();
  int    lidar_type;
  V3D    gravity_;
  bool   imu_en;
  V3D    mean_acc;
  bool   imu_need_init_ = true;
  bool   after_imu_init_ = false;
  bool   b_first_frame_ = true;
  double time_last_scan = 0.0;
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001);
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001);

 private:
  void IMU_init(const MeasureGroup &meas, int &N);
  void Forward_propagation_without_imu(const MeasureGroup &meas, PointCloudXYZI &pcl_out);
  V3D mean_gyr;
  int init_iter_num = 1;
};
