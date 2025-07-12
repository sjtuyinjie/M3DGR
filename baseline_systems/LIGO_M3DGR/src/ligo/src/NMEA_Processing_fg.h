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
// #include <mavros_msgs/GPSRAW.h>
#include <nav_msgs/Odometry.h>
#include "NMEA_Assignment.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define WINDOW_SIZE (10) // should be 0

class NMEAProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NMEAProcess();
  ~NMEAProcess();
  
  void Reset();
  void processNMEA(const nav_msgs::OdometryPtr &gnss_meas, state_output &state);
  bool NMEALIAlign();
  void TrajAlign(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&local_traj, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&enu_traj, Eigen::Vector3d &pos, Eigen::Matrix3d &rot);
  // bool TrajAlign(Eigen::Vector3d &pos, Eigen::Matrix3d &rot);
  void updateNMEAstatistics(Eigen::Vector3d &pos);
  Eigen::Vector3d local2enu(Eigen::Matrix3d enu_rot, Eigen::Vector3d anc, Eigen::Vector3d &pos);
  void SetInit();
  bool AddFactor(gtsam::Rot3 rel_rot_, gtsam::Point3 rel_pos_, gtsam::Vector3 rel_v_, Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot);
  // std::vector<ObsPtr> gnss_meas_buf[WINDOW_SIZE+1]; //
  std::vector<nav_msgs::OdometryPtr> nmea_meas_; //[WINDOW_SIZE+1]; //
  // std::vector<EphemBasePtr> gnss_ephem_buf[WINDOW_SIZE+1]; // 
  // Eigen::Matrix3d rot_window[WINDOW_SIZE+1]; //
  Eigen::Matrix3d rot_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d pos_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d vel_window[WINDOW_SIZE+1]; //
  // Eigen::Vector3d pos_window[WINDOW_SIZE+1]; //
  // Eigen::Vector3d vel_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d Tex_imu_r;
  Eigen::Matrix3d Rex_imu_r;
  std::queue<nav_msgs::OdometryPtr> nmea_msg;

  bool invalid_lidar = false;
  // double dt[4];
  // double ddt; 
  size_t id_accumulate = 0; // 
  size_t frame_delete = 0; // 

  int frame_num = 0; // 
  double last_nmea_time = 0.0; //
  double nmea_sample_period = 0.1;

  // double diff_t_gnss_local = 0.0;
  // Eigen::Vector3d ecef_pos, first_xyz_ecef_pvt, first_xyz_ecef_lla, first_lla_pvt, first_lla_lla;
  Eigen::Matrix3d Rot_nmea_init = Eigen::Matrix3d::Identity();
  bool nmea_ready = false;
  int frame_count = 0; //
  int delete_thred = 0;
  int wind_size = WINDOW_SIZE;
  int norm_vec_num = 0;
  bool nolidar = false;
  bool nolidar_cur = false;
  std::vector<Eigen::Vector3d> norm_vec_holder;
  // double para_yaw_enu_local[1];
  // double para_rcv_dt[(WINDOW_SIZE+1)*4] = {0}; //
  // double para_rcv_ddt[WINDOW_SIZE+1] = {0}; //
  Eigen::Vector3d anc_enu, gravity_init;
  Eigen::Vector3d anc_local = Eigen::Vector3d::Zero();
  // Eigen::Matrix3d R_ecef_enu;
  double yaw_enu_local = 0.0;
  
  void runISAM2opt(void);
  // void GnssPsrDoppMeas(const ObsPtr &obs_, const EphemBasePtr &ephem_);
  // void SvPosCals(const ObsPtr &obs_, const EphemBasePtr &ephem_);
  bool Evaluate(state_output &state);
  state_output state_const_;
  state_output state_const_last;
  double nmea_weight = 1.0;
  Eigen::Matrix<double, 24, 24> sqrt_lidar;
  double odo_weight1 = 1.0;
  double odo_weight2 = 1.0;
  double odo_weight3 = 1.0;
  // Eigen::Matrix<double, 3, 3> rot_weight = Eigen::Matrix3d::Identity();
  double odo_weight4 = 2.0;
  double odo_weight5 = 2.0;
  double odo_weight6 = 2.0;
  IntegrationBase* pre_integration = new IntegrationBase{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  NMEAAssignment* p_assign = new NMEAAssignment();
  // private:
    // int freq_idx = 0;
    // double freq = 0.0;
    // Eigen::Vector3d sv_pos;
    // Eigen::Vector3d sv_vel;
    // double svdt, svddt, tgd;
    // double pr_uura, dp_uura;
    // Eigen::Matrix3d rot_pos;
};

// # endif