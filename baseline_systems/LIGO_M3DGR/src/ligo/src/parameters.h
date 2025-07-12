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
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"
#include "GNSS_Processing_fg.h"
#include "NMEA_Processing_fg.h"
#include "IMU_Processing.h"
// #include "LI_init/LI_init.h"
#include "Urbannav_process/handler.h"
#include <sensor_msgs/NavSatFix.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <omp.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ivox/ivox3d.h>
#include <Python.h>
#include <condition_variable>
#include <sensor_msgs/Imu.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Vector3.h>
#include <Curvefitter/curvefitter.hpp>

using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;

extern std::vector<curvefitter::PoseData> pose_graph_key_pose;
extern std::vector<double> pose_time_vector;
extern std::vector<std::vector<Eigen::Vector3d> > LiDAR_points;
extern int points_num;
extern double map_time;
extern typename curvefitter::TrajectoryManager<4>::Ptr traj_manager;
extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern state_output state_out;
extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame;
extern bool space_down_sample;
extern bool publish_odometry_without_downsample;
extern int  init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro;
extern float  plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern float  DET_RANGE;
extern bool   imu_en, init_with_imu;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov; 
extern int    lidar_type, pcd_save_interval;
extern int    gt_file_type;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT, extrinT_gnss;
extern std::vector<double> extrinR, extrinR_gnss;
extern std::vector<double> ppp_anc;
extern bool   runtime_pos_log, pcd_save_en, path_en;
extern bool   scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern shared_ptr<GNSSProcess> p_gnss;
extern shared_ptr<NMEAProcess> p_nmea;
extern bool is_first_frame;
extern bool dyn_filter;
extern double dyn_filter_resolution;

extern std::string gnss_ephem_topic, gnss_glo_ephem_topic, gnss_meas_topic, gnss_iono_params_topic;
extern std::string gt_fname, ephem_fname, ppp_fname;
extern std::string gnss_tp_info_topic, local_trigger_info_topic, rtk_pvt_topic, rtk_lla_topic;
extern std::string nmea_meas_topic;
extern std::vector<double> default_gnss_iono_params;
extern double gnss_local_time_diff, gnss_ekf_noise;
extern bool next_pulse_time_valid, update_gnss, update_nmea;
extern bool time_diff_valid, is_first_gnss, is_first_nmea;
extern double latest_gnss_time, next_pulse_time, last_nmea_time; 
extern double time_diff_gnss_local, time_diff_nmea_local;
extern bool gnss_local_online_sync, nolidar; 
extern double li_init_gyr_cov, li_init_acc_cov, lidar_time_inte, first_imu_time;
extern int orig_odom_freq;
extern double online_refine_time; //unit: s
extern bool GNSS_ENABLE;
extern bool NMEA_ENABLE;
extern double time_update_last, time_current, time_predict_last_const, t_last;
extern Eigen::Matrix3d Rot_gnss_init;

extern MeasureGroup Measures;

extern std::vector<Eigen::Vector3d> est_poses;
extern std::vector<Eigen::Vector3d> local_poses;
extern std::vector<Eigen::Matrix3d> local_rots;
extern std::vector<double> time_frame;

extern ofstream fout_out, fout_rtk, fout_global, fout_ppp;
void readParameters(ros::NodeHandle &n);
void open_file();
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &orient);
void cout_state_to_file(Eigen::Vector3d &pos_enu);
void cout_state_to_file_nmea();
void reset_cov_output(Eigen::Matrix<double, 24, 24> & P_init_output);