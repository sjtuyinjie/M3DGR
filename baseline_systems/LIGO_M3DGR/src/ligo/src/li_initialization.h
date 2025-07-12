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

#include <common_lib.h>
#include "Estimator.h"
#define MAXN                (720000)

extern bool data_accum_finished, data_accum_start, online_calib_finish, refine_print;
extern int frame_num_init;
extern double time_lag_IMU_wtr_lidar, move_start_time, online_calib_starts_time; //, mean_acc_norm = 9.81;

extern double timediff_imu_wrt_lidar;
extern bool timediff_set_flg;
extern V3D gravity_lio;
extern mutex mtx_buffer;
extern condition_variable sig_buffer;
extern int loop_count;
extern int scan_count_point;
extern int frame_ct, wait_num;
extern std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
extern std::deque<double>               time_buffer;
extern std::deque<sensor_msgs::Imu::Ptr> imu_deque;
extern std::queue<std::vector<ObsPtr>> gnss_meas_buf;
extern std::queue<nav_msgs::OdometryPtr> nmea_meas_buf;
extern std::mutex m_time;
extern bool lidar_pushed, imu_pushed;
extern double imu_first_time;
extern bool lose_lid;
extern sensor_msgs::Imu imu_last, imu_next;
extern PointCloudXYZI::Ptr  ptr_con;
extern double s_plot[MAXN], s_plot3[MAXN];
extern bool first_gps;
extern Eigen::Vector3d first_gps_lla;
extern Eigen::Vector3d first_gps_ecef;
// extern sensor_msgs::Imu::ConstPtr imu_last_ptr;

void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg);
void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg);
void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg);
void rtk_pvt_callback(const GnssPVTSolnMsgConstPtr &groundt_pvt);
void rtk_lla_callback(const sensor_msgs::NavSatFixConstPtr &lla_msg);
void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg);
void gnss_meas_callback_urbannav(const nlosExclusion::GNSS_Raw_ArrayConstPtr &meas_msg);
void local_trigger_info_callback(const ligo::LocalSensorExternalTriggerConstPtr &trigger_msg);
void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg);
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg); 
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg); 
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in); 
// void LI_Init_set();
bool sync_packages(MeasureGroup &meas, queue<std::vector<ObsPtr>> &gnss_msg, queue<nav_msgs::OdometryPtr> &nmea_msg);

// bool sync_packages_nmea(MeasureGroup &meas, queue<nav_msgs::Odometry> &nmea_msg);
void nmea_meas_callback(const nav_msgs::OdometryConstPtr &meas_msg);
void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg);

// #endif