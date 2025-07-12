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

#include "parameters.h"

typename curvefitter::TrajectoryManager<4>::Ptr traj_manager = std::make_shared<curvefitter::TrajectoryManager<4>>();
bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<curvefitter::PoseData> pose_graph_key_pose;
std::vector<double> pose_time_vector;
std::vector<std::vector<Eigen::Vector3d> > LiDAR_points;
int points_num;
double map_time;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false;
bool space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
float  DET_RANGE = 450;
bool   imu_en = true;
bool   init_with_imu = true;
double imu_time_inte = 0.005, gnss_ekf_noise = 0.01;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
int    gt_file_type;
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT(3, 0.0), extrinT_gnss(3, 0.0);
std::vector<double> extrinR(9, 0.0), extrinR_gnss(9, 0.0);
std::vector<double> ppp_anc(3, 0.0);
bool   runtime_pos_log, pcd_save_en, path_en;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
// shared_ptr<LI_Init> Init_LI;
shared_ptr<ImuProcess> p_imu;
shared_ptr<GNSSProcess> p_gnss;
shared_ptr<NMEAProcess> p_nmea;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

std::string gnss_ephem_topic, gnss_glo_ephem_topic, gnss_meas_topic, gnss_iono_params_topic;
std::string gt_fname, ephem_fname, ppp_fname;
std::string gnss_tp_info_topic, local_trigger_info_topic, rtk_pvt_topic, rtk_lla_topic;
std::string nmea_meas_topic;
std::vector<double> default_gnss_iono_params(8, 0.0);
double gnss_local_time_diff = 18.0;
bool next_pulse_time_valid = false, update_gnss = false, update_nmea = false;
bool time_diff_valid = false, is_first_gnss = true, is_first_nmea;
double latest_gnss_time = -1, next_pulse_time = 0.0, last_nmea_time = -1; 
double time_diff_gnss_local = 0.0, time_diff_nmea_local = 0.0;
bool gnss_local_online_sync = true, nolidar = false; 
double li_init_gyr_cov = 0.1, li_init_acc_cov = 0.1, lidar_time_inte = 0.1, first_imu_time = 0.0;
int orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool GNSS_ENABLE = true;
bool NMEA_ENABLE = true;
bool dyn_filter = false;
double dyn_filter_resolution = 1.0;
Eigen::Matrix3d Rot_gnss_init(Eye3d);
std::vector<Eigen::Vector3d> est_poses;
std::vector<Eigen::Vector3d> local_poses;
std::vector<Eigen::Matrix3d> local_rots;
std::vector<double> time_frame;

MeasureGroup Measures;

ofstream fout_out, fout_rtk, fout_global, fout_ppp; 

void readParameters(ros::NodeHandle &nh)
{
  p_pre.reset(new Preprocess());
//   Init_LI.reset(new LI_Init());
  p_imu.reset(new ImuProcess());
  p_gnss.reset(new GNSSProcess());
  p_nmea.reset(new NMEAProcess());
  nh.param<bool>("prop_at_freq_of_imu", prop_at_freq_of_imu, 1);
  nh.param<bool>("check_satu", check_satu, 1);
  nh.param<int>("init_map_size", init_map_size, 100);
  nh.param<bool>("space_down_sample", space_down_sample, 1);
  nh.param<double>("mapping/satu_acc",satu_acc,3.0);
  nh.param<double>("mapping/satu_gyro",satu_gyro,35.0);
  nh.param<double>("mapping/acc_norm",acc_norm,1.0);
  nh.param<float>("mapping/plane_thr", plane_thr, 0.05f);
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh.param<std::string>("common/lid_topic",lid_topic,"/livox/lidar");
  nh.param<std::string>("common/imu_topic", imu_topic,"/livox/imu");
  nh.param<bool>("common/con_frame",con_frame,false);
  nh.param<int>("common/con_frame_num",con_frame_num,1);
  nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
  nh.param<double>("filter_size_map",filter_size_map_min,0.5);
  nh.param<float>("mapping/det_range",DET_RANGE,300.f);
  nh.param<double>("mapping/fov_degree",fov_deg,180);
  nh.param<bool>("mapping/imu_en",imu_en,true);
  nh.param<bool>("mapping/init_with_imu",init_with_imu,true);
  nh.param<double>("mapping/imu_time_inte",imu_time_inte,0.005);
  nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);
  nh.param<double>("mapping/acc_cov_input",acc_cov_input,0.1);
  nh.param<double>("mapping/vel_cov",vel_cov,20);
  nh.param<double>("mapping/gyr_cov_input",gyr_cov_input,0.1);
  nh.param<double>("mapping/gyr_cov_output",gyr_cov_output,0.1);
  nh.param<double>("mapping/acc_cov_output",acc_cov_output,0.1);
  nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
  nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
  nh.param<double>("mapping/imu_meas_acc_cov",imu_meas_acc_cov,0.1);
  nh.param<double>("mapping/imu_meas_omg_cov",imu_meas_omg_cov,0.1);
  nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
  nh.param<double>("preprocess/det_range", p_pre->det_range, 1.0);
  nh.param<int>("preprocess/lidar_type", lidar_type, 1);
  nh.param<int>("gnss/gt_file_type", gt_file_type, 1);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, 1);
  nh.param<double>("mapping/match_s", match_s, 81);
  nh.param<std::vector<double>>("mapping/gravity", gravity, std::vector<double>());
  nh.param<std::vector<double>>("mapping/gravity_init", gravity_init, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
  nh.param<std::vector<double>>("nmea/ppp_anc", ppp_anc, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
  nh.param<bool>("odometry/publish_odometry_without_downsample", publish_odometry_without_downsample, false);
  nh.param<bool>("publish/path_en",path_en, true);
  nh.param<bool>("publish/scan_publish_en",scan_pub_en,1);
  nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en,1);
  nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);

  nh.param<double>("mapping/lidar_time_inte",lidar_time_inte,0.1);
//   nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);
  nh.param<bool>("mapping/dyn_filter",dyn_filter,1);
  nh.param<double>("mapping/dyn_filter_resolution",dyn_filter_resolution,0.1);
  // nh.param<double>("gnss/odo_weight",p_gnss->odo_weight, 0.1);
  nh.param<double>("gnss/gnss_ekf_noise",gnss_ekf_noise,0.01);
  nh.param<vector<double>>("gnss/gnss_extrinsic_T", extrinT_gnss, vector<double>());
  nh.param<vector<double>>("gnss/gnss_extrinsic_R", extrinR_gnss, vector<double>());

  nh.param<float>("mapping/ivox_grid_resolution", ivox_options_.resolution_, 0.2);
  nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
    p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
    nh.param<bool>("gnss/gnss_enable", GNSS_ENABLE, false);
    cout << "gnss enable:" << GNSS_ENABLE << endl;
    if (GNSS_ENABLE)
    {
        nh.param<double>("gnss/psr_dopp_weight",p_gnss->relative_sqrt_info, 10);
        nh.param<double>("gnss/cp_weight",p_gnss->cp_weight, 0.1);
        nh.param<bool>("gnss/outlier_rejection", p_gnss->p_assign->outlier_rej, false);
        nh.param<string>("gnss/gnss_ephem_topic",gnss_ephem_topic,"/ublox_driver/ephem");
        nh.param<string>("gnss/gnss_glo_ephem_topic",gnss_glo_ephem_topic,"/ublox_driver/glo_ephem");
        nh.param<string>("gnss/gnss_meas_topic",gnss_meas_topic,"/ublox_driver/range_meas");
        nh.param<string>("gnss/ephem_file_name",ephem_fname,"BRDM00DLR_S_20221870000_01D_MN.rnx");
        nh.param<string>("gnss/gt_file_name",gt_fname,"UrbanNav_TST_GT_raw.txt");
        nh.param<string>("nmea/ppp_file_name",ppp_fname,"TST.pos");
        nh.param<string>("gnss/gnss_iono_params_topic",gnss_iono_params_topic,"/ublox_driver/iono_params");
        nh.param<string>("gnss/rtk_pvt_topic",rtk_pvt_topic,"/ublox_driver/receiver_pvt");
        nh.param<string>("gnss/rtk_lla_topic",rtk_lla_topic,"/ublox_driver/receiver_lla");
        nh.param<string>("gnss/gnss_tp_info_topic",gnss_tp_info_topic,"/ublox_driver/time_pulse_info");
        nh.param<vector<double>>("gnss/gnss_iono_default_parameters",default_gnss_iono_params,vector<double>());
        p_gnss->gravity_init << VEC_FROM_ARRAY(gravity);
        nh.param<bool>("gnss/gnss_local_online_sync",gnss_local_online_sync,1);
        if (gnss_local_online_sync)
        {
            nh.param<string>("gnss/local_trigger_info_topic",local_trigger_info_topic,"/external_trigger");
        }
        else
        {
            nh.param<double>("gnss/gnss_local_time_diff",gnss_local_time_diff, 18.0);
            time_diff_gnss_local = gnss_local_time_diff;
        }
        nh.param<double>("gnss/gnss_elevation_thres",p_gnss->p_assign->gnss_elevation_threshold, 30.0);
        nh.param<double>("gnss/prior_noise",p_gnss->p_assign->prior_noise, 0.010);
        nh.param<double>("gnss/marg_noise",p_gnss->p_assign->marg_noise, 0.010);
        nh.param<double>("gnss/b_acc_noise",p_gnss->pre_integration->acc_w, 0.10);
        nh.param<double>("gnss/b_omg_noise",p_gnss->pre_integration->gyr_w, 0.10);
        nh.param<double>("gnss/acc_noise",p_gnss->pre_integration->acc_n, 0.10);
        nh.param<double>("gnss/omg_noise",p_gnss->pre_integration->gyr_n, 0.10);
        nh.param<double>("gnss/ddt_noise",p_gnss->p_assign->ddt_noise, 0.10);
        nh.param<double>("gnss/dt_noise",p_gnss->p_assign->dt_noise, 0.10);
        nh.param<double>("gnss/psr_dopp_noise",p_gnss->p_assign->psr_dopp_noise,0.1);
        nh.param<double>("gnss/odo_noise",p_gnss->p_assign->odo_noise,0.1);
        nh.param<double>("gnss/grav_noise",p_gnss->p_assign->grav_noise,0.1);
        nh.param<double>("gnss/cp_noise",p_gnss->p_assign->cp_noise,0.1);
        nh.param<double>("gnss/gnss_psr_std_thres",p_gnss->p_assign->gnss_psr_std_threshold, 2.0);
        nh.param<double>("gnss/gnss_dopp_std_thres",p_gnss->p_assign->gnss_dopp_std_threshold, 2.0);
        nh.param<double>("gnss/gnss_cp_std_thres",p_gnss->p_assign->gnss_cp_std_threshold, 2.0);
        p_gnss->p_assign->gnss_cp_std_threshold /= 0.004;
        nh.param<double>("gnss/gnss_cp_time_thres",p_gnss->gnss_cp_time_threshold, 2.0);
        nh.param<int>("gnss/gtsam_variable_thres",p_gnss->delete_thred, 200);
        nh.param<int>("gnss/gtsam_marg_variable_thres",p_gnss->p_assign->marg_thred, 1);
        nh.param<double>("gnss/outlier_thres",p_gnss->p_assign->outlier_thres, 0.1);
        nh.param<double>("gnss/outlier_thres_init",p_gnss->p_assign->outlier_thres_init, 0.1);
        nh.param<double>("gnss/gnss_sample_period",p_gnss->gnss_sample_period, 0.1);
        nh.param<bool>("gnss/nolidar",nolidar, false); // not ready yet. only for information. when this value is true, ligo becomes a system fusing only IMU and GNSS
        nh.param<bool>("gnss/ephem_from_rinex",p_gnss->p_assign->ephem_from_rinex, false);
        nh.param<bool>("gnss/obs_from_rinex",p_gnss->p_assign->obs_from_rinex, false);
        nh.param<bool>("gnss/pvt_is_gt",p_gnss->p_assign->pvt_is_gt, false);
        nh.param<int>("gnss/window_size",p_gnss->wind_size, 2);
        p_gnss->p_assign->initNoises();
    }
    else
    {
        nh.param<string>("gnss/rtk_pvt_topic",rtk_pvt_topic,"/ublox_driver/receiver_pvt");
    }
    nh.param<bool>("nmea/nmea_enable", NMEA_ENABLE, false);
    cout << "nmea enable:" << NMEA_ENABLE << endl;
    if (NMEA_ENABLE)
    {
        nh.param<bool>("gnss/outlier_rejection", p_nmea->p_assign->outlier_rej, false);
        nh.param<double>("nmea/nmea_weight",p_nmea->nmea_weight, 0.1);
        nh.param<string>("nmea/posit_odo_topic",nmea_meas_topic,"/mavros/local_position/odom");
        p_nmea->gravity_init << VEC_FROM_ARRAY(gravity);
        nh.param<double>("nmea/nmea_local_time_diff",time_diff_nmea_local, 0.0);
        nh.param<double>("gnss/prior_noise",p_nmea->p_assign->prior_noise, 0.010);
        nh.param<double>("gnss/marg_noise",p_nmea->p_assign->marg_noise, 0.010);
        nh.param<double>("gnss/b_acc_noise",p_nmea->pre_integration->acc_w, 0.10);
        nh.param<double>("gnss/b_omg_noise",p_nmea->pre_integration->gyr_w, 0.10);
        nh.param<double>("gnss/acc_noise",p_nmea->pre_integration->acc_n, 0.10);
        nh.param<double>("gnss/omg_noise",p_nmea->pre_integration->gyr_n, 0.10);
        nh.param<double>("nmea/rot_noise",p_nmea->p_assign->rot_noise,1.0);
        nh.param<double>("nmea/vel_noise",p_nmea->p_assign->vel_noise,1.0);
        nh.param<double>("gnss/odo_noise",p_nmea->p_assign->odo_noise,0.1);
        nh.param<double>("gnss/grav_noise",p_nmea->p_assign->grav_noise,0.1);
        nh.param<double>("nmea/pos_noise",p_nmea->p_assign->pos_noise,0.1);
        nh.param<int>("gnss/gtsam_variable_thres",p_nmea->delete_thred, 200);
        nh.param<int>("gnss/gtsam_marg_variable_thres",p_nmea->p_assign->marg_thred, 1);
        nh.param<double>("gnss/outlier_thres",p_nmea->p_assign->outlier_thres, 0.1);
        nh.param<double>("gnss/outlier_thres_init",p_nmea->p_assign->outlier_thres_init, 0.1);
        nh.param<double>("gnss/gnss_sample_period",p_nmea->nmea_sample_period, 0.1);
        nh.param<double>("nmea/ppp_std_thres",p_nmea->p_assign->ppp_std_threshold, 20.0);
        nh.param<bool>("gnss/nolidar",nolidar, false);
        nh.param<int>("gnss/window_size",p_nmea->wind_size, 2);
        p_nmea->p_assign->initNoises();
    }
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    if (GNSS_ENABLE)
    {
        fout_rtk.open(DEBUG_FILE_DIR("pos_rtk.txt"),ios::out);
        fout_rtk.setf(ios::fixed, ios::floatfield);
        fout_rtk.precision(6);
        fout_global.open(DEBUG_FILE_DIR("pos_est.txt"),ios::out);
        fout_global.setf(ios::fixed, ios::floatfield);
        fout_global.precision(6);
        fout_ppp.open(DEBUG_FILE_DIR("pos_ppp.txt"),ios::out);
        fout_ppp.setf(ios::fixed, ios::floatfield);
        fout_ppp.precision(6);
    }
    if (fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void cout_state_to_file(Eigen::Vector3d &pos_lla)
{
    {
        Eigen::Vector3d pos_enu, pos_ecef;
        if (!nolidar)
        {
            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // maybe improper.normalized()
            // Eigen::Vector3d truth_imu;
            // truth_imu << 0.0, 0.0, 0.14; // 0.0, 0.02, -0.43; // -0.16126, 0.35852, -0.30799; // deg // 
            // Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Matrix3d enu_rot = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            Eigen::Vector3d anc_cur = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            pos_enu = p_gnss->local2enu(enu_rot, anc_cur, pos_r);
            pos_ecef = enu_rot * pos_r + anc_cur;
            local_poses.push_back(anc_cur);
            local_rots.push_back(enu_rot);
            pos_lla = ecef2geo(pos_ecef);
        }
        else
        {
            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // .normalized()
            pos_enu = p_gnss->local2enu(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), pos_r);
            pos_lla = ecef2geo(pos_r);
        }
        // local_poses.push_back(kf_output.x_.pos);
        // local_rots.push_back(kf_output.x_.rot);
        est_poses.push_back(pos_enu);
        time_frame.push_back(time_predict_last_const);
    }
}

void cout_state_to_file_nmea()
{
    {
        Eigen::Vector3d pos_enu;
        if (!nolidar)
        {
            Eigen::Vector3d truth_imu;
            truth_imu << 0.0, 0.0, 0.14; // 0.0, 0.02, -0.43; // 
            // Eigen::Vector3d pos_r = kf_output.x_.rot * p_nmea->Tex_imu_r + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Matrix3d enu_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            Eigen::Vector3d anc_cur = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            pos_enu = enu_rot * pos_r + anc_cur;
        }
        else
        {
            pos_enu = kf_output.x_.rot * p_nmea->Tex_imu_r + kf_output.x_.pos; // .normalized()
        }
        local_poses.push_back(kf_output.x_.pos);
        local_rots.push_back(kf_output.x_.rot);
        est_poses.push_back(pos_enu);
        time_frame.push_back(time_predict_last_const);
    }
}

void reset_cov_output(Eigen::Matrix<double, 24, 24> & P_init_output)
{
    P_init_output = MD(24, 24)::Identity() * 0.01;
    P_init_output.block<3, 3>(15, 15) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(18, 18) = MD(6,6)::Identity() * 0.001;
}
