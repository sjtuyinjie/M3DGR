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

#include "GNSS_Processing_fg.h"

GNSSProcess::GNSSProcess()
    : diff_t_gnss_local(0.0)
{
  Reset();
  // initNoises();
}

GNSSProcess::~GNSSProcess() {}

void GNSSProcess::Reset() 
{
  ROS_WARN("Reset GNSSProcess");
  std::map<sat_first, std::map<uint32_t, double[6]>> empty_map_c;
  sat2cp.swap(empty_map_c);
  // sat2time_index.swap(empty_map_i);
  // sat2ephem.swap(empty_map_e);
  for (size_t i = 0; i < WINDOW_SIZE+1; i++)
  {
    std::vector<ObsPtr> empty_vec_o;
    std::vector<EphemBasePtr> empty_vec_e;
    gnss_meas_buf[i].swap(empty_vec_o);
    gnss_ephem_buf[i].swap(empty_vec_e);
  }
  p_assign->change_ext = 1;
  std::map<uint32_t, uint32_t> empty_map_t;
  std::map<uint32_t, double> empty_map_st;
  p_assign->sat_track_status.swap(empty_map_t);
  p_assign->sat_track_time.swap(empty_map_st);
  p_assign->sat_track_last_time.swap(empty_map_st);
  p_assign->hatch_filter_meas.swap(empty_map_st);
  p_assign->last_cp_meas.swap(empty_map_st);
  p_assign->gtSAMgraph.resize(0); 
  p_assign->initialEstimate.clear();
  p_assign->isamCurrentEstimate.clear();
  p_assign->sum_d = 0;
  p_assign->sum_d2 = 0;
  // p_assign->hatch_filter_meas = 0;
  // p_assign->last_cp = 0;
  // index_delete = 0;
  frame_delete = 0;
  p_assign->factor_id_frame.clear();
  id_accumulate = 0;
  frame_num = 0;
  last_gnss_time = 0.0;
  first_gnss_time = 0.0;
  frame_count = 0;
  invalid_lidar = false;
  Rot_gnss_init.setIdentity();
  p_assign->process_feat_num = 0;
  gnss_ready = false;
  {
    pre_integration->repropagate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 5; // may matter? improtant!
  p_assign->isam = gtsam::ISAM2(parameters);
}

void GNSSProcess::inputIonoParams(double ts, const std::vector<double> &iono_params) 
{
  if (iono_params.size() != 8)    return;

  // update ionosphere parameters
  std::vector<double> empty_vec_d;
  p_assign->latest_gnss_iono_params.swap(empty_vec_d);
  std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(p_assign->latest_gnss_iono_params));
}

void GNSSProcess::inputpvt(double ts, double lat, double lon, double alt, int float_sol, int diff_sol) 
{
  Eigen::Vector3d lla;
  lla << lat, lon, alt;
  if (pvt_time.empty())
  {
    first_lla_pvt = lla;
    first_xyz_ecef_pvt = geo2ecef(lla);
    // first_lla_pvt << 22.609671,114.017229,98.401000;
    // first_xyz_ecef_pvt << -2397684.725162,5380932.949883,2436910.600325;
    printf("first ecef xyz:%f,%f,%f\n",first_xyz_ecef_pvt(0),first_xyz_ecef_pvt(1),first_xyz_ecef_pvt(2));
    printf("first lla:%f,%f,%f\n",first_lla_pvt(0),first_lla_pvt(1),first_lla_pvt(2));
  }
  Eigen::Vector3d xyz_ecef = geo2ecef(lla);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_pvt, xyz_ecef - first_xyz_ecef_pvt);
  pvt_time.push_back(ts);
  pvt_holder.push_back(xyz_enu);
  diff_holder.push_back(diff_sol);
  float_holder.push_back(float_sol);
}

void GNSSProcess::inputlla(double ts, double lat, double lon, double alt) // 
{
  Eigen::Vector3d lla;
  lla << lat, lon, alt;
  if (lla_time.empty())
  {
    first_lla_lla = lla;
    first_xyz_ecef_lla = geo2ecef(lla);
  }
  Eigen::Vector3d xyz_ecef = geo2ecef(lla);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_lla, xyz_ecef - first_xyz_ecef_lla);
  lla_time.push_back(ts);
  lla_holder.push_back(xyz_enu);
}

Eigen::Vector3d GNSSProcess::local2enu(Eigen::Matrix3d R_enu_local_, Eigen::Vector3d anc, Eigen::Vector3d &pos)
{
  Eigen::Vector3d enu_pos;
  if (!nolidar)
  {
    enu_pos = R_enu_local_ * pos; // - anc_local); // 

    // Eigen::Matrix3d R_ecef_enu_ = ecef2rotation(anc);
    // Eigen::Vector3d ecef_pos_ = anc + R_ecef_enu_ * enu_pos;
    Eigen::Vector3d ecef_pos_ = anc + enu_pos;
    // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
    enu_pos = ecef2enu(first_lla_pvt, ecef_pos_ - first_xyz_ecef_pvt);
  }
  else
  {
    Eigen::Vector3d pos_r = pos;
    // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
    enu_pos = ecef2enu(first_lla_pvt, pos_r - first_xyz_ecef_pvt);
  }
  return enu_pos;
}

void GNSSProcess::inputGNSSTimeDiff(const double t_diff) // 
{
    diff_t_gnss_local = t_diff;
}

void GNSSProcess::processGNSS(const std::vector<ObsPtr> &gnss_meas, state_output &state)
{
  std::vector<double>().swap(psr_meas_hatch_filter);
  std::vector<ObsPtr> valid_meas;
  std::vector<EphemBasePtr> valid_ephems;
  valid_ephems.clear();
  valid_meas.clear();
  if (gnss_meas.empty())  
  {
    if (gnss_ready)
    {
      std::vector<ObsPtr> empty_vec_o;
      std::vector<EphemBasePtr> empty_vec_e;
      gnss_meas_buf[0].swap(empty_vec_o);
      gnss_ephem_buf[0].swap(empty_vec_e);
    }
    return;
  }

  if (gnss_ready)
  {
    Eigen::Vector3d pos_gnss = state.pos + state.rot * Tex_imu_r; // .normalized()
    updateGNSSStatistics(pos_gnss);
  }
  p_assign->processGNSSBase(gnss_meas, psr_meas_hatch_filter, valid_meas, valid_ephems, gnss_ready, ecef_pos, last_gnss_time);
  
  if (!gnss_ready)
  {
    if (valid_meas.size() < 4) return; // right or not? valid_meas.empty() || 
    {
      rot_window[frame_count] = state.rot; //.normalized().toRotationMatrix();
      pos_window[frame_count] = state.pos + state.rot * Tex_imu_r; // .normalized()
      Eigen::Matrix3d omg_skew;
      omg_skew << SKEW_SYM_MATRX(state.omg);
      vel_window[frame_count] = state.vel + state.rot * omg_skew * Tex_imu_r; // .normalized().toRotationMatrix()
      // vel_window[frame_count] = state.vel;
    }
    gnss_meas_buf[frame_count] = valid_meas; 
    gnss_ephem_buf[frame_count] = valid_ephems;
    frame_count ++;
    gnss_ready = GNSSLIAlign();
    if (gnss_ready)
    {
      ROS_INFO("GNSS Initialization is done");
      state_const_ = state;
      // state_const_last = state;
    }
  }
  else
  {  
    gnss_meas_buf[0] = valid_meas; 
    gnss_ephem_buf[0] = valid_ephems;
  }
}

void GNSSProcess::runISAM2opt(void) //
{
  gtsam::FactorIndices delete_factor;
  gtsam::FactorIndices().swap(delete_factor);

  if (gnss_ready)
  {
    bool delete_happen = false;
    if (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 4000)
    {
      delete_happen = true;
    while (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 3000)
    { 
      if (!p_assign->factor_id_frame.empty())       
      {
        // if (frame_delete > 0)
        {
        for (size_t i = 0; i < p_assign->factor_id_frame[0].size(); i++)
        {
          {
            delete_factor.push_back(p_assign->factor_id_frame[0][i]);
          }
        }
        // index_delete += p_assign->factor_id_frame[0].size();
        }
      
        p_assign->factor_id_frame.pop_front();
        frame_delete ++;
      }
      if (p_assign->factor_id_frame.empty()) break;
    }
    }

    if (delete_happen)
    {
      p_assign->delete_variables(nolidar, frame_delete, frame_num, id_accumulate, delete_factor);
    }
    else
    {
      p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
      p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
      p_assign->initialEstimate.clear();
      p_assign->isam.update();
    }
  }
  else
  {
    p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
    p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
    p_assign->initialEstimate.clear();
    p_assign->isam.update();
  }
  p_assign->isamCurrentEstimate = p_assign->isam.calculateEstimate();
  
  if (nolidar) // || invalid_lidar)
  {
    pre_integration->repropagate(p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6),
                                p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9));
  }
  std::map<sat_first, std::map<uint32_t, double[6]>>::iterator it_old;
  if (!sat2cp.empty())
  {
    it_old = sat2cp.begin();
    while (it_old->first.frame_num < frame_delete)
    {
      std::map<uint32_t, double[6]>().swap(it_old->second);
      size_t del_size = sat2cp.erase(it_old->first);
      if (sat2cp.empty()) break;
      it_old = sat2cp.begin();
    }
  }
}

bool GNSSProcess::GNSSLIAlign()
{
  if (frame_count < wind_size + 1) return false;
  
  for (uint32_t i = 0; i < wind_size; i++)
  {
    if (time2sec(gnss_meas_buf[i+1][0]->time) - time2sec(gnss_meas_buf[i][0]->time) > 15 * gnss_sample_period) // need IMU to prop
    {
      // if (frame_count == wind_size + 1)
      // {
        for (uint32_t j = i+1; j < wind_size+1; ++j)
        {
          gnss_meas_buf[j-i-1] = gnss_meas_buf[j];
          gnss_ephem_buf[j-i-1] = gnss_ephem_buf[j];
          rot_window[j-i-1] = rot_window[j];
          pos_window[j-i-1] = pos_window[j];
          vel_window[j-i-1] = vel_window[j];
        }
        frame_count -= i+1;
        for (uint32_t j = frame_count; j < wind_size+1; ++j) // wind_size-i
        {
          std::vector<ObsPtr> empty_vec_o;
          std::vector<EphemBasePtr> empty_vec_e;
          gnss_meas_buf[j].swap(empty_vec_o);
          gnss_ephem_buf[j].swap(empty_vec_e); 
        }             
      // }
      return false;
    }
  }

  std::vector<std::vector<ObsPtr>> curr_gnss_meas_buf;
  std::vector<std::vector<EphemBasePtr>> curr_gnss_ephem_buf;
  if ((pos_window[wind_size] - pos_window[0]).norm() < 10.0) // && pos_window[0].norm() < 10.0) 
  {
    for (uint32_t i = 0; i < (wind_size+1); ++i)
    {
        curr_gnss_meas_buf.push_back(gnss_meas_buf[i]);
        curr_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
    }

    GNSSLIInitializer gnss_li_initializer(curr_gnss_meas_buf, curr_gnss_ephem_buf, p_assign->latest_gnss_iono_params);

    // 1. get a rough global location
    Eigen::Matrix<double, 7, 1> rough_xyzt;
    // Eigen::Matrix<double, 3, 1> rough_xyz;
    rough_xyzt.setZero();
    // rough_xyz.setZero();
    if (!gnss_li_initializer.coarse_localization(rough_xyzt))
    {
        std::cerr << "Fail to obtain a coarse location.\n";
        for (uint32_t i = 0; i < (wind_size); ++i)
        {
          gnss_meas_buf[i] = gnss_meas_buf[i+1];
          gnss_ephem_buf[i] = gnss_ephem_buf[i+1];
          rot_window[i] = rot_window[i+1];
          pos_window[i] = pos_window[i+1];
          vel_window[i] = vel_window[i+1];
        }
        frame_count = wind_size;
        std::vector<ObsPtr> empty_vec_o;
        std::vector<EphemBasePtr> empty_vec_e;
        gnss_meas_buf[frame_count].swap(empty_vec_o);
        gnss_ephem_buf[frame_count].swap(empty_vec_e);
        return false;
    }
    {
      size_t dt_idx = -1;
      for (uint32_t k_ = 0; k_ < 4; k_++)
      {
        if (fabs(rough_xyzt(3+k_)) > 0)
        {
          dt_idx = k_;
          break;
        }
      }
      if (dt_idx < 0)
      {
        std::cerr << "Fail to quick init anchor point.\n";
        for (uint32_t i = 0; i < (wind_size); ++i)
        {
          gnss_meas_buf[i] = gnss_meas_buf[i+1]; // change the strategy
          gnss_ephem_buf[i] = gnss_ephem_buf[i+1];

          rot_window[i] = rot_window[i+1];
          pos_window[i] = pos_window[i+1];
          vel_window[i] = vel_window[i+1];
        }
        frame_count = wind_size;
        std::vector<ObsPtr> empty_vec_o;
        std::vector<EphemBasePtr> empty_vec_e;
        gnss_meas_buf[frame_count].swap(empty_vec_o);
        gnss_ephem_buf[frame_count].swap(empty_vec_e);
        return false;
      }
      anc_local = pos_window[0] - rot_window[0] * Tex_imu_r; // [wind_size]; // ? 
      yaw_enu_local = 0.0; // -2418165.665753, 5385967.410215, 2405315.115443; // 
      para_rcv_ddt[0] = 0.0; // 128.0;
      // rough_xyz = rough_xyzt.head<3>();
      // if (anc_local.norm() > 100)
      // {
      //   std::vector<Eigen::Vector3d> local_vs;
      //   for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
      //       local_vs.push_back(vel_window[i]); // values at gnss measurement
      //   if (gnss_li_initializer.yaw_alignment(local_vs, rough_xyz, yaw_enu_local, para_rcv_ddt[0]))
      //   {
      //     printf("yaw_enu_local:%f\n",yaw_enu_local);
      //   }
      //   else
      //   {
      //     yaw_enu_local = 0.0;
      //     para_rcv_ddt[0] = 0.0;
      //     rough_xyz = rough_xyzt.head<3>();
      //   }
      // }
      anc_ecef = rough_xyzt.head<3>(); // - anc_local; << -2418181.50, 5385962.29, 2405305.18;
      R_ecef_enu = ecef2rotation(anc_ecef); // * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).matrix(); // * yawAngle; // * pitchAngle * rollAngle; //<< 0.772234, 0.501306, -0.390316,
                      // 0.047633, 0.566933, 0.822386,
                      // 0.633550, -0.653666, 0.413926; //
      anc_ecef -= R_ecef_enu * anc_local; // anc_local too large: need initialize yaw
      // R_ecef_enu = ecef2rotation(anc_ecef); // * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).matrix(); // * yawAngle; // * pitchAngle * rollAngle; //<< 0.772234, 0.501306, -0.390316,
      para_rcv_dt[4*wind_size] = rough_xyzt(3+dt_idx);
    }
  }
  else
  {
    for (uint32_t i = 0; i < (wind_size+1); ++i)
    {
      // for (uint32_t j = 0; j < gnss_meas_buf[i].size(); j++)
      // {
      //   int freq_idx_ = -1;
      //   double freq = L1_freq(gnss_meas_buf[i][j], &freq_idx_); // L1_freq NEEDED
      //   // if (freq_idx_ < 0)   continue; 
      // }
        curr_gnss_meas_buf.push_back(gnss_meas_buf[i]);
        curr_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
      // }

      GNSSLIInitializer gnss_li_initializer(curr_gnss_meas_buf, curr_gnss_ephem_buf, p_assign->latest_gnss_iono_params);

      // 1. get a rough global location
      Eigen::Matrix<double, 7, 1> rough_xyzt;
      Eigen::Matrix<double, 3, 1> rough_xyz;
      rough_xyzt.setZero();
      rough_xyz.setZero();
      if (!gnss_li_initializer.coarse_localization(rough_xyzt))
      {
        for (uint32_t j = i; j < wind_size; ++j)
        {
          gnss_meas_buf[j] = gnss_meas_buf[j+1];
          gnss_ephem_buf[j] = gnss_ephem_buf[j+1];
          rot_window[j] = rot_window[j+1];
          pos_window[j] = pos_window[j+1];
          vel_window[j] = vel_window[j+1];
        }
        frame_count -= 1;
        for (uint32_t j = frame_count; j < wind_size+1; ++j) // wind_size-i
        {
          std::vector<ObsPtr> empty_vec_o;
          std::vector<EphemBasePtr> empty_vec_e;
          gnss_meas_buf[j].swap(empty_vec_o);
          gnss_ephem_buf[j].swap(empty_vec_e); 
        }          
        return false;   
      }
      size_t dt_idx = -1;
      for (uint32_t k_ = 0; k_ < 4; k_++)
      {
        if (fabs(rough_xyzt(3+k_)) > 0)
        {
          dt_idx = k_;
          break;
        }
      }
      // if (num_dt_fail == 4)
      if (dt_idx < 0)
      {
        std::cerr << "Fail to quick init anchor point.\n";
        for (uint32_t j = i; j < wind_size; ++j)
        {
          gnss_meas_buf[j] = gnss_meas_buf[j+1]; // change the strategy
          gnss_ephem_buf[j] = gnss_ephem_buf[j+1];

          rot_window[j] = rot_window[j+1];
          pos_window[j] = pos_window[j+1];
          vel_window[j] = vel_window[j+1];
        }
        frame_count = wind_size;
        std::vector<ObsPtr> empty_vec_o;
        std::vector<EphemBasePtr> empty_vec_e;
        gnss_meas_buf[frame_count].swap(empty_vec_o);
        gnss_ephem_buf[frame_count].swap(empty_vec_e);
        return false;
      }
      pos_ecef_window[i] = rough_xyzt.head<3>();
      para_rcv_dt[4*i] = rough_xyzt(3+dt_idx);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(wind_size+1, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>(wind_size+1, 1));

    // Fill in the CloudIn data
    // for (auto& point : *cloud_in)
    for (size_t i = 0; i < wind_size + 1; i++)
    {
      cloud_in->points[i].x = pos_window[i](0);
      cloud_in->points[i].y = pos_window[i](1);
      cloud_in->points[i].z = pos_window[i](2);
      cloud_out->points[i].x = pos_ecef_window[i](0);
      cloud_out->points[i].y = pos_ecef_window[i](1);
      cloud_out->points[i].z = pos_ecef_window[i](2);
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "ICP has " << (icp.hasConverged()?"converged":"not converged") << ", score: " << icp.getFitnessScore() << std::endl;
    // std::cout << icp.getFinalTransformation() << std::endl;
    // Eigen::Vector3d pos_trans;
    // Eigen::Matrix3d rot_trans;
    // TrajAlign(local_traj, enu_traj, pos_trans, rot_trans);
    if (!icp.hasConverged())
    {
        std::cerr << "Fail to obtain a coarse location.\n";
        for (uint32_t i = 0; i < (wind_size); ++i)
        {
          gnss_meas_buf[i] = gnss_meas_buf[i+1];
          gnss_ephem_buf[i] = gnss_ephem_buf[i+1];
          rot_window[i] = rot_window[i+1];
          pos_window[i] = pos_window[i+1];
          vel_window[i] = vel_window[i+1];
          pos_ecef_window[i] = pos_ecef_window[i+1];
          para_rcv_dt[4*i] = para_rcv_dt[4*(i+1)];
        }
        frame_count = wind_size;
        std::vector<ObsPtr> empty_vec_o;
        std::vector<EphemBasePtr> empty_vec_e;
        gnss_meas_buf[frame_count].swap(empty_vec_o);
        gnss_ephem_buf[frame_count].swap(empty_vec_e);
        return false;
    }
    
    Eigen::Matrix4d sim_trans = icp.getFinalTransformation().cast<double>();
    // Eigen::Vector3d pos_nmea(nmea_meas_[0]->pose.pose.position.x, nmea_meas_[0]->pose.pose.position.y, nmea_meas_[0]->pose.pose.position.z);
    // Eigen::Vector3d pos_nmea(nmea_meas_[0]->pose.pose.position.x, nmea_meas_[0]->pose.pose.position.y, nmea_meas_[0]->pose.pose.position.z);
    anc_ecef = sim_trans.block<3, 1>(0, 3); // icp.getFinalTransformation().template block<3, 1>(0, 3); // pos_trans; // pos_nmea - pos_window[0]; //
    
    anc_local = Eigen::Vector3d::Zero(); // pos_window[0]; // [WINDOW_SIZE]; // ? 
    yaw_enu_local = 0.0; // -2418165.665753, 5385967.410215, 2405315.115443; // 
    para_rcv_ddt[0] = 0.0; // 128.0;
      
    R_ecef_enu = sim_trans.block<3, 3>(0, 0); // ecef2rotation(anc_ecef) * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).matrix(); // * yawAngle; // * pitchAngle * rollAngle; //<< 0.772234, 0.501306, -0.390316,
                      // 0.047633, 0.566933, 0.822386, 
                      // 0.633550, -0.653666, 0.413926; //
  }
    SetInit();
    frame_num = 1; // frame_count;
    last_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);
    first_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);
    // printf("first gnss time: %f", first_gnss_time);
  // }

  for (uint32_t k_ = 1; k_ < wind_size+1; k_++)
  {
    std::vector<ObsPtr>().swap(gnss_meas_buf[k_]);
    std::vector<EphemBasePtr>().swap(gnss_ephem_buf[k_]);
  }
  runISAM2opt();
  return true;
}

void GNSSProcess::updateGNSSStatistics(Eigen::Vector3d &pos) // delete
{
  if (!nolidar)
  {
    Eigen::Vector3d anc_cur;
    Eigen::Matrix3d R_enu_local_;
    // if (frame_num == 1)
    // {
    //   anc_cur = anc_ecef;
    //   R_enu_local_ = R_ecef_enu * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()) * Rot_gnss_init;
    // }
    // else
    {
      anc_cur = p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
      R_enu_local_ = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
    }
    Eigen::Vector3d enu_pos = R_enu_local_ * pos; // - anc_local);
    // R_ecef_enu = ecef2rotation(anc_cur);
    // ecef_pos = anc_cur + R_ecef_enu * enu_pos;
    ecef_pos = anc_cur + enu_pos;
  }
  else
  {
    ecef_pos = pos;
  }
}

void GNSSProcess::GnssPsrDoppMeas(const ObsPtr &obs_, const EphemBasePtr &ephem_) 
{
  freq = L1_freq(obs_, &freq_idx);
  LOG_IF(FATAL, freq < 0) << "No L1 observation found.";

  uint32_t sys = satsys(obs_->sat, NULL);
  double tof = obs_->psr[freq_idx] / LIGHT_SPEED;
  gtime_t sv_tx = time_add(obs_->time, -tof);

  if (sys == SYS_GLO)
  {
      GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephem_);
      svdt = geph2svdt(sv_tx, glo_ephem);
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
      sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
      tgd = 0.0;
      pr_uura = 2.0 * (obs_->psr_std[freq_idx]/0.16);
      dp_uura = 2.0 * (obs_->dopp_std[freq_idx]/0.256);
  }
  else
  {
      EphemPtr eph = std::dynamic_pointer_cast<Ephem>(ephem_);
      svdt = eph2svdt(sv_tx, eph); // used in eva
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = eph2pos(sv_tx, eph, &svdt); // used in eva
      sv_vel = eph2vel(sv_tx, eph, &svddt); // used in eva
      tgd = eph->tgd[0];
      if (sys == SYS_GAL)
      {
          pr_uura = (eph->ura - 2.0) * (obs_->psr_std[freq_idx]/0.16);
          dp_uura = (eph->ura - 2.0) * (obs_->dopp_std[freq_idx]/0.256);
      }
      else
      {
          pr_uura = (eph->ura - 1.0) * (obs_->psr_std[freq_idx]/0.16);
          dp_uura = (eph->ura - 1.0) * (obs_->dopp_std[freq_idx]/0.256);
      }
  }
  LOG_IF(FATAL, pr_uura <= 0) << "pr_uura is " << pr_uura; // get those parameters mainly, both used in eva
  LOG_IF(FATAL, dp_uura <= 0) << "dp_uura is " << dp_uura;
  // relative_sqrt_info = 10;
}

void GNSSProcess::SvPosCals(const ObsPtr &obs_, const EphemBasePtr &ephem_) 
{
  freq = L1_freq(obs_, &freq_idx);
  LOG_IF(FATAL, freq < 0) << "No L1 observation found.";

  uint32_t sys = satsys(obs_->sat, NULL);
  double tof = obs_->psr[freq_idx] / LIGHT_SPEED;
  gtime_t sv_tx = time_add(obs_->time, -tof);

  if (sys == SYS_GLO)
  {
      GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephem_);
      svdt = geph2svdt(sv_tx, glo_ephem);
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
      sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
  }
  else
  {
      EphemPtr eph = std::dynamic_pointer_cast<Ephem>(ephem_);
      svdt = eph2svdt(sv_tx, eph); // used in eva
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = eph2pos(sv_tx, eph, &svdt); // used in eva
      sv_vel = eph2vel(sv_tx, eph, &svddt); // used in eva
  }
}

bool GNSSProcess::Evaluate(state_output &state)
{
  if (gnss_meas_buf[0].empty()) // ||gnss_meas_buf[0].size() < 4) //  
  {
    // cout << "no valid gnss" << endl;
    return false;
  }

  double time_current = time2sec(gnss_meas_buf[0][0]->time);
  double delta_t = time_current - last_gnss_time;

  gtsam::Rot3 rel_rot; // = gtsam::Rot3(pre_integration->delta_q);
  gtsam::Point3 rel_pos, pos, ba, bg, acc, omg;
  gtsam::Vector3 rel_vel, vel; 
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  if (!nolidar) // && !invalid_lidar)
  {
    // Eigen::Matrix3d last_rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(0)).matrix(); // state_const_.rot; // 
    // // cout << "check time period" << pre_integration->sum_dt << ";" << time_current - last_gnss_time <<  endl;
    // Eigen::Vector3d last_pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(0); // state_.pos; // 
    // Eigen::Vector3d last_vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(3); // state_.vel; //
    // Eigen::Vector3d cur_grav = state.rot.transpose() * state.gravity; //  
    rot = state.rot; //.normalized().toRotationMatrix(); last_rot.transpose() *
    // rel_rot = gtsam::Rot3(last_rot.transpose() * state.rot.normalized().toRotationMatrix());
    pos = state.pos; // - anc_local; // last_rot.transpose() * (state.pos - last_pos - last_vel * delta_t - 0.5 * state.gravity * delta_t * delta_t);  - last_pos 
    // (state.pos - last_pos);
    vel = state.vel; // last_rot.transpose() * (state.vel - last_vel - state.gravity * delta_t); // (state.vel - last_vel);  - last_vel
    ba = state.ba;
    bg = state.bg;
    acc = state.acc;
    omg = state.omg;
  }
  else
  {
    ba = state.ba;
    bg = state.bg;
    rel_rot = gtsam::Rot3(pre_integration->delta_q);
    rel_pos = pre_integration->delta_p;
    rel_vel = pre_integration->delta_v; 
  }
  
  if (!nolidar) // && invalid_lidar)
  {
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector_imu;
    init_vel_bias_vector_imu.block<3,1>(0,0) = state.pos; // - anc_local;
    init_vel_bias_vector_imu.block<3,1>(3,0) = state.vel;
    Eigen::Matrix<double, 12, 1> init_others_vector_imu;
    init_others_vector_imu.block<3,1>(0,0) = state.omg;
    init_others_vector_imu.block<3,1>(3,0) = state.acc;
    init_others_vector_imu.block<3,1>(6,0) = state.bg;
    init_others_vector_imu.block<3,1>(9,0) = state.ba;
    p_assign->initialEstimate.insert(A(frame_num), gtsam::Vector6(init_vel_bias_vector_imu));
    p_assign->initialEstimate.insert(G(frame_num), gtsam::Vector3(state.gravity));
    p_assign->initialEstimate.insert(O(frame_num), gtsam::Vector12(init_others_vector_imu));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.rot));  // .normalized().toRotationMatrix()
  }
  else
  {
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = state.pos;
    init_vel_bias_vector.block<3,1>(3,0) = state.vel;
    init_vel_bias_vector.block<3,1>(6,0) = state.ba;
    init_vel_bias_vector.block<3,1>(9,0) = state.bg;
    p_assign->initialEstimate.insert(F(frame_num), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.rot)); // .normalized().toRotationMatrix()
  }              
  // rot_pos = state.rot; //.normalized().toRotationMatrix();
  if (AddFactor(rel_rot, rel_pos, rel_vel, state.gravity, delta_t, time_current, ba, bg, pos, vel, acc, omg, rot))
  {
    frame_num ++;
    runISAM2opt();
    // auto ekfPosNoise = p_assign->isam.marginalCovariance(A(frame_num-1));
    // odo_weight = 60 / (ekfPosNoise(0,0) + ekfPosNoise(1,1) + ekfPosNoise(2,2));
  }
  else
  {
    return false;
  }

  // state.cov.block<3,3>(0, 0) = isam.marginalCovariance(R(frame_num-1));
  // state.cov.block<6,6>(3, 3) = isam.marginalCovariance(F(frame_num-1)).block<6, 6>(0, 0);
  
  if (nolidar)
  {
    state.rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    // state.rot.normalize();
    state.pos = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(0);
    state.vel = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(3);
    state.ba = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6);
    state.bg = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9);
    state.gravity = ecef2rotation(state.pos) * gravity_init;
  }
  else
  {
    state_const_.rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    state_const_.pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(0); // + anc_local;
    state_const_.vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(3);
    state.gravity = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix().transpose() * ecef2rotation(p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0))) * gravity_init;
  }    
  last_gnss_time = time_current;
  std::map<sat_first, std::map<uint32_t, double[6]>>::iterator it_old;
  if (!sat2cp.empty())
  {
    it_old = sat2cp.begin();
    while (time_current - it_old->first.timecur > gnss_cp_time_threshold)
    {
      std::map<uint32_t, double[6]>().swap(it_old->second);
      size_t del_size = sat2cp.erase(it_old->first);
      if (sat2cp.empty()) break;
      it_old = sat2cp.begin();
    }
  }
  return true;
}

bool GNSSProcess::AddFactor(gtsam::Rot3 rel_rot, gtsam::Point3 rel_pos, gtsam::Vector3 rel_vel, Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot)
{
  double rcv_dt[4];
  bool rcv_sys[4];
  rcv_sys[0] = false; rcv_sys[1] = false; rcv_sys[2] = false; rcv_sys[3] = false;
  double rcv_ddt;
  invalid_lidar = false;
  if (!nolidar)
  {
    double weight_lid = 1;
    if (p_assign->process_feat_num < 10) 
    {
      weight_lid = 0;
      invalid_lidar = true;
    }
    else
    {
      if (norm_vec_num < 10)
      {
        invalid_lidar = true;
      }
      weight_lid = 2 * double(norm_vec_num) / double(p_assign->process_feat_num);
    }
    norm_vec_num = 0;
    p_assign->process_feat_num = 0;
    double weight_check1 =  sqrt_lidar(0, 0) < sqrt_lidar(1, 1) ? sqrt_lidar(0, 0) : sqrt_lidar(1, 1);
    weight_check1 = weight_check1 < sqrt_lidar(2, 2) ? weight_check1 : sqrt_lidar(2, 2); 
    double weight_check2 = sqrt_lidar(0, 0) > sqrt_lidar(1, 1) ? sqrt_lidar(0, 0) : sqrt_lidar(1, 1);
    weight_check2 = weight_check2 > sqrt_lidar(2, 2) ? weight_check2 : sqrt_lidar(2, 2); 
    if (weight_check2 / weight_check1 > 3)
    {
      sqrt_lidar *= 0.5 / weight_check1; // .block<3, 3>(0, 0)
    }
    else
    {
      double scale_weight = weight_lid > 0.5 ? weight_lid : 0.5;
      sqrt_lidar *= scale_weight / weight_check1; // .block<3, 3>(0, 0)
    }
  }
  // sqrt_lidar.block<6, 6>(3, 3) *= 2;
// for (size_t j = 3; j < 9; j++)
// {
//   sqrt_lidar(j, j) += 0.50;
// }
  // if ((delta_t > 15 * gnss_sample_period && nolidar) || (delta_t > 15 * gnss_sample_period && invalid_lidar && !nolidar))
  // {
    // Reset();
    // return false;
  // }
  if (nolidar_cur && !nolidar) nolidar_cur = false;
  rcv_ddt = p_assign->isamCurrentEstimate.at<gtsam::Vector1>(C(frame_num-1))[0];
  rcv_dt[0] = p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(frame_num-1))[0] + rcv_ddt * delta_t;
  rcv_dt[1] = p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(frame_num-1))[1] + rcv_ddt * delta_t;
  rcv_dt[2] = p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(frame_num-1))[2] + rcv_ddt * delta_t;
  rcv_dt[3] = p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(frame_num-1))[3] + rcv_ddt * delta_t;

  const std::vector<ObsPtr> &curr_obs = gnss_meas_buf[0];
  const std::vector<EphemBasePtr> &curr_ephem = gnss_ephem_buf[0];

  // find best sat in the current gnss measurements
  std::map<sat_first, std::map<uint32_t, double[6]> >::reverse_iterator it;
  
  std::deque<uint32_t> pair_sat_copy;
  // std::deque<uint32_t>().swap(pair_sat_copy);

  std::deque<double> meas_sats, meas_sats_final;
  // std::deque<double> meas_cov_sats; //, meas_cov_sats_final;
  // std::deque<double> meas_time_sats, meas_time_sats_final;
  std::deque<int> meas_index_sats, meas_index_sats_final;
  std::deque<Eigen::Vector3d> meas_RTex_sats, meas_RTex_sats_final;
  std::deque<Eigen::Vector3d> meas_svpos_sats, meas_svpos_sats_final;
  
  for (uint32_t j = 0; j < curr_obs.size(); j++) //   && j < 10
  {
    std::map<uint32_t, double[6]>::iterator it_old; // t_old_best, 
    double meas;
    // double meas_cov;
    double meas_time;
    int meas_index;
    Eigen::Vector3d meas_svpos;
    Eigen::Vector3d RTex_sats;
    // Eigen::Vector3d best_svpos;
    // if (pair_sat.size() > 0)
    // if (curr_obs[j]->cp_std[freq_idx] < p_assign->gnss_cp_std_threshold)
    {
      // if (j == pair_sat.front())
      // if (curr_obs[j]->cp[freq_idx] > 10)
      {
        bool cp_found = false;
        for (it = sat2cp.rbegin(); it != sat2cp.rend(); it++)
        {
          it_old = it->second.find(curr_obs[j]->sat); // the same satellite
          // it_old_best = it->second.find(curr_obs[best_sat]->sat);
          if (it_old != it->second.end()) // && it_old_best != it->second.end())
          {
            if (it->first.timecur >= p_assign->sat_track_time[curr_obs[j]->sat])
            {
            // if ((time_current - it->first.timecur) / gnss_sample_period <= p_assign->sat_track_status[curr_obs[best_sat]->sat] - p_assign->gnss_track_num_threshold &&
            // if ((time_current - it->first.timecur) / gnss_sample_period <= p_assign->sat_track_status[curr_obs[j]->sat]) //- p_assign->gnss_track_num_threshold)
            if (time_current > p_assign->sat_track_time[curr_obs[j]->sat] && p_assign->sat_track_status[curr_obs[j]->sat] > 0) //- p_assign->gnss_track_num_threshold)
            {
              cp_found = true;
              meas = it_old->second[0]; // - it_old_best->second[1] + it_old->second[1]); it_old_best->second[0] -
              // meas_cov = (it_old->second[2] * it_old->second[2]); // it_old_best->second[2] * it_old_best->second[2] + 
              meas_time = it->first.timecur;
              meas_index = it->first.frame_num;
              RTex_sats << it->first.RTex[0], it->first.RTex[1], it->first.RTex[2];
              meas_svpos << it_old->second[3], it_old->second[4], it_old->second[5];
              // best_svpos << it_old_best->second[3], it_old_best->second[4], it_old_best->second[5];
              break;
            }
            }
          }
        }
      
        if (cp_found)
        {
          meas_sats.push_back(meas);
          // meas_cov_sats.push_back(meas_cov);
          // meas_time_sats.push_back(meas_time);
          meas_index_sats.push_back(meas_index);
          meas_svpos_sats.push_back(meas_svpos);
          meas_RTex_sats.push_back(RTex_sats);
          // meas_svpos_best.push_back(best_svpos);
          pair_sat_copy.push_back(j);
        }
        // pair_sat.pop_front();
      }
    }
  }

  std::map<uint32_t, double[6]> curr_cp_map;
  std::vector<double> meas_cp;
  // std::vector<double> cov_cp;
  std::vector<Eigen::Vector3d> sv_pos_pair, sat_svpos;
  double cov_cp_best, meas_cp_best; //, esti_cp_best, 
  // Eigen::Vector3d sv_pos_best;
  std::vector<size_t> factor_id_cur, sys_idx_cp;
  M3D omg_skew;
  omg_skew << SKEW_SYM_MATRX(omg);
  Eigen::Vector3d hat_omg_T = omg_skew * Tex_imu_r;
  for (uint32_t j = 0; j < curr_obs.size(); j++) //   && j < 10
  {
    bool balance = false;
    if (j > curr_obs.size() / 2)
    {
      balance = true;
    }
    const uint32_t sys = satsys(curr_obs[j]->sat, NULL);
    const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);
    GnssPsrDoppMeas(curr_obs[j], curr_ephem[j]); //, latest_gnss_iono_params);
    freq = L1_freq(curr_obs[j], &freq_idx); // save
    const double wavelength = LIGHT_SPEED / freq; // save
    if (curr_obs[j]->cp_std[freq_idx] < p_assign->gnss_cp_std_threshold)
    {
      if (curr_obs[j]->cp[freq_idx] * wavelength > 100)
      {
        curr_cp_map[curr_obs[j]->sat][0] = curr_obs[j]->cp[freq_idx] * wavelength + svdt * LIGHT_SPEED - tgd * LIGHT_SPEED;
        // curr_cp_map[curr_obs[j]->sat][2] = curr_obs[j]->cp_std[freq_idx] * 0.004;
        curr_cp_map[curr_obs[j]->sat][3] = sv_pos[0];
        curr_cp_map[curr_obs[j]->sat][4] = sv_pos[1];
        curr_cp_map[curr_obs[j]->sat][5] = sv_pos[2];
 
        if (pair_sat_copy.size() > 0)
        {
          for (size_t k = 0; k < pair_sat_copy.size(); k++)
          {
            if (j == pair_sat_copy[k])
            {
              meas_cp.push_back(curr_obs[j]->cp[freq_idx] * wavelength + svdt * LIGHT_SPEED - tgd * LIGHT_SPEED);
              sys_idx_cp.push_back(sys_idx);
              meas_sats_final.push_back(meas_sats[k]);
              // cov_cp.push_back(curr_obs[j]->cp_std[freq_idx] * curr_obs[j]->cp_std[freq_idx] * 0.004 * 0.004);
              // meas_cov_sats_final.push_back(meas_cov_sats[k]);
              sv_pos_pair.push_back(sv_pos);
              meas_svpos_sats_final.push_back(meas_svpos_sats[k]);
              // meas_time_sats_final.push_back(meas_time_sats[k]);
              meas_index_sats_final.push_back(meas_index_sats[k]);
              meas_RTex_sats_final.push_back(meas_RTex_sats[k]);
              break;
              // pair_sat_copy.pop_front();
            }
          }
        }
      }
    }
    /////////////////////////////////
    double values[27];
    values[0] = Tex_imu_r[0]; values[1] = Tex_imu_r[1]; values[2] = Tex_imu_r[2]; //values[3] = anc_local[0]; values[4] = anc_local[1]; values[5] = anc_local[2];
    values[3] = sv_pos[0]; values[4] = sv_pos[1]; values[5] = sv_pos[2]; values[6] = sv_vel[0]; values[7] = sv_vel[1]; values[8] = sv_vel[2];
    values[9] = svdt; values[10] = tgd; values[11] = svddt; values[12] = pr_uura; values[13] = dp_uura; values[14] = relative_sqrt_info; // psr_weight_adjust;
    values[15] = p_assign->latest_gnss_iono_params[0]; values[16] = p_assign->latest_gnss_iono_params[1]; values[17] = p_assign->latest_gnss_iono_params[2]; values[18] = p_assign->latest_gnss_iono_params[3]; 
    values[19] = p_assign->latest_gnss_iono_params[4]; values[20] = p_assign->latest_gnss_iono_params[5]; values[21] = p_assign->latest_gnss_iono_params[6]; values[22] = p_assign->latest_gnss_iono_params[7]; 
    values[23] = time_current; values[24] = freq; values[25] = psr_meas_hatch_filter[j]; values[26] = curr_obs[j]->dopp[freq_idx]; //curr_obs[j]->psr[freq_idx]; 
    rcv_sys[sys_idx] = true;
    if (!nolidar)
    { 
      Eigen::Vector3d RTex = rot * Tex_imu_r;
      values[0] = RTex[0]; values[1] = RTex[1]; values[2] = RTex[2];
      if (frame_num < delete_thred)
      {      
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), balance, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise_init));
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), balance, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise));
      }
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise));
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactor(R(frame_num), A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise));
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorPos(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, rot_pos, hat_omg_T, p_assign->robustpsrdoppNoise));
    }
    else
    {   
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNolidar(R(frame_num), F(frame_num), B(frame_num), C(frame_num), values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise_init)); // not work
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNolidar(R(frame_num), F(frame_num), B(frame_num), C(frame_num), values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise)); // not work
      } 
    }
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;
  }
  sat_first cur_key;
  cur_key.RTex = rot * Tex_imu_r;
  cur_key.timecur = time2sec(curr_obs[0]->time);
  cur_key.frame_num = frame_num;
  sat2cp[cur_key] = curr_cp_map;
  // if (frame_num < delete_thred)
  // {
  //   p_assign->gtSAMgraph.add(ligo::DdtSmoothFactor(C(frame_num-1), C(frame_num), p_assign->ddtNoise_init));
  //   // p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector1>(C(frame_num), gtsam::Vector1(rcv_ddt), p_assign->ddtNoise));
  //   p_assign->gtSAMgraph.add(ligo::DtDdtFactor(B(frame_num-1), B(frame_num), C(frame_num-1), C(frame_num), rcv_sys, delta_t, p_assign->dtNoise_init)); // not work
  // }
  // else
  // {
    p_assign->gtSAMgraph.add(ligo::DdtSmoothFactor(C(frame_num-1), C(frame_num), p_assign->ddtNoise));
    // p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector1>(C(frame_num), gtsam::Vector1(rcv_ddt), p_assign->ddtNoise));
  
  p_assign->gtSAMgraph.add(ligo::DtDdtFactor(B(frame_num-1), B(frame_num), C(frame_num-1), C(frame_num), rcv_sys, delta_t, p_assign->dtNoise)); // not work
  // }
  {
  // if (frame_num > 1)
  {
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate+1);
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate);
  }
  // else
  // {
    // p_assign->factor_id_frame.push_back(std::vector<size_t>(id_accumulate, id_accumulate+1));
  // }
  }
  // else
  // {
    // p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate+1);
    // p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate); 
  // }
  id_accumulate += 2;
  if (!nolidar)
  {
    bool no_weight = false;
    // if (frame_num < delete_thred)
    // {
      // p_assign->gtSAMgraph.add(ligo::GnssLioFactor(P(0), E(0), R(0), A(0), R(frame_num), A(frame_num), gravity_init, state_gravity, ba, bg, rot, sqrt_lidar, p_assign->odomaNoise)); //LioNoise)); // odomNoiseIMU));
    // }
    // else
    {
      // p_assign->gtSAMgraph.add(ligo::GnssLioHardFactorNoR(A(frame_num), ba, bg, sqrt_lidar, no_weight, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
      p_assign->gtSAMgraph.add(ligo::GnssLioFactor(P(0), E(0), R(frame_num), A(frame_num), O(frame_num), G(frame_num), gravity_init, state_gravity, pos, vel, rot, ba, bg, acc, omg, sqrt_lidar, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
    }
      // p_assign->gtSAMgraph.add(ligo::GnssLioHardFactor(R(frame_num), A(frame_num), ba, bg, rot, sqrt_lidar, no_weight, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;    
  }
  else
  {
    p_assign->gtSAMgraph.add(ligo::GnssLioFactorNolidar(R(frame_num-1), F(frame_num-1), R(frame_num), F(frame_num), rel_rot, rel_pos, rel_vel, 
                  state_gravity, delta_t, ba, bg, pre_integration, p_assign->odomNoiseIMU));
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate);
    id_accumulate += 1;
  }
  p_assign->initialEstimate.insert(C(frame_num), gtsam::Vector1(rcv_ddt));
  p_assign->initialEstimate.insert(B(frame_num), gtsam::Vector4(rcv_dt[0], rcv_dt[1], rcv_dt[2], rcv_dt[3]));  
  for (uint32_t j = 0; j < meas_index_sats_final.size(); j++)
  {
    double values[11];
    values[0] = Tex_imu_r[0]; values[1] = Tex_imu_r[1]; values[2] = Tex_imu_r[2]; // values[3] = anc_local[0]; values[4] = anc_local[1]; values[5] = anc_local[2];
    values[3] = meas_svpos_sats_final[j][0]; values[4] = meas_svpos_sats_final[j][1]; values[5] = meas_svpos_sats_final[j][2]; 
    // values[9] = meas_svpos_sats_final[j][0]; values[10] = meas_svpos_sats_final[j][1]; values[11] = meas_svpos_sats_final[j][2];
    // values[12] = sv_pos_best[0]; values[13] = sv_pos_best[1]; values[14] = sv_pos_best[2]; 
    values[6] = sv_pos_pair[j][0]; values[7] = sv_pos_pair[j][1]; values[8] = sv_pos_pair[j][2];
    values[9] = meas_cp[j] - meas_sats_final[j]; values[10] = cp_weight; //_adjust; 
    // values[14] = rcv_dt[0] - p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(meas_index_sats_final[j]))[0] + dt_com;
    if (!nolidar)
    {
      Eigen::Vector3d RTex1 = rot * Tex_imu_r;
      values[0] = RTex1[0]; values[1] = RTex1[1]; values[2] = RTex1[2]; 
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise_init));
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise));
      }
      // p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise));
    }
    else
    {
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise_init)); // not work
      }
      else
      {// p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise)); // not work
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise)); // not work
      }// Eigen::Matrix3d rot_before = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(meas_index_sats_final[j])).matrix();
      // p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidarPos(F(meas_index_sats_final[j]), F(frame_num), values, rot_before, rot_pos, p_assign->robustcpNoise)); // not work
    }
    // factor_id_cur.push_back(id_accumulate);
    p_assign->factor_id_frame[meas_index_sats_final[j]-frame_delete].push_back(id_accumulate);
    id_accumulate += 1;
  }

  {
    p_assign->factor_id_frame.push_back(factor_id_cur);
    std::vector<size_t>().swap(factor_id_cur);
  }
  // if (meas_index_sats_final.size() < 4)
  // {
  //   runISAM2opt();
  //   frame_num ++;
  //   return false;
  // }
  return true;
}

void GNSSProcess::SetInit()
{
  if (!nolidar)
  {
    // Eigen::Matrix3d R_enu_local_;
    // R_enu_local_ = R_ecef_enu; // * Rot_gnss_init; // * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()) 
    // prior factor 
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector;
    Eigen::Matrix<double, 12, 1> init_others_vector;
    // init_vel_bias_vector.block<3,1>(0,0) = Rot_gnss_init.transpose() * pos_window[wind_size];
    init_vel_bias_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
    init_vel_bias_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(6,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(9,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    // dt[0] = para_rcv_dt[wind_size*4]; dt[1] = para_rcv_dt[wind_size*4+1], dt[2] = para_rcv_dt[wind_size*4+2], dt[3] = para_rcv_dt[wind_size*4+3];
    // ddt = para_rcv_ddt[wind_size];
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(Rot_gnss_init)); //.transpose() * rot_window[wind_size]));
    p_assign->initialEstimate.insert(G(0), gtsam::Vector3(gravity_init)); //.transpose() * rot_window[wind_size]));
    // p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(A(0), gtsam::Vector6(init_vel_bias_vector));
    p_assign->initialEstimate.insert(O(0), gtsam::Vector12(init_others_vector));
    // p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]));
    p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size])); //(1429495.922912-134967.935, 1429510.167255-134967.935, 1429516.520987-134967.935, 1429082.399893-134967.935)); //
    // p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[wind_size]));
    p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[0])); //(163.119147)); //(161.874045) 
    // p_assign->initialEstimate.insert(Y(0), gtsam::Vector1(yaw_enu_local));
    p_assign->initialEstimate.insert(E(0), gtsam::Vector3(anc_ecef[0], anc_ecef[1], anc_ecef[2]));
    // cout << anc_ecef.transpose() << endl;
    p_assign->initialEstimate.insert(P(0), gtsam::Rot3(R_ecef_enu));

    gtsam::PriorFactor<gtsam::Rot3> init_rot_ext(P(0), gtsam::Rot3(gtsam::Rot3(R_ecef_enu)), p_assign->priorextrotNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_pos_ext(E(0), gtsam::Vector3(anc_ecef[0], anc_ecef[1], anc_ecef[2]), p_assign->priorextposNoise);
    // gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]), p_assign->priordtNoise);
    gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size], para_rcv_dt[4*wind_size]), p_assign->priordtNoise);
    // gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[wind_size]), p_assign->priorddtNoise);
    gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[0]), p_assign->priorddtNoise); // (161.874045) 163.119147
    gtsam::PriorFactor<gtsam::Rot3> init_rot_(R(0), gtsam::Rot3(Rot_gnss_init), p_assign->priorrotNoise);
    gtsam::PriorFactor<gtsam::Vector6> init_vel_(A(0), gtsam::Vector6(init_vel_bias_vector), p_assign->priorNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector12> init_bias_(O(0), gtsam::Vector12(init_others_vector), p_assign->priorBiasNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_grav_(G(0), gtsam::Vector3(gravity_init), p_assign->priorGravNoise);
    p_assign->gtSAMgraph.add(init_rot_ext);
    p_assign->gtSAMgraph.add(init_pos_ext);
    p_assign->gtSAMgraph.add(init_dt);
    p_assign->gtSAMgraph.add(init_ddt);
    p_assign->gtSAMgraph.add(init_rot_);
    p_assign->gtSAMgraph.add(init_vel_);
    p_assign->gtSAMgraph.add(init_bias_);
    p_assign->gtSAMgraph.add(init_grav_);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3, 4, 5, 6, 7});
    id_accumulate += 8;
  }
  else
  {
  //   Eigen::Matrix3d R_enu_local_;
  //   R_enu_local_ = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ());
    // dt[0] = para_rcv_dt[wind_size*4], dt[1] = para_rcv_dt[wind_size*4+1], dt[2] = para_rcv_dt[wind_size*4+2], dt[3] = para_rcv_dt[wind_size*4+3];
    // ddt = para_rcv_ddt[wind_size];
    gtsam::PriorFactor<gtsam::Rot3> init_rot(R(0), gtsam::Rot3(R_ecef_enu * rot_window[wind_size]), p_assign->priorrotNoise); //  * R_enu_local_
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = anc_ecef + R_ecef_enu * (pos_window[wind_size] - rot_window[wind_size] * Tex_imu_r); //  * R_enu_local_- pos_window[0]
    init_vel_bias_vector.block<3,1>(3,0) = R_ecef_enu * vel_window[wind_size]; // R_enu_local_ * 
    init_vel_bias_vector.block<6,1>(6,0) = Eigen::Matrix<double, 6, 1>::Zero();
    gtsam::PriorFactor<gtsam::Vector12> init_vel_bias(F(0), gtsam::Vector12(init_vel_bias_vector), p_assign->priorposNoise);
    // gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]), p_assign->priordtNoise);
    gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0]), p_assign->priordtNoise);
    gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[0]), p_assign->priorddtNoise); // para_rcv_ddt[wind_size]
    p_assign->gtSAMgraph.add(init_rot);
    p_assign->gtSAMgraph.add(init_vel_bias);
    p_assign->gtSAMgraph.add(init_dt);
    p_assign->gtSAMgraph.add(init_ddt);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3}); //{i * 4, i * 4 + 1, i * 4  + 2, i * 4 + 3});
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(R_ecef_enu * rot_window[wind_size])); // R_enu_local_ * 
    p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    // p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]));
    p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0]));
    p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[0])); // para_rcv_ddt[wind_size]
    id_accumulate += 4;
  }
}