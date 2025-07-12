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
#include <vector>
#include <eigen3/Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <common_lib.h>
#include <numeric>
#include <opencv2/core/eigen.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <fstream>

#include <gnss_factor/gnss_cp_factor_nor.hpp>
// #include <gnss_factor/gnss_cp_factor_pos.hpp>
#include <gnss_factor/gnss_lio_rel_factor.hpp>
// #include <gnss_factor/gnss_lio_gravity_hard_factor.hpp>
// #include <gnss_factor/gnss_lio_gravity_factor.hpp>
#include <gnss_factor/gnss_cp_factor_nolidar.hpp>
// #include <gnss_factor/gnss_cp_factor_nolidar_pos.hpp>
#include <gnss_factor/gnss_ddt_smooth_factor.hpp>
#include <gnss_factor/gnss_dt_ddt_factor.hpp>
#include <gnss_factor/gnss_lio_factor.hpp>
#include <gnss_factor/gnss_lio_factor_nolidar.hpp>
#include <gnss_factor/gnss_prior_factor.hpp>
#include <gnss_factor/gnss_psr_dopp_factor_nor.hpp>
// #include <gnss_factor/gnss_psr_dopp_factor_pos.hpp>
#include <gnss_factor/gnss_psr_dopp_factor_nolidar.hpp>
// #include <gnss_factor/gnss_psr_dopp_factor_nolidar_pos.hpp>

using namespace gnss_comm;

using gtsam::symbol_shorthand::R; // Pose3 ()
using gtsam::symbol_shorthand::P; // Pose3 (x,y,z,r,p,y)
// using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // clock drift (dt_g,dt_r,dt_e,dt_c)
using gtsam::symbol_shorthand::C; // rate of clock drift  (ddt)
using gtsam::symbol_shorthand::E; // ext_p
using gtsam::symbol_shorthand::F; // pos, vel and imu bias (vel ba bg)
using gtsam::symbol_shorthand::A; // pos, vel
using gtsam::symbol_shorthand::O; // pos, vel
using gtsam::symbol_shorthand::G; // pos, vel
// using gtsam::symbol_shorthand::G; // ext_R
// using gtsam::symbol_shorthand::Y; // local enu (yaw)
// using gtsam::symbol_shorthand::A; // anchor point (anc) total = 18 dimensions

struct sat_first
{
    double timecur;
    Eigen::Vector3d RTex;
    int frame_num;

    bool operator <(const sat_first &s) const
    {
        return (timecur < s.timecur);
    }
};

class GNSSAssignment
{
    public:
        GNSSAssignment();
        GNSSAssignment(const GNSSAssignment&) = delete;
        GNSSAssignment& operator=(const GNSSAssignment&) = delete;
        ~GNSSAssignment() {};
        // ofstream fout_std;

        // use GTSAM
        gtsam::NonlinearFactorGraph gtSAMgraph; // store factors //
        gtsam::Values initialEstimate; // store initial values of the node //

        gtsam::Values isamCurrentEstimate; // 
        gtsam::ISAM2 isam;

        double prior_noise = 0.01;
        double marg_noise = 0.01;
        double ddt_noise = 0.01;
        double dt_noise = 0.01;
        double odo_noise = 0.01;
        double grav_noise = 0.01;
        double psr_dopp_noise = 0.01;
        double cp_noise = 0.01;

        double ave_std = 0.0;
        int num_std = 0;

        bool outlier_rej = false;
        double outlier_thres = 0.1;
        double outlier_thres_init = 10;
        // int gnss_track_num_threshold = 20;
        int process_feat_num = 0;

        gtsam::noiseModel::Base::shared_ptr margrotNoise;
        gtsam::noiseModel::Base::shared_ptr margposNoise;
        // gtsam::noiseModel::Diagonal::shared_ptr priorvelNoise;
        gtsam::noiseModel::Base::shared_ptr margNoise;
        gtsam::noiseModel::Base::shared_ptr margdtNoise;
        gtsam::noiseModel::Base::shared_ptr margddtNoise;

        gtsam::noiseModel::Base::shared_ptr priorrotNoise;
        gtsam::noiseModel::Base::shared_ptr priorposNoise;
        gtsam::noiseModel::Base::shared_ptr priorextrotNoise;
        gtsam::noiseModel::Base::shared_ptr priorNoise;
        gtsam::noiseModel::Base::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Base::shared_ptr priorGravNoise;
        gtsam::noiseModel::Base::shared_ptr priordtNoise;
        gtsam::noiseModel::Base::shared_ptr priorextposNoise;
        gtsam::noiseModel::Base::shared_ptr dtNoise;
        gtsam::noiseModel::Base::shared_ptr dtNoise_init;
        gtsam::noiseModel::Base::shared_ptr priorddtNoise;
        gtsam::noiseModel::Base::shared_ptr ddtNoise;
        gtsam::noiseModel::Base::shared_ptr ddtNoise_init;
        gtsam::noiseModel::Base::shared_ptr odomNoise;
        gtsam::noiseModel::Base::shared_ptr odomaNoise;
        // gtsam::noiseModel::Diagonal::shared_ptr odomNoiseIMU;
        gtsam::noiseModel::Base::shared_ptr odomNoiseIMU;
        gtsam::noiseModel::Base::shared_ptr robustpsrdoppNoise;
        gtsam::noiseModel::Base::shared_ptr robustpsrdoppNoise_init;
        gtsam::noiseModel::Base::shared_ptr robustcpNoise;
        gtsam::noiseModel::Base::shared_ptr robustcpNoise_init;
        gtsam::noiseModel::Base::shared_ptr adapNoise;
        // gtsam::noiseModel::Gaussian::shared_ptr testNoise;
        void initNoises(void); 

        int marg_thred = 1;
        int change_ext = 1;
        std::deque<std::vector<size_t>> factor_id_frame; // 

        std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
        std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem_rnx;
        std::vector<double> latest_gnss_iono_params;
        bool ephem_from_rinex = false;
        bool obs_from_rinex = false;
        bool pvt_is_gt = true;
        std::map<uint32_t, std::map<double, size_t>> sat2time_index;
        std::map<uint32_t, std::map<double, size_t>> sat2time_index_rnx;
        void Ephemfromrinex(const std::string &rinex_filepath);
        void inputEphem(EphemBasePtr ephem_ptr);
        void rinex2iono_params(const std::string &rinex_filepath, std::vector<double> &iono_params);
        void rinex2ephems(const std::string &rinex_filepath, std::map<uint32_t, std::vector<EphemBasePtr>> &sat2ephem_);
        void Obsfromrinex(const std::string &rinex_filepath, std::queue<std::vector<ObsPtr>> &rinex_meas);
        int freq_idx_ = 0;
        ObsPtr rinex_line2obs(const std::string rinex_str, const std::map<uint8_t, std::vector<std::string>> &sys2type);
        void rinex2obs(const std::string &rinex_filepath, std::queue<std::vector<ObsPtr>> &rinex_meas);
        // int satno_rtk(int sys, int prn);
        double gnss_psr_std_threshold = 30.0;
        double gnss_dopp_std_threshold = 30.0;
        double sum_d = 0, sum_d2 = 0;
        double gnss_cp_std_threshold = 30;
        // double hatch_filter_meas = 0, last_cp = 0;
        bool cp_locked = false;
        std::map<uint32_t, uint32_t> sat_track_status; //
        std::map<uint32_t, double> sat_track_time; //
        std::map<uint32_t, double> sat_track_last_time; //
        std::map<uint32_t, double> hatch_filter_meas; //
        // std::map<uint32_t, double> hatch_filter_noise; //
        std::map<uint32_t, double> last_cp_meas; //
        double gnss_elevation_threshold = 30;
        void processGNSSBase(const std::vector<ObsPtr> &gnss_meas, std::vector<double> &psr_meas, std::vector<ObsPtr> &valid_meas, std::vector<EphemBasePtr> &valid_ephems, bool gnss_ready, Eigen::Vector3d ecef_pos, double last_gnss_time_process);
        void delete_variables(bool nolidar, size_t frame_delete, int frame_num, size_t &id_accumulate, gtsam::FactorIndices delete_factor);

        double str2double(const std::string &num_str);
        EphemPtr rinex_line2ephem(const std::vector<std::string> &ephem_lines);
        GloEphemPtr rinex_line2glo_ephem(const std::vector<std::string> &ephem_lines, const uint32_t gpst_leap_seconds);
};