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

#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_spp.hpp>

using namespace gnss_comm;

class GNSSLIInitializer
{
    public:
        GNSSLIInitializer(const std::vector<std::vector<ObsPtr>> &gnss_meas_buf_, 
            const std::vector<std::vector<EphemBasePtr>> &gnss_ephem_buf_, 
            const std::vector<double> &iono_params_);
        GNSSLIInitializer(const GNSSLIInitializer&) = delete;
        GNSSLIInitializer& operator=(const GNSSLIInitializer&) = delete;
        ~GNSSLIInitializer() {};

        // get a rough location by SPP
        bool coarse_localization(Eigen::Matrix<double, 7, 1> &result);
        bool yaw_alignment(const std::vector<Eigen::Vector3d> &local_vs, const Eigen::Vector3d &rough_anchor_ecef, 
            double &aligned_yaw, double &rcv_ddt);
        bool anchor_refinement(const std::vector<Eigen::Vector3d> &local_ps, 
            const double aligned_yaw, const double aligned_ddt, 
            const Eigen::Matrix<double, 7, 1> &rough_ecef_dt, Eigen::Matrix<double, 7, 1> &refined_ecef_dt);
        bool yaw_refinement(const std::vector<Eigen::Vector3d> &local_vs, const Eigen::Vector3d &refined_anchor_ecef,
            const std::vector<Eigen::Vector3d> &local_ps, double &aligned_yaw, double &rcv_ddt);
    
        Eigen::Matrix<double, 7, 1> Psr_pos(const std::vector<ObsPtr> &obs, 
                const std::vector<EphemBasePtr> &ephems, const std::vector<double> &iono_params);
    private:
        const std::vector<std::vector<ObsPtr>> &gnss_meas_buf;
        const std::vector<std::vector<EphemBasePtr>> &gnss_ephem_buf;
        const std::vector<double> &iono_params;

        uint32_t num_all_meas;
        std::vector<std::vector<SatStatePtr>> all_sat_states;

        static constexpr uint32_t MAX_ITERATION = 10;
        static constexpr double   CONVERGENCE_EPSILON = 1e-5;
};

// #endif