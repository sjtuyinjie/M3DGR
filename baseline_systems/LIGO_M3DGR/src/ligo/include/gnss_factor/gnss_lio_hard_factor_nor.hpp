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

#ifndef GNSS_LIO_HARD_FACTOR_NOR_H_
#define GNSS_LIO_HARD_FACTOR_NOR_H_

#include <Eigen/Dense>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
using namespace gnss_comm;

namespace ligo {

class GnssLioHardFactorNoR : public gtsam::NoiseModelFactor1<gtsam::Vector6> //, gtsam::Vector3>
{
    public: 
        GnssLioHardFactorNoR(gtsam::Key j1, Eigen::Vector3d pos_lio_, Eigen::Vector3d vel_lio_, Eigen::Matrix<double, 6, 6>  sqrt_lidar_, bool no_weight_, const gtsam::SharedNoiseModel& model) : 
        pos_lio(pos_lio_), vel_lio(vel_lio_), sqrt_lidar(sqrt_lidar_), no_weight(no_weight_), gtsam::NoiseModelFactor1<gtsam::Vector6>(model, j1) {}
        
        virtual ~GnssLioHardFactorNoR() {}
        gtsam::Vector evaluateError(const gtsam::Vector6 &pos_vel,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const
        {
            if (H1)
            {
                (*H1) = gtsam::Matrix::Zero(6, 6);
                (*H1).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3);
                (*H1).block<3, 3>(3, 3) = gtsam::Matrix::Identity(3, 3);
                // (*H2).block<6, 6>(9, 6) = gtsam::Matrix::Zero(6, 6);
                if (!no_weight) (*H1) = sqrt_lidar * (*H1);
            }
            gtsam::Vector residual(6);
            // residual.segment<3>(3) = res_r;
            residual.segment<3>(0) = pos_vel.block<3, 1>(0, 0) - pos_lio;
            residual.segment<3>(3) = pos_vel.block<3, 1>(3, 0) - vel_lio;
            if (!no_weight) residual = sqrt_lidar * residual;
            return residual;
        }
    private:
        Eigen::Matrix<double, 3, 1> pos_lio;
        Eigen::Matrix<double, 3, 1> vel_lio;
        Eigen::Matrix<double, 6, 6> sqrt_lidar;
        bool no_weight;
};
}
#endif