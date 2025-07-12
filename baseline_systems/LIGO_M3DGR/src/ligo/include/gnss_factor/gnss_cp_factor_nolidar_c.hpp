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

#ifndef GNSS_CP_FACTOR_NOLIDAR_H_
#define GNSS_CP_FACTOR_NOLIDAR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
using namespace gnss_comm;

namespace ligo {

class GnssCpFactorNolidar : public gtsam::NoiseModelFactor5<gtsam::Rot3, gtsam::Vector12, gtsam::Rot3, gtsam::Vector12, gtsam::Vector1>
{
    public: 
        GnssCpFactorNolidar(gtsam::Key i1, gtsam::Key i2, gtsam::Key j1, gtsam::Key j2, gtsam::Key i3, size_t sys_idx_, double delta_t_, double values_[11], const gtsam::SharedNoiseModel& model) :
        sys_idx(sys_idx_), delta_t(delta_t_), gtsam::NoiseModelFactor5<gtsam::Rot3, gtsam::Vector12, gtsam::Rot3, gtsam::Vector12, gtsam::Vector1>(model, i1, i2, j1, j2, i3) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            // anc_local << values_[3], values_[4], values_[5];
            sv_pos_i << values_[3], values_[4], values_[5];
            // sv_pos_pi << values_[9], values_[10], values_[11];
            sv_pos_j << values_[6], values_[7], values_[8];
            // sv_pos_pj << values_[15], values_[16], values_[17];
            cp_measured = values_[9];
            cp_weight = values_[10];
        }

        virtual ~GnssCpFactorNolidar() {}
        
        gtsam::Vector evaluateError(const gtsam::Rot3 &rot1, const gtsam::Vector12 &pos1, const gtsam::Rot3 &rot2, const gtsam::Vector12 &pos2, 
            const gtsam::Vector1 &ddt, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, 
            boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
            boost::optional<gtsam::Matrix&> H5 = boost::none) const
        {
            const Eigen::Vector3d P_ecef1 = rot1 * Tex_imu_r + pos1.segment<3>(0);
            const Eigen::Vector3d P_ecef2 = rot2 * Tex_imu_r + pos2.segment<3>(0);

            // Eigen::Vector3d P_ecef1, P_ecef2;
            // {
                // P_ecef1 = local_pos1;
                // P_ecef2 = local_pos2;
            // }

            Eigen::Vector3d rcv2sat_ecef_j = sv_pos_j - P_ecef2;
            Eigen::Vector3d rcv2sat_ecef_i = sv_pos_i - P_ecef1;
            // Eigen::Vector3d rcv2sat_ecef_pj = sv_pos_pj - P_ecef2;
            // Eigen::Vector3d rcv2sat_ecef_pi = sv_pos_pi - P_ecef1;
            Eigen::Vector3d rcv2sat_unit_j = rcv2sat_ecef_j.normalized();
            Eigen::Vector3d rcv2sat_unit_i = rcv2sat_ecef_i.normalized();
            // Eigen::Vector3d rcv2sat_unit_pj = rcv2sat_ecef_pj.normalized();
            // Eigen::Vector3d rcv2sat_unit_pi = rcv2sat_ecef_pi.normalized();

            Eigen::Matrix3d hat_T;
            hat_T << SKEW_SYM_MATRX(Tex_imu_r);

            gtsam::Vector1 residual;

            {
                // double dt_com1 = 0, dt_com2 = 0;
                // if (sys_idx > 0) 
                // {
                //     dt_com1 = dt1[sys_idx];
                //     dt_com2 = dt2[sys_idx];
                // }
                residual[0] = (rcv2sat_ecef_j.norm() - rcv2sat_ecef_i.norm() + ddt[0] * delta_t - cp_measured) * cp_weight;

                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(1, 3);
                    (*H1).block<1,3>(0,0) = -rcv2sat_unit_i.transpose() * rot1.matrix() * hat_T * cp_weight;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(1, 12);
                    (*H2).block<1,3>(0,0) = rcv2sat_unit_i.transpose() * cp_weight;
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(1, 3);
                    (*H3).block<1,3>(0,0) = rcv2sat_unit_j.transpose() * rot2.matrix() * hat_T * cp_weight;
                }

                if (H4)
                {
                    (*H4) = gtsam::Matrix::Zero(1, 12);
                    (*H4).block<1,3>(0,0) = -rcv2sat_unit_j.transpose() * cp_weight;
                }

                if (H5)
                {
                    (*H5) = gtsam::Matrix::Zero(1, 1);
                    // if (sys_idx > 0) (*H5)(0, sys_idx) = -cp_weight;
                    (*H5)(0, 0) = delta_t * cp_weight;
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d sv_pos_i, sv_pos_j, Tex_imu_r, anc_local;
        double cp_measured, cp_weight, delta_t;
        size_t sys_idx;
};
}

#endif