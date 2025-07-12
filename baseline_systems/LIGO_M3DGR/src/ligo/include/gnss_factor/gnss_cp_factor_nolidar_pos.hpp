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

#ifndef GNSS_CP_FACTOR_NOLIDAR_POS_H_
#define GNSS_CP_FACTOR_NOLIDAR_POS_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
using namespace gnss_comm;

namespace ligo {

class GnssCpFactorNolidarPos : public gtsam::NoiseModelFactor2<gtsam::Vector12, gtsam::Vector12>
{
    public: 
        GnssCpFactorNolidarPos(gtsam::Key i1, gtsam::Key j1, double values_[20], Eigen::Matrix3d rot1_, Eigen::Matrix3d rot2_, const gtsam::SharedNoiseModel& model) :
        rot1(rot1_), rot2(rot2_), gtsam::NoiseModelFactor2<gtsam::Vector12, gtsam::Vector12>(model, i1, j1) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            anc_local << values_[3], values_[4], values_[5];
            sv_pos_bi << values_[6], values_[7], values_[8];
            sv_pos_pi << values_[9], values_[10], values_[11];
            sv_pos_bj << values_[12], values_[13], values_[14];
            sv_pos_pj << values_[15], values_[16], values_[17];
            cp_measured = values_[18];
            cp_weight = values_[19];
        }

        virtual ~GnssCpFactorNolidarPos() {}
        
        gtsam::Vector evaluateError(const gtsam::Vector12 &pos1, const gtsam::Vector12 &pos2, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const
        {
            const Eigen::Vector3d local_pos1 = rot1 * Tex_imu_r + pos1.segment<3>(0);
            const Eigen::Vector3d local_pos2 = rot2 * Tex_imu_r + pos2.segment<3>(0);

            Eigen::Vector3d P_ecef1, P_ecef2;
            {
                P_ecef1 = local_pos1;
                P_ecef2 = local_pos2;
            }

            Eigen::Vector3d rcv2sat_ecef_bj = sv_pos_bj - P_ecef2;
            Eigen::Vector3d rcv2sat_ecef_bi = sv_pos_bi - P_ecef1;
            Eigen::Vector3d rcv2sat_ecef_pj = sv_pos_pj - P_ecef2;
            Eigen::Vector3d rcv2sat_ecef_pi = sv_pos_pi - P_ecef1;
            Eigen::Vector3d rcv2sat_unit_bj = rcv2sat_ecef_bj.normalized();
            Eigen::Vector3d rcv2sat_unit_bi = rcv2sat_ecef_bi.normalized();
            Eigen::Vector3d rcv2sat_unit_pj = rcv2sat_ecef_pj.normalized();
            Eigen::Vector3d rcv2sat_unit_pi = rcv2sat_ecef_pi.normalized();

            gtsam::Vector1 residual;

            {
                residual[0] = (rcv2sat_ecef_bj.norm() - rcv2sat_ecef_pj.norm() - rcv2sat_ecef_bi.norm() + rcv2sat_ecef_pi.norm() - cp_measured) * cp_weight;

                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(1, 12);
                    (*H1).block<1,3>(0,0) = (rcv2sat_unit_bi-rcv2sat_unit_pi).transpose() * cp_weight;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(1, 12);
                    (*H2).block<1,3>(0,0) = -(rcv2sat_unit_bj-rcv2sat_unit_pj).transpose() * cp_weight;
                }

                return residual;
            }
        }
    private:
        Eigen::Vector3d sv_pos_bi, sv_pos_bj, sv_pos_pi, sv_pos_pj, Tex_imu_r, anc_local;
        Eigen::Matrix3d rot1, rot2;
        double cp_measured, cp_weight;
};
}

#endif