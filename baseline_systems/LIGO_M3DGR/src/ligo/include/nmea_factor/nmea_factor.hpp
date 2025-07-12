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

#ifndef NMEA_FACTOR_H_
#define NMEA_FACTOR_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>

namespace ligo {

class NMEAFactor : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>
{
    public: 
        NMEAFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, bool invalid_lidar_, double values_[17],
        Eigen::Vector3d hat_omg_T_, Eigen::Matrix3d Rex_imu_r_, const gtsam::SharedNoiseModel& model) :
        hat_omg_T(hat_omg_T_), invalid_lidar(invalid_lidar_), Rex_imu_r(Rex_imu_r_),
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>(model, j1, j2, j3, j4) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            anc_local << values_[3], values_[4], values_[5];
            pos_meas << values_[6], values_[7], values_[8];
            vel_meas << values_[9], values_[10], values_[11];
            rot_meas = Eigen::Quaterniond(values_[12], values_[13], values_[14], values_[15]).normalized().toRotationMatrix();
            relative_sqrt_info = values_[16];
        }

        virtual ~NMEAFactor() {}

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot_ext, const gtsam::Vector3 &pos_ext, const gtsam::Vector6 &pos_vel, const gtsam::Rot3 &rot,
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, 
            boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const
        {
            Eigen::Vector3d ref_enu = pos_ext;

            const Eigen::Vector3d local_pos = rot * Tex_imu_r + pos_vel.segment<3>(0) - anc_local;
            const Eigen::Vector3d local_vel = rot.matrix().transpose()*pos_vel.segment<3>(3);

            Eigen::Matrix3d R_enu_local = rot_ext.matrix(); // R_ecef_enu_cur * R_enu_local;

            Eigen::Vector3d P_enu = R_enu_local * local_pos + ref_enu;
            
            Eigen::Vector3d V_enu = local_vel + hat_omg_T;

            Eigen::Matrix3d R_enu = R_enu_local * rot.matrix() * Rex_imu_r;


            gtsam::Vector9 residual;
            
            {    
                residual.segment<3>(0) = (P_enu - pos_meas) * relative_sqrt_info;

                residual.segment<3>(3) = (V_enu - vel_meas) * relative_sqrt_info;

                Eigen::Matrix3d res_R = rot_meas.transpose() * R_enu;
                Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));

                residual.segment<3>(6) = res_r * relative_sqrt_info;

                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(9,3);
                    Eigen::Matrix3d d_pos;
                        d_pos << 0.0, local_pos[2], -local_pos[1], 
                                    -local_pos[2], 0.0, local_pos[0], 
                                    local_pos[1], -local_pos[0], 0.0;
                    (*H1).block<3,3>(0,0) = R_enu_local * d_pos * relative_sqrt_info;
                    // (*H1).block<3,3>(3,0) = -d_vel * relative_sqrt_info;
                    (*H1).block<3,3>(6,0) = Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * rot.matrix().transpose() * relative_sqrt_info;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(9,3);
                    (*H2).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3) * relative_sqrt_info;
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(9,6);
                    (*H3).block<3, 3>(0, 0) = R_enu_local * relative_sqrt_info;
                    (*H3).block<3, 3>(3, 3) = rot.matrix().transpose() * relative_sqrt_info;
                }

                if (H4)
                {
                    Eigen::Matrix3d d_vel;
                    d_vel << 0.0, -local_vel[2], local_vel[1], 
                                local_vel[2], 0.0, -local_vel[0], 
                                -local_vel[1], local_vel[0], 0.0;
                    (*H4) = gtsam::Matrix::Zero(9, 3);
                    (*H4).block<3, 3>(3, 0) = d_vel * relative_sqrt_info;
                    (*H4).block<3, 3>(6, 0) = Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * relative_sqrt_info;
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d Tex_imu_r, anc_local, pos_meas, vel_meas, hat_omg_T;
        Eigen::Matrix3d rot_meas, Rex_imu_r;
        double relative_sqrt_info;
        bool invalid_lidar;
};
}

#endif