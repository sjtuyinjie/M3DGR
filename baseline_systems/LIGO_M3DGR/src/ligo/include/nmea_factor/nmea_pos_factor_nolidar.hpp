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

#ifndef NMEA_FACTOR_NOLIDAR_H_
#define NMEA_FACTOR_NOLIDAR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>

namespace ligo {

class NMEAFactorNolidar : public gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector12>
{
    public: 
        NMEAFactorNolidar(gtsam::Key j1, gtsam::Key j2, double values_[17], Eigen::Vector3d hat_omg_T_, Eigen::Matrix3d Rex_imu_r_, const gtsam::SharedNoiseModel& model) :
        hat_omg_T(hat_omg_T_), Rex_imu_r(Rex_imu_r_), gtsam::NoiseModelFactor2<gtsam::Rot3, gtsam::Vector12>(model, j1, j2) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            pos_meas << values_[6], values_[7], values_[8];
            // vel_meas << values_[9], values_[10], values_[11];
            // rot_meas = Eigen::Quaterniond(values_[12], values_[13], values_[14], values_[15]).normalized().toRotationMatrix();
            relative_sqrt_info = values_[16];
        }

        virtual ~NMEAFactorNolidar() {}

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot, const gtsam::Vector12 &pos_vel,
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const
        {
            Eigen::Vector3d P_enu = pos_vel.segment<3>(0) + rot.matrix() * Tex_imu_r;
            
            // Eigen::Vector3d V_local = rot.matrix().transpose() * pos_vel.segment<3>(3);
            // Eigen::Vector3d V_enu = V_local + hat_omg_T;

            // Eigen::Matrix3d R_enu = rot.matrix() * Rex_imu_r;

            gtsam::Vector3 residual;
            Eigen::Matrix3d hat_T; //, hat_T_omg;
            hat_T << SKEW_SYM_MATRX(Tex_imu_r);
            // hat_T_omg << SKEW_SYM_MATRX(V_local);
            
            {    
                residual.segment<3>(0) = (P_enu - pos_meas) * relative_sqrt_info;

                // residual.segment<3>(3) = (V_enu - vel_meas) * relative_sqrt_info;

                // Eigen::Matrix3d res_R = rot_meas.transpose() * R_enu;
                // Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));

                // residual.segment<3>(6) = relative_sqrt_info * res_r;

                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(3,3);
                    (*H1).block<3, 3>(0, 0) = -relative_sqrt_info * rot.matrix() * hat_T;
                    // (*H1).block<3, 3>(3, 0) = relative_sqrt_info * hat_T_omg;
                    // (*H1).block<3, 3>(6, 0) = relative_sqrt_info * Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose();
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(3,12);
                    (*H2).block<3, 3>(0, 0) = relative_sqrt_info * Eigen::Matrix3d::Identity();
                    // (*H2).block<3, 3>(3, 3) = relative_sqrt_info * rot.matrix().transpose();
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d Tex_imu_r, hat_omg_T, pos_meas, vel_meas;
        Eigen::Matrix3d Rex_imu_r, rot_meas;
        double relative_sqrt_info;
};
}

#endif