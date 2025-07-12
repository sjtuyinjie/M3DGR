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

#ifndef NMEA_LIO_GRAV_REL_FACTOR_H_
#define NMEA_LIO_GRAV_REL_FACTOR_H_

#include <Eigen/Dense>
#include <vector>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace ligo {

class NmeaLioGravRelFactor : public gtsam::NoiseModelFactor5<gtsam::Rot3, gtsam::Rot3, gtsam::Vector6, gtsam::Vector12, gtsam::Vector3> //, gtsam::Vector3>
{
    public: 
        NmeaLioGravRelFactor(gtsam::Key i1, gtsam::Key k1, gtsam::Key k2, gtsam::Key j1, gtsam::Key j2, Eigen::Vector3d grav_init_, Eigen::Vector3d grav_lio_, 
        Eigen::Vector3d pos_lio_, Eigen::Vector3d vel_lio_, Eigen::Matrix3d rot_lio_, Eigen::Vector3d ba_lio_, Eigen::Vector3d bg_lio_, Eigen::Vector3d acc_lio_, Eigen::Vector3d omg_lio_, 
        Eigen::Matrix<double, 24, 24>  sqrt_lidar_, const gtsam::SharedNoiseModel& model) : 
        rot_lio(rot_lio_), grav(grav_init_), grav_lio(grav_lio_), pos_lio(pos_lio_), vel_lio(vel_lio_), omg_lio(omg_lio_), acc_lio(acc_lio_), bg_lio(bg_lio_), ba_lio(ba_lio_), sqrt_lidar(sqrt_lidar_), 
        gtsam::NoiseModelFactor5<gtsam::Rot3, gtsam::Rot3, gtsam::Vector6, gtsam::Vector12, gtsam::Vector3>(model, i1, k1, k2, j1, j2) {}
        
        virtual ~NmeaLioGravRelFactor() {}

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot_ext, const gtsam::Rot3 &rot1, const gtsam::Vector6 &pos_vel1, const gtsam::Vector12 &others_, const gtsam::Vector3 &grav_,
        boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
        boost::optional<gtsam::Matrix&> H5 = boost::none) const
        {
            // Eigen::Matrix3d d = rot1.transpose() * rot2.matrix();
            // Eigen::Vector3d delta_p = pos_vel2.segment<3>(0) - pos_vel1.segment<3>(0); // - pos_vel_bias1.segment<3>(3) * dt - 0.5 * grav * dt * dt);
            // Eigen::Vector3d delta_v = pos_vel2.segment<3>(3) - pos_vel1.segment<3>(3);

            Eigen::Matrix3d res_R = rot_lio.transpose() * rot1.matrix();
            Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
            // Eigen::Matrix3d res_R = rot_lio.transpose() * rot.matrix();
            // Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
            Eigen::Vector3d gm = rot_ext.matrix().transpose() * grav_lio;
            Eigen::Matrix3d hat_grav;
            hat_grav << SKEW_SYM_MATRX(gm);
            
            if (H1)
            {
                (*H1) = gtsam::Matrix::Zero(27, 3);
                (*H1).block<3, 3>(0, 0) = hat_grav;
            }
            if (H2) 
            {
                (*H2) = gtsam::Matrix::Zero(27, 3);
                (*H2).block<3, 3>(6,0) = -Jacob_right_inv<double>(res_r); // * d.transpose();  //- d.transpose(); 
                // (*H2).block<3, 3>(6, 0) = Jacob_right_inv<double>(res_r); // gtsam::Matrix::Identity(4, 4);
                (*H2).block<24, 3>(3, 0) = sqrt_lidar * (*H2).block<24, 3>(3, 0);
            }
            if (H3)
            {
                (*H3) = gtsam::Matrix::Zero(27, 6);
                (*H3).block<3, 3>(3, 0) = gtsam::Matrix::Identity(3, 3);
                (*H3).block<3, 3>(9, 3) = gtsam::Matrix::Identity(3, 3);
                // (*H2).block<6, 6>(9, 6) = gtsam::Matrix::Zero(6, 6);
                (*H3).block<24, 6>(3, 0) = sqrt_lidar * (*H3).block<24, 6>(3, 0);
            }
            if (H4) 
            {
                (*H4) = gtsam::Matrix::Zero(27, 12);
                (*H4).block<6, 6>(12, 0) = gtsam::Matrix::Identity(6, 6);
                (*H4).block<6, 6>(21, 6) = gtsam::Matrix::Identity(6, 6); 
                (*H4).block<24, 12>(3, 0) = sqrt_lidar * (*H4).block<24, 12>(3, 0);
            }
            if (H5)
            {
                (*H5) = gtsam::Matrix::Zero(27, 3);
                (*H5).block<3, 3>(18, 0) = gtsam::Matrix::Identity(3, 3);
                (*H5).block<24, 3>(3, 0) = sqrt_lidar * (*H5).block<24, 3>(3, 0);
            }
            gtsam::Vector residual(27);
            residual.segment<3>(0) = gm - grav_lio;
            residual.segment<3>(6) = res_r;
            residual.segment<3>(3) = pos_vel1.segment<3>(0)- pos_lio; // pos2- pos1 - rel_pos;
            residual.segment<3>(9) = pos_vel1.segment<3>(3) - vel_lio; // vel2 - vel1 - rel_vel;
            residual.segment<3>(12) = others_.segment<3>(0) - omg_lio; // vel2 - vel1 - rel_vel;
            residual.segment<3>(15) = others_.segment<3>(3) - acc_lio; // vel2 - vel1 - rel_vel;
            residual.segment<3>(21) = others_.segment<3>(6) - bg_lio; // vel2 - vel1 - rel_vel;
            residual.segment<3>(24) = others_.segment<3>(9) - ba_lio; // vel2 - vel1 - rel_vel;
            residual.segment<3>(18) = grav_ - grav_lio; // vel2 - vel1 - rel_vel;
            residual.segment<24>(3) = sqrt_lidar * residual.segment<24>(3);
            return residual;
        }
    private:
        Eigen::Matrix<double, 3, 1> pos_lio;
        Eigen::Matrix<double, 3, 1> vel_lio, grav, grav_lio, bg_lio, ba_lio, omg_lio, acc_lio;
        Eigen::Matrix3d rot_lio;
        Eigen::Matrix<double, 24, 24> sqrt_lidar;
        bool no_weight;
};
}
#endif