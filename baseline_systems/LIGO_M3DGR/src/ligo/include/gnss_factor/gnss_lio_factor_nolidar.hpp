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

#ifndef GNSS_LIO_FACTOR_NOLIDAR_H_
#define GNSS_LIO_FACTOR_NOLIDAR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include "integration_base.h"
using namespace gnss_comm;

namespace ligo {

class GnssLioFactorNolidar : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector12, gtsam::Rot3, gtsam::Vector12>
{
    public: 
        GnssLioFactorNolidar(gtsam::Key i1, gtsam::Key i2, gtsam::Key j1, gtsam::Key j2, gtsam::Rot3 rel_rot_, gtsam::Point3 rel_pos_, gtsam::Vector3 rel_vel_,
            gtsam::Vector3 grav_, double dt_, gtsam::Vector3 bias_ai_, gtsam::Vector3 bias_gi_, IntegrationBase* pre_integration_, const gtsam::SharedNoiseModel& model) :
            gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector12, gtsam::Rot3, gtsam::Vector12>(model, i1, i2, j1, j2), 
            rel_rot(rel_rot_), rel_pos(rel_pos_), rel_vel(rel_vel_), grav(grav_), dt(dt_), linearized_ba(bias_ai_), linearized_bg(bias_gi_), pre_integration(pre_integration_) {
        }
        virtual ~GnssLioFactorNolidar() {}
        gtsam::Vector evaluateError(const gtsam::Rot3 &rot1, const gtsam::Vector12 &pos_vel_bias1, const gtsam::Rot3 &rot2, const gtsam::Vector12 &pos_vel_bias2, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const
        {
            Eigen::Matrix3d d = rot1.transpose() * rot2.matrix();
            Eigen::Vector3d delta_p = rot1.transpose() * (pos_vel_bias2.segment<3>(0) - pos_vel_bias1.segment<3>(0) - pos_vel_bias1.segment<3>(3) * dt - 0.5 * grav * dt * dt);
            Eigen::Vector3d delta_v = rot1.transpose() * (pos_vel_bias2.segment<3>(3) - pos_vel_bias1.segment<3>(3) - grav * dt);

            Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(0, 9);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(0, 12);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(3, 12);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(6, 9);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(6, 12);
            // cout << "check cov:" << pre_integration->covariance << ";" << pre_integration->delta_p.transpose() << endl;
            Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();

            Eigen::Vector3d dba = pos_vel_bias1.segment<3>(6) - linearized_ba; // only bai
            Eigen::Vector3d dbg = pos_vel_bias1.segment<3>(9) - linearized_bg; // only bgi
            Eigen::Vector3d delta_r = dq_dbg * dbg;
            // double _dt_ = 1.0;
            Eigen::Matrix3d corrected_delta_q = rel_rot.matrix() * Exp<double>(delta_r);
            Eigen::Matrix3d res_R = corrected_delta_q.transpose() * d.matrix();
            Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
        
            if (H1) 
            {
                (*H1) = gtsam::Matrix::Zero(15,3);
                (*H1).block<3,3>(3,0) = -Jacob_right_inv<double>(res_r) * d.transpose(); // - d.transpose(); 
                (*H1).block<3,3>(0,0) = skew_sym_mat(delta_p); 
                (*H1).block<3,3>(6,0) = skew_sym_mat(delta_v); 
                (*H1) = sqrt_info * (*H1);
            }
            if (H2) 
            {
                (*H2) = gtsam::Matrix::Zero(15,12);
                (*H2).block<3,3>(0,0) = - rot1.transpose(); // gtsam::Matrix::Identity(3,3); 
                (*H2).block<3,3>(0,3) = - rot1.transpose() * dt; // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(6,3) = - rot1.transpose(); // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(3,9) = -Jacob_right_inv<double>(res_r) * corrected_delta_q.transpose() * d.matrix() * dq_dbg; // -dq_dbg; // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(0,6) = -dp_dba; // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(0,9) = -dp_dbg; // gtsam::Matrix::Identity(3,3);ssss
                (*H2).block<3,3>(6,6) = -dv_dba; // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(6,9) = -dv_dbg; // gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(9,6) = - gtsam::Matrix::Identity(3,3);
                (*H2).block<3,3>(12,9) = - gtsam::Matrix::Identity(3,3);
                (*H2) = sqrt_info * (*H2);
            }
            if (H3) 
            {
                (*H3) = gtsam::Matrix::Zero(15,3);
                (*H3).block<3,3>(3,0) = Jacob_right_inv<double>(res_r); // gtsam::Matrix::Identity(3,3); 
                (*H3) = sqrt_info * (*H3);
            }
            if (H4) 
            {
                (*H4) = gtsam::Matrix::Zero(15,12);
                (*H4).block<3,3>(0,0) = rot1.transpose(); 
                (*H4).block<3,3>(6,3) = rot1.transpose();
                (*H4).block<3,3>(9,6) = gtsam::Matrix::Identity(3,3);
                (*H4).block<3,3>(12,9) = gtsam::Matrix::Identity(3,3);
                (*H4) = sqrt_info * (*H4);
            }

            // Eigen::Vector3d dba = pos_vel_bias1.segment<3>(6) - linearized_ba; // only bai
            // Eigen::Vector3d dbg = pos_vel_bias1.segment<3>(9) - linearized_bg; // only bgi

            // Eigen::Vector3d delta_r = dq_dbg * dbg;
            // // double _dt_ = 1.0;
            // Eigen::Matrix3d corrected_delta_q = rel_rot.matrix() * Exp(delta_r);
            Eigen::Vector3d corrected_delta_v = rel_vel + dv_dba * dba + dv_dbg * dbg;
            Eigen::Vector3d corrected_delta_p = rel_pos + dp_dba * dba + dp_dbg * dbg;

            gtsam::Vector residual(15);
            // Eigen::Matrix3d res_R = corrected_delta_q.transpose() * d.matrix();
            residual.segment<3>(3) = res_r; // gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
            residual.segment<3>(0) = delta_p - corrected_delta_p;
            residual.segment<3>(6) = delta_v - corrected_delta_v;
            residual.segment<6>(9) = pos_vel_bias2.segment<6>(6) - pos_vel_bias1.segment<6>(6);
            residual = sqrt_info * residual;
            // cout << "check residual:" << residual.transpose() << ";" << sqrt_info << endl;
            return residual;
        }
    private:
        gtsam::Rot3 rel_rot;
        gtsam::Point3 rel_pos;
        gtsam::Vector3 rel_vel;
        gtsam::Vector3 linearized_ba;
        gtsam::Vector3 linearized_bg;
        double dt;
        gtsam::Vector3 grav;
        IntegrationBase* pre_integration;
};
}

#endif