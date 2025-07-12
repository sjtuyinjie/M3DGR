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

#ifndef GNSS_PSR_DOPP_FACTOR_NOLIDAR_H_
#define GNSS_PSR_DOPP_FACTOR_NOLIDAR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
using namespace gnss_comm;

namespace ligo {

class GnssPsrDoppFactorNolidar : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector12, gtsam::Vector4, gtsam::Vector1>
{
    public: 
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // GnssPsrDoppFactor() = delete;
        GnssPsrDoppFactorNolidar(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, double values_[27], int sys_idx_, Eigen::Vector3d hat_omg_T_, const gtsam::SharedNoiseModel& model) :
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector12, gtsam::Vector4, gtsam::Vector1>(model, j1, j2, j3, j4), sys_idx(sys_idx_), hat_omg_T(hat_omg_T_) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            sv_pos << values_[3], values_[4], values_[5];
            sv_vel << values_[6], values_[7], values_[8];
            svdt = values_[9];
            tgd = values_[10];
            svddt = values_[11];
            pr_uura = values_[12];
            dp_uura = values_[13];
            relative_sqrt_info = values_[14];
            for (int i = 0; i < 8; i++)
            {
                latest_gnss_iono_params.push_back(values_[15+i]);
            }
            time_current = values_[23];
            freq = values_[24];
            psr_measured = values_[25];
            dopp_measured = values_[26];
        }
        virtual ~GnssPsrDoppFactorNolidar() {}
        gtsam::Vector evaluateError(const gtsam::Rot3 &rot, const gtsam::Vector12 &pos_vel_bias, const gtsam::Vector4 &dt, const gtsam::Vector1 &ddt,
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, 
            boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const
        {          
            const Eigen::Vector3d P_ecef = rot * Tex_imu_r + pos_vel_bias.segment<3>(0);
            const Eigen::Vector3d V_ecef = pos_vel_bias.segment<3>(3) + rot * hat_omg_T;

            // Eigen::Vector3d P_ecef = local_pos;
            
            // Eigen::Vector3d V_ecef = local_vel;

            double ion_delay = 0, tro_delay = 0;
            double azel[2] = {0, M_PI/2.0};
            if (P_ecef.norm() > 0)
            {
                sat_azel(P_ecef, sv_pos, azel);
                Eigen::Vector3d rcv_lla = ecef2geo(P_ecef);
                tro_delay = calculate_trop_delay(sec2time(time_current), rcv_lla, azel);
                ion_delay = calculate_ion_delay(sec2time(time_current), latest_gnss_iono_params, rcv_lla, azel); // rely on local pose
            }
            double sin_el = sin(azel[1]);
            double sin_el_2 = sin_el*sin_el;
            double pr_weight = sin_el_2 / pr_uura * relative_sqrt_info; // not requisite
            double dp_weight = sin_el_2 / dp_uura * relative_sqrt_info * 10; // * PSR_TO_DOPP_RATIO; // not requisite

            Eigen::Vector3d rcv2sat_ecef = sv_pos - P_ecef;
            Eigen::Vector3d rcv2sat_unit = rcv2sat_ecef.normalized();
            
            const double wavelength = LIGHT_SPEED / freq;
            // double dt_com = 0;
            // if (sys_idx > 0) dt_com = dt[sys_idx];
            const double psr_sagnac = EARTH_OMG_GPS*(sv_pos(0)*P_ecef(1)-sv_pos(1)*P_ecef(0))/LIGHT_SPEED;
            double psr_estimated = rcv2sat_ecef.norm() + psr_sagnac + dt[sys_idx] - svdt*LIGHT_SPEED + // why not multiply light_speed?  + dt_com
                                    ion_delay + tro_delay + tgd*LIGHT_SPEED;
            const double dopp_sagnac = EARTH_OMG_GPS/LIGHT_SPEED*(sv_vel(0)*P_ecef(1)+
                    sv_pos(0)*V_ecef(1) - sv_vel(1)*P_ecef(0) - sv_pos(1)*V_ecef(0));
            double dopp_estimated = (sv_vel - V_ecef).dot(rcv2sat_unit) + ddt[0] + dopp_sagnac - svddt*LIGHT_SPEED;

            Eigen::Matrix3d hat_T, hat_T_omg;
            hat_T << SKEW_SYM_MATRX(Tex_imu_r);
            hat_T_omg << SKEW_SYM_MATRX(hat_omg_T);
            gtsam::Vector2 residual;
            
            {    
                residual[0] = (psr_estimated - psr_measured) * pr_weight;
            
                residual[1] = (dopp_estimated + dopp_measured*wavelength) * dp_weight;
                const double norm3 = pow(rcv2sat_ecef.norm(), 3);
                const double norm2 = rcv2sat_ecef.squaredNorm();
                Eigen::Matrix3d unit2rcv_pos;
                for (size_t i = 0; i < 3; ++i)
                {
                    for (size_t k = 0; k < 3; ++k)
                    {
                        if (i == k)
                            unit2rcv_pos(i, k) = (norm2-rcv2sat_ecef(i)*rcv2sat_ecef(i))/norm3;
                        else
                            unit2rcv_pos(i, k) = (-rcv2sat_ecef(i)*rcv2sat_ecef(k))/norm3;
                    }
                }
                unit2rcv_pos *= -1;
                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(2,3);
                    (*H1).block<1,3>(0,0) = rcv2sat_unit.transpose() * rot.matrix() * hat_T * pr_weight; 
                    (*H1).block<1,3>(1,0) = -(sv_vel-V_ecef).transpose() * unit2rcv_pos * rot.matrix() * hat_T_omg * dp_weight;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(2,12);
                    (*H2).block<1,3>(0,0) = -rcv2sat_unit.transpose() * pr_weight; 
                    (*H2).block<1,3>(1,0) = (sv_vel-V_ecef).transpose() * unit2rcv_pos * dp_weight;
                    (*H2).block<1,3>(1,3) = rcv2sat_unit.transpose() * (-1.0) * dp_weight;
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(2,4);
                    // if (sys_idx > 0) (*H3)(0, sys_idx) = 1.0 * pr_weight;
                    (*H3)(0, sys_idx) = 1.0 * pr_weight;
                }

                if (H4)
                {
                    (*H4) = gtsam::Matrix::Zero(2,1);
                    (*H4)(1,0) = 1.0 * dp_weight;
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d Tex_imu_r, anc_local, sv_pos, sv_vel, hat_omg_T;
        double svdt, tgd, svddt, pr_uura, dp_uura, relative_sqrt_info, time_current, freq, psr_measured, dopp_measured;
        std::vector<double> latest_gnss_iono_params;
        int sys_idx;        
};
}

#endif