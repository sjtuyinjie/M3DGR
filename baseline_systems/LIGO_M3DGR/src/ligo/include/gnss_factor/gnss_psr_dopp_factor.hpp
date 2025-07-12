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

#ifndef GNSS_PSR_DOPP_FACTOR_H_
#define GNSS_PSR_DOPP_FACTOR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
using namespace gnss_comm;

namespace ligo {

#define PSR_TO_DOPP_RATIO (5)

class GnssPsrDoppFactor : public gtsam::NoiseModelFactor6<gtsam::Rot3, gtsam::Vector6, gtsam::Vector4, gtsam::Vector1, gtsam::Vector3, gtsam::Rot3> 
{
    public: 
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // GnssPsrDoppFactor() = delete;
        GnssPsrDoppFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, gtsam::Key j5, gtsam::Key j6, bool invalid_lidar_, double values_[30], int sys_idx_, 
        Eigen::Vector3d hat_omg_T_, const gtsam::SharedNoiseModel& model) :
        hat_omg_T(hat_omg_T_), sys_idx(sys_idx_), invalid_lidar(invalid_lidar_),
        gtsam::NoiseModelFactor6<gtsam::Rot3, gtsam::Vector6, gtsam::Vector4, gtsam::Vector1, gtsam::Vector3, gtsam::Rot3>(model, j1, j2, j3, j4, j5, j6) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            anc_local << values_[3], values_[4], values_[5];
            sv_pos << values_[6], values_[7], values_[8];
            sv_vel << values_[9], values_[10], values_[11];
            svdt = values_[12];
            tgd = values_[13];
            svddt = values_[14];
            pr_uura = values_[15];
            dp_uura = values_[16];
            relative_sqrt_info = values_[17];
            for (int i = 0; i < 8; i++)
            {
                latest_gnss_iono_params.push_back(values_[18+i]);
            }
            time_current = values_[26];
            freq = values_[27];
            psr_measured = values_[28];
            dopp_measured = values_[29];
        }

        virtual ~GnssPsrDoppFactor() {}

        void jacobAnc(Eigen::Vector3d &R1TE3, Eigen::Vector3d &vecLon, Eigen::Vector3d &vecLat, Eigen::Vector3d &ref_ecef) const
        {
            double s1, s2, e2, a, ep, p, h, lat, ds1dx, ds2dx, ds1dy, ds2dy, ds1dz, ds2dz, sins, coss;
            e2 = EARTH_ECCE_2;
            a = EARTH_SEMI_MAJOR; // _glo?
            ep = ref_ecef(0)*ref_ecef(0) + ref_ecef(1) * ref_ecef(1);
            p = a*a*(1-e2);
            h = ref_ecef(2)*ref_ecef(2)*a*a;
            s1 = ref_ecef(2) + e2/(1-e2) * sqrt(p) * pow(ref_ecef(2)*a/sqrt(h+ep*p),3);
            s2 = sqrt(ep) - a * e2 * pow((ep*p)/(h+ep*p),1.5);
            lat = atan(s1/s2);
            sins = -s1/(s1*s1+s2*s2);
            coss = s2/(s1*s1+s2*s2);
            
            R1TE3 << 0.0, -sin(lat-M_PI/2), cos(lat-M_PI/2);
            ds1dx = e2/(1-e2) * sqrt(p) * a * ref_ecef(2) * h * (-3) * p * ref_ecef(0) / pow(h+ep*p,2.5);
            ds1dy = e2/(1-e2) * sqrt(p) * a * ref_ecef(2) * h * (-3) * p * ref_ecef(1) / pow(h+ep*p,2.5);
            ds1dz = 1 + e2/(1-e2) * sqrt(p) * 3 *sqrt(h/(h+ep*p)) * a * a * ref_ecef(2) * ep * p / pow(h+ep*p,2);

            ds2dx = ref_ecef(0) / sqrt(ep) - a * e2 * pow(p,1.5) * 3 * sqrt(ep)*ref_ecef(0)*h/pow(h+ep*p,2.5);
            ds2dy = ref_ecef(1) / sqrt(ep) - a * e2 * pow(p,1.5) * 3 * sqrt(ep)*ref_ecef(1)*h/pow(h+ep*p,2.5);
            ds2dz = a*e2*3 * pow(p,1.5) * a * a * ref_ecef(2) * pow(ep, 1.5) / pow(h+ep*p, 2.5);

            vecLon << -ref_ecef(1)/ep, ref_ecef(0)/ep, 0.0;
            vecLat << coss * ds1dx + sins * ds2dx, coss * ds1dy + sins * ds2dy, coss * ds1dz + sins * ds2dz;
        }

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot, const gtsam::Vector6 &pos_vel_bias, const gtsam::Vector4 &dt, const gtsam::Vector1 &ddt, const gtsam::Vector3 &ext_p, const gtsam::Rot3 &ext_R, //const gtsam::Vector3 &anc, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none, 
            boost::optional<gtsam::Matrix&> H5 = boost::none, boost::optional<gtsam::Matrix&> H6 = boost::none) const //, boost::optional<gtsam::Matrix&> H7 = boost::none) const
        {
            Eigen::Vector3d ref_ecef = ext_p;

            const Eigen::Vector3d local_pos = rot * Tex_imu_r + pos_vel_bias.segment<3>(0);
            const Eigen::Vector3d local_vel = pos_vel_bias.segment<3>(3) + rot * hat_omg_T;

            // Eigen::Matrix3d R_enu_local;
            // R_enu_local = ext_R.matrix();
            // Eigen::Matrix3d R_ecef_enu_cur = ecef2rotation(ref_ecef); // provide anchor value
            Eigen::Matrix3d R_ecef_local = ext_R.matrix(); // R_ecef_enu_cur * R_enu_local;

            Eigen::Vector3d P_ecef = R_ecef_local * (local_pos - anc_local) + ref_ecef;
            
            Eigen::Vector3d V_ecef = R_ecef_local * local_vel;

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
            double dp_weight = sin_el_2 / dp_uura * relative_sqrt_info * PSR_TO_DOPP_RATIO; // not requisite

            Eigen::Vector3d rcv2sat_ecef = sv_pos - P_ecef;
            Eigen::Vector3d rcv2sat_unit = rcv2sat_ecef.normalized();
            
            const double wavelength = LIGHT_SPEED / freq;

            const double psr_sagnac = EARTH_OMG_GPS*(sv_pos(0)*P_ecef(1)-sv_pos(1)*P_ecef(0))/LIGHT_SPEED;
            double psr_estimated = rcv2sat_ecef.norm() + psr_sagnac + dt[sys_idx] - svdt*LIGHT_SPEED + // why not multiply light_speed?  
                                    ion_delay + tro_delay + tgd*LIGHT_SPEED;
            const double dopp_sagnac = EARTH_OMG_GPS/LIGHT_SPEED*(sv_vel(0)*P_ecef(1)+
                    sv_pos(0)*V_ecef(1) - sv_vel(1)*P_ecef(0) - sv_pos(1)*V_ecef(0));
            double dopp_estimated = (sv_vel - V_ecef).dot(rcv2sat_unit) + ddt[0] + dopp_sagnac - svddt*LIGHT_SPEED;

            Eigen::Matrix3d hat_T, hat_hat_omg_T;
            hat_T << SKEW_SYM_MATRX(Tex_imu_r);
            hat_hat_omg_T << SKEW_SYM_MATRX(hat_omg_T);
            gtsam::Vector2 residual;
            
            {    
                residual[0] = (psr_estimated - psr_measured) * pr_weight;

                residual[1] = (dopp_estimated + dopp_measured*wavelength) * dp_weight; // / wavelength; // 
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
                    (*H1).block<1,3>(0,0) = rcv2sat_unit.transpose() * R_ecef_local * rot.matrix() * hat_T * pr_weight;
                    (*H1).block<1,3>(1,0) = -(sv_vel-V_ecef).transpose() * unit2rcv_pos * R_ecef_local * rot.matrix() * hat_T * dp_weight
                                            + rcv2sat_unit.transpose() * R_ecef_local * rot.matrix() * hat_hat_omg_T * dp_weight;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(2,6);
                    (*H2).block<1,3>(0,0) = -rcv2sat_unit.transpose() * R_ecef_local * pr_weight;
                    (*H2).block<1,3>(1,0) = (sv_vel-V_ecef).transpose() * unit2rcv_pos * R_ecef_local * dp_weight;
                    (*H2).block<1,3>(1,3) = rcv2sat_unit.transpose() * (-1.0) * R_ecef_local * dp_weight;
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(2,4);
                    (*H3)(0, sys_idx) = 1.0 * pr_weight;
                }

                if (H4)
                {
                    (*H4) = gtsam::Matrix::Zero(2,1);
                    (*H4)(1,0) = 1.0 * dp_weight;
                }

                if (H5)
                {
                    // Eigen::Vector3d R1TE3, vecLon, vecLat, vecP, vecV;
                    // jacobAnc(R1TE3, vecLon, vecLat, ref_ecef);

                    // vecP = R_enu_local * (local_pos - anc_local);
                    // vecV = R_enu_local * local_vel;
                    
                    // Eigen::Vector3d E1;
                    // E1 << 1.0, 0.0, 0.0;

                    // Eigen::Matrix3d hatP, hatV;
                    // hatP << 0.0, -vecP(2), vecP(1),
                    //     vecP(2), 0.0, -vecP(0),
                    //     -vecP(1), vecP(0), 0.0;
                    // hatV << 0.0, -vecV(2), vecV(1),
                    //     vecV(2), 0.0, -vecV(0),
                    //     -vecV(1), vecV(0), 0.0;
                    (*H5) = gtsam::Matrix::Zero(2,3);
                    // if (!invalid_lidar)
                    {
                    (*H5).block<1,3>(0,0) = -rcv2sat_unit.transpose() * pr_weight;
                    // (*H5).block<1,3>(0,0) = -rcv2sat_unit.transpose() * (Eye3d + R_ecef_enu_cur * hatP * R1TE3 * vecLon.transpose() - R_ecef_enu_cur * hatP * E1 * vecLat.transpose()) * pr_weight;
                    (*H5).block<1,3>(1,0) = (sv_vel-V_ecef).transpose() * unit2rcv_pos * dp_weight;
                    // (*H5).block<1,3>(1,0) = (sv_vel-V_ecef).transpose() * unit2rcv_pos * (Eye3d + R_ecef_enu_cur * hatP * R1TE3 * vecLon.transpose() - R_ecef_enu_cur * hatP * E1 * vecLat.transpose()) * dp_weight
                                            // + rcv2sat_unit.transpose() * (-1.0) * (R_ecef_enu_cur * hatV * R1TE3 * vecLon.transpose() - R_ecef_enu_cur * hatV * E1 * vecLat.transpose()) * dp_weight;             
                    }
                }

                if (H6)
                {
                    (*H6) = gtsam::Matrix::Zero(2,3);
                    // if (!invalid_lidar)
                    {
                    Eigen::Vector3d pos_v = local_pos - anc_local;
                    Eigen::Matrix3d d_pos, d_vel;
                    if (pos_vel_bias.segment<3>(3).norm() > 0.3)
                    {
                        d_pos << 0.0, -pos_v[2], pos_v[1], 
                                    pos_v[2], 0.0, -pos_v[0], 
                                    -pos_v[1], pos_v[0], 0.0;
                        d_vel << 0.0, -local_vel[2], local_vel[1], 
                                    local_vel[2], 0.0, -local_vel[0], 
                                    -local_vel[1], local_vel[0], 0.0;
                        // d_pos << 0.0, 0.0, pos_v[1], 
                        //             0.0, 0.0, -pos_v[0], 
                        //             0.0, 0.0, 0.0;
                        // d_vel << 0.0, 0.0, local_vel[1], 
                        //             0.0, 0.0, -local_vel[0], 
                        //             0.0, 0.0, 0.0;
                    // }
                    // else
                    // {
                    //     d_pos << 0.0, -pos_v[2], 0.0, 
                    //                 pos_v[2], 0.0, 0.0, 
                    //                 -pos_v[1], pos_v[0], 0.0;
                    //     d_vel << 0.0, -local_vel[2], 0.0, 
                    //                 local_vel[2], 0.0, 0.0, 
                    //                 -local_vel[1], local_vel[0], 0.0;
                    // }
                    (*H6).block<1,3>(0,0) = rcv2sat_unit.transpose() * (R_ecef_local * d_pos) * pr_weight;
                    (*H6).block<1,3>(1,0) = rcv2sat_unit.transpose() * (R_ecef_local * d_vel) * dp_weight - (sv_vel-V_ecef).transpose() * unit2rcv_pos * 
                                    R_ecef_local * d_pos * dp_weight;
                    // printf("check hessian:%f, %f, %f\n", (*H6)(0, 0), (*H6)(0, 1), (*H6)(0, 2));
                    }
                    }
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d Tex_imu_r, anc_local, sv_pos, sv_vel, hat_omg_T;
        double svdt, tgd, svddt, pr_uura, dp_uura, relative_sqrt_info, time_current, freq, psr_measured, dopp_measured;
        std::vector<double> latest_gnss_iono_params;
        int sys_idx;
        bool invalid_lidar;
};
}

#endif