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

#ifndef GNSS_CP_FACTOR_H_
#define GNSS_CP_FACTOR_H_

#include <vector>
#include <Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
using namespace gnss_comm;

namespace ligo {

class GnssCpFactor : public gtsam::NoiseModelFactor6<gtsam::Vector3, gtsam::Rot3, gtsam::Rot3, gtsam::Vector6, gtsam::Rot3, gtsam::Vector6>
{
    public: 
        GnssCpFactor(gtsam::Key f1, gtsam::Key f2, gtsam::Key i1, gtsam::Key i2, gtsam::Key j1, gtsam::Key j2, bool invalid_lidar_, double values_[20], const gtsam::SharedNoiseModel& model) :
        gtsam::NoiseModelFactor6<gtsam::Vector3, gtsam::Rot3, gtsam::Rot3, gtsam::Vector6, gtsam::Rot3, gtsam::Vector6>(model, f1, f2, i1, i2, j1, j2) {
            Tex_imu_r << values_[0], values_[1], values_[2];
            anc_local << values_[3], values_[4], values_[5];
            sv_pos_bi << values_[6], values_[7], values_[8];
            sv_pos_pi << values_[9], values_[10], values_[11];
            sv_pos_bj << values_[12], values_[13], values_[14];
            sv_pos_pj << values_[15], values_[16], values_[17];
            cp_measured = values_[18];
            cp_weight = values_[19];
            invalid_lidar = invalid_lidar_;
        }

        virtual ~GnssCpFactor() {}

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

        gtsam::Vector evaluateError(const gtsam::Vector3 &ext_p, const gtsam::Rot3 &ext_R, const gtsam::Rot3 &rot1, const gtsam::Vector6 &pos1, const gtsam::Rot3 &rot2, const gtsam::Vector6 &pos2, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none, 
            boost::optional<gtsam::Matrix&> H4 = boost::none, boost::optional<gtsam::Matrix&> H5 = boost::none, boost::optional<gtsam::Matrix&> H6 = boost::none) const
        {
            Eigen::Vector3d ref_ecef = ext_p;

            const Eigen::Vector3d local_pos1 = rot1 * Tex_imu_r + pos1.segment<3>(0);
            const Eigen::Vector3d local_pos2 = rot2 * Tex_imu_r + pos2.segment<3>(0);

            // Eigen::Matrix3d R_enu_local = ext_R.matrix();
            // Eigen::Matrix3d R_ecef_enu_cur = ecef2rotation(ref_ecef); // provide anchor value
            Eigen::Matrix3d R_ecef_local = ext_R.matrix(); // R_ecef_enu_cur * R_enu_local;

            Eigen::Vector3d P_ecef1, P_ecef2;
            {
                P_ecef1 = R_ecef_local * (local_pos1 - anc_local) + ref_ecef;
                P_ecef2 = R_ecef_local * (local_pos2 - anc_local) + ref_ecef;
            }

            Eigen::Vector3d rcv2sat_ecef_bj = sv_pos_bj - P_ecef2;
            Eigen::Vector3d rcv2sat_ecef_bi = sv_pos_bi - P_ecef1;
            Eigen::Vector3d rcv2sat_ecef_pj = sv_pos_pj - P_ecef2;
            Eigen::Vector3d rcv2sat_ecef_pi = sv_pos_pi - P_ecef1;
            Eigen::Vector3d rcv2sat_unit_bj = rcv2sat_ecef_bj.normalized();
            Eigen::Vector3d rcv2sat_unit_bi = rcv2sat_ecef_bi.normalized();
            Eigen::Vector3d rcv2sat_unit_pj = rcv2sat_ecef_pj.normalized();
            Eigen::Vector3d rcv2sat_unit_pi = rcv2sat_ecef_pi.normalized();

            Eigen::Matrix3d hat_T;
            hat_T << SKEW_SYM_MATRX(Tex_imu_r);

            gtsam::Vector1 residual;
            {
                residual[0] = (rcv2sat_ecef_bj.norm() - rcv2sat_ecef_pj.norm() - rcv2sat_ecef_bi.norm() + rcv2sat_ecef_pi.norm() - cp_measured) * cp_weight;
                // cout << "check cp residual:" << residual << endl;
                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(1,3);
                    
                    // Eigen::Vector3d vecP2, vecP1, vecLon, vecLat, R1TE3;
                    // jacobAnc(R1TE3, vecLon, vecLat, ref_ecef);

                    // vecP2 = R_enu_local * (local_pos2 - anc_local);
                    // vecP1 = R_enu_local * (local_pos1 - anc_local);

                    // Eigen::Vector3d E1;
                    // E1 << 1.0, 0.0, 0.0;

                    // Eigen::Matrix3d hatP1, hatP2;
                    // hatP1 << 0.0, -vecP1(2), vecP1(1),
                    //     vecP1(2), 0.0, -vecP1(0),
                    //     -vecP1(1), vecP1(0), 0.0;
                    // hatP2 << 0.0, -vecP2(2), vecP2(1),
                    //     vecP2(2), 0.0, -vecP2(0),
                    //     -vecP2(1), vecP2(0), 0.0;
                    // if (!invalid_lidar)
                    {
                    (*H1).block<1,3>(0,0) = (-rcv2sat_unit_bj + rcv2sat_unit_pj).transpose() * cp_weight + (rcv2sat_unit_bi - rcv2sat_unit_pi).transpose() * cp_weight;
                    // (*H1).block<1,3>(0,0) = (-rcv2sat_unit_bj + rcv2sat_unit_pj).transpose() * (Eye3d + R_ecef_enu_cur * hatP2 * R1TE3 * vecLon.transpose() - R_ecef_enu_cur * hatP2 * E1 * vecLat.transpose()) * cp_weight
                    //          + (rcv2sat_unit_bi - rcv2sat_unit_pi).transpose() * (Eye3d + R_ecef_enu_cur * hatP1 * R1TE3 * vecLon.transpose() - R_ecef_enu_cur * hatP1 * E1 * vecLat.transpose()) * cp_weight;
                    }
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(1,3);
                    // if (!invalid_lidar)
                    {
                    Eigen::Matrix3d d_pos1, d_pos2;
                    Eigen::Vector3d pos_v1 = local_pos1 - anc_local;
                    Eigen::Vector3d pos_v2 = local_pos2 - anc_local;
                    if (pos2.segment<3>(3).norm() > 0.3)
                    {
                        d_pos1 << 0.0, -pos_v1[2], pos_v1[1], 
                                    pos_v1[2], 0.0, -pos_v1[0], 
                                    -pos_v1[1], pos_v1[0], 0.0;
                        d_pos2 << 0.0, -pos_v2[2], pos_v2[1], 
                                    pos_v2[2], 0.0, -pos_v2[0], 
                                    -pos_v2[1], pos_v2[0], 0.0;
                        // d_pos1 << 0.0, 0.0, pos_v1[1], 
                        //             0.0, 0.0, -pos_v1[0], 
                        //             0.0, 0.0, 0.0;
                        // d_pos2 << 0.0, 0.0, pos_v2[1], 
                        //             0.0, 0.0, -pos_v2[0], 
                        //             0.0, 0.0, 0.0;
                    // }
                    // else
                    // {
                    //     d_pos1 << 0.0, -pos_v1[2], 0.0, 
                    //                 pos_v1[2], 0.0, 0.0, 
                    //                 -pos_v1[1], pos_v1[0], 0.0;
                    //     d_pos2 << 0.0, -pos_v2[2], 0.0, 
                    //                 pos_v2[2], 0.0, 0.0, 
                    //                 -pos_v2[1], pos_v2[0], 0.0;
                    // }
                    (*H2).block<1,3>(0,0) = (rcv2sat_unit_bj - rcv2sat_unit_pj).transpose() * R_ecef_local * d_pos2 * cp_weight
                                -(rcv2sat_unit_bi - rcv2sat_unit_pi).transpose() * R_ecef_local * d_pos1 * cp_weight;
                    // printf("check hessian:%f, %f, %f\n", (*H2)(0, 0), (*H2)(0, 1), (*H2)(0, 2));
                    }
                    }
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(1, 3);
                    (*H3).block<1,3>(0,0) = -(rcv2sat_unit_bi-rcv2sat_unit_pi).transpose() * R_ecef_local * rot1.matrix() * hat_T * cp_weight;
                }

                if (H4)
                {
                    (*H4) = gtsam::Matrix::Zero(1, 6);
                    (*H4).block<1,3>(0,0) = (rcv2sat_unit_bi-rcv2sat_unit_pi).transpose() * R_ecef_local * cp_weight;
                }

                if (H5)
                {
                    (*H5) = gtsam::Matrix::Zero(1, 3);
                    (*H5).block<1,3>(0,0) = (rcv2sat_unit_bj-rcv2sat_unit_pj).transpose() * R_ecef_local * rot2.matrix() * hat_T * cp_weight;
                }

                if (H6)
                {
                    (*H6) = gtsam::Matrix::Zero(1, 6);
                    (*H6).block<1,3>(0,0) = -(rcv2sat_unit_bj-rcv2sat_unit_pj).transpose() * R_ecef_local * cp_weight;
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d sv_pos_bi, sv_pos_bj, sv_pos_pi, sv_pos_pj, Tex_imu_r, anc_local;
        double cp_measured, cp_weight;
        bool invalid_lidar;
};
}

#endif