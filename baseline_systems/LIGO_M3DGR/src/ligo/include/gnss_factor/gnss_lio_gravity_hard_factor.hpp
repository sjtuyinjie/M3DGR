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

#ifndef GNSS_LIO_GRAV_HARD_FACTOR_H_
#define GNSS_LIO_GRAV_HARD_FACTOR_H_

#include <Eigen/Dense>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
using namespace gnss_comm;
#define PI  3.1415926535897932384626433832795

namespace ligo {

class GnssLioGravHardFactor : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector6> //, gtsam::Vector3>
{
    public: 
        GnssLioGravHardFactor(gtsam::Key i1, gtsam::Key i2, gtsam::Key j1, gtsam::Key j2, Eigen::Vector3d grav_init_, Eigen::Vector3d grav_lio_, 
        Eigen::Vector3d pos_lio_, Eigen::Vector3d vel_lio_, Eigen::Matrix3d rot_lio_, Eigen::Matrix<double, 9, 9>  sqrt_lidar_, bool no_weight_, const gtsam::SharedNoiseModel& model) : 
        rot_lio(rot_lio_), grav(grav_init_), grav_lio(grav_lio_), pos_lio(pos_lio_), vel_lio(vel_lio_), sqrt_lidar(sqrt_lidar_), no_weight(no_weight_), 
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Rot3, gtsam::Vector6>(model, i1, i2, j1, j2) {}
        
        virtual ~GnssLioGravHardFactor() {}

        Eigen::Matrix<double, 3, 3> Exp(double v1, double v2, double v3) const
        {
            double &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
            Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
            if (norm > 0.00001)
            {
                double r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
                Eigen::Matrix<double, 3, 3> K;
                K << 0.0, -r_ang[2], r_ang[1],
                        r_ang[2], 0.0, -r_ang[0],
                        -r_ang[1], r_ang[0], 0.0;

                /// Roderigous Tranformation
                return Eigen::Matrix3d::Identity() + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
            }
            else
            {
                return Eigen::Matrix3d::Identity();
            }
        }

        void grav_jaco(Eigen::Matrix3d rot, Eigen::Vector3d xyz, Eigen::Vector3d &gravm, Eigen::Matrix3d &der_rot, Eigen::Matrix3d &der_pos) const
        {
            double e2 = EARTH_ECCE_2;
            double a = EARTH_SEMI_MAJOR;
            double a2 = a * a;
            double b2 = a2 * (1 - e2);
            double b = sqrt(b2);
            double ep2 = (a2 - b2) / b2;
            double p = xyz.head<2>().norm();
            double p2 = p * p;

            // two sides and hypotenuse of right angle triangle with one angle = theta:
            double s1 = xyz.z() * a;
            double s2 = p * b;
            double h = sqrt(s1 * s1 + s2 * s2);
            double sin_theta = s1 / h;
            double cos_theta = s2 / h;

            // two sides and hypotenuse of right angle triangle with one angle = lat:
            s1 = xyz.z() + ep2 * b * pow(sin_theta, 3);
            s2 = p - a * e2 * pow(cos_theta, 3);
            // h = sqrt(s1 * s1 + s2 * s2);
            double tan_lat = s1 / s2;
            // double sin_lat = s1 / h;
            // double cos_lat = s2 / h;
            double lat = atan(tan_lat) - PI / 2;
            // double lat_deg = lat * R2D;

            // double N = a2 * pow((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat), -0.5);
            // double altM = p / cos_lat - N;

            double lon = atan2(xyz.y(), xyz.x()) + PI / 2;
            double deci2 = xyz.z() * xyz.z() * a2 + p2 * b2;
            double deci = sqrt(deci2);
            // double lon_deg = lon * R2D;
            Eigen::Matrix3d rot_lon = Exp(0.0, 0.0, lon);            
            Eigen::Matrix3d rot_lat = Exp(-lat, 0.0, 0.0);     
            Eigen::Vector3d gm_lon = rot_lat * grav;
            Eigen::Vector3d gm = rot.transpose() * rot_lon * gm_lon;
            gravm = gm;
            Eigen::Matrix3d hat_gm, hat_g, hat_gm_lon;
            hat_gm << 0.0, -gm(2), gm(1),
                    gm(2), 0.0, -gm(0),
                    -gm(1), gm(0), 0.0;     
            hat_g << 0.0, -grav(2), grav(1),
                    grav(2), 0.0, -grav(0),
                    -grav(1), grav(0), 0.0;  
            hat_gm_lon << 0.0, -gm_lon(2), gm_lon(1),
                    gm_lon(2), 0.0, -gm_lon(0),
                    -gm_lon(1), gm_lon(0), 0.0;   
            der_rot = hat_gm;
            Eigen::Vector3d dg_dlat = -rot.transpose() * rot_lon * hat_gm_lon.block<3, 1>(0, 2);
            Eigen::Vector3d dg_dlon = rot.transpose() * rot_lon * rot_lat * hat_g.block<3, 1>(0, 0);
            double dlon_dx = -xyz.y() / p2;
            double dlon_dy = xyz.x() / p2;
            double dlat_dx = 1 / (1 + tan_lat * tan_lat) * (-s1 / s2 / s2 * (xyz.x() / p - 3*a*e2*(p2*b2/deci2)*(xyz.x()*b/p/deci-p*b2*b*xyz.x()/deci2/deci))
                            - 1/s2*(3*b*ep2*xyz.z()*xyz.z()*a2/deci2*xyz.z()*a*b2*xyz.x()/deci2/deci));
            double dlat_dy = 1 / (1 + tan_lat * tan_lat) * (-s1 / s2 / s2 * (xyz.y() / p - 3*a*e2*(p2*b2/deci2)*(xyz.y()*b/p/deci-p*b2*b*xyz.y()/deci2/deci))
                            - 1/s2*(3*b*ep2*xyz.z()*xyz.z()*a2/deci2*xyz.z()*a*b2*xyz.y()/deci2/deci));
            double dlat_dz = 1 / (1 + tan_lat * tan_lat) * (-tan_lat / s2 *(3*a*e2*p2*b2/deci2)*p*b*a2*xyz.z()/deci2/deci
                            +1/s2*(1+3*ep2*b*xyz.z()*xyz.z()*a2/deci2*(a/deci-a2*a*xyz.z()*xyz.z()/deci2/deci)));
            der_pos.block<3, 1>(0, 0) = dg_dlon * dlon_dx + dg_dlat * dlat_dx;
            der_pos.block<3, 1>(0, 1) = dg_dlon * dlon_dy + dg_dlat * dlat_dy;
            der_pos.block<3, 1>(0, 2) = dg_dlat * dlat_dz;
        }

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot_ext, const gtsam::Vector3 &pos_ext, const gtsam::Rot3 &rot, const gtsam::Vector6 &pos_vel,
        boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const
        {
            Eigen::Matrix3d res_R = rot_lio.transpose() * rot.matrix();
            Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
            Eigen::Matrix3d der_rot, der_pos;
            Eigen::Vector3d gm;
            grav_jaco(rot_ext.matrix(), pos_ext, gm, der_rot, der_pos);

            if (H1)
            {
                (*H1) = gtsam::Matrix::Zero(12, 3);
                (*H1).block<3, 3>(0, 0) = der_rot;
            }
            if (H2)
            {
                (*H2) = gtsam::Matrix::Zero(12, 3);
                (*H2).block<3, 3>(0, 0) = der_pos;
            }
            if (H3) 
            {
                (*H3) = gtsam::Matrix::Zero(12, 3);
                (*H3).block<3, 3>(6, 0) = Jacob_right_inv<double>(res_r); // gtsam::Matrix::Identity(4, 4);
                if (!no_weight) (*H3).block<9, 3>(3, 0) = sqrt_lidar * (*H3).block<9, 3>(3, 0);
            }
            if (H4)
            {
                (*H4) = gtsam::Matrix::Zero(12, 6);
                (*H4).block<3, 3>(3, 0) = gtsam::Matrix::Identity(3, 3);
                (*H4).block<3, 3>(9, 3) = gtsam::Matrix::Identity(3, 3);
                // (*H2).block<6, 6>(9, 6) = gtsam::Matrix::Zero(6, 6);
                if (!no_weight) (*H4).block<9, 6>(3, 0) = sqrt_lidar * (*H4).block<9, 6>(3, 0);
            }
            gtsam::Vector residual(12);
            residual.segment<3>(0) = gm - grav_lio;
            residual.segment<3>(6) = res_r;
            residual.segment<3>(3) = pos_vel.block<3, 1>(0, 0) - pos_lio;
            residual.segment<3>(9) = pos_vel.block<3, 1>(3, 0) - vel_lio;
            if (!no_weight) residual.segment<9>(3) = sqrt_lidar * residual.segment<9>(3);
            return residual;
        }
    private:
        Eigen::Matrix<double, 3, 1> pos_lio;
        Eigen::Matrix<double, 3, 1> vel_lio, grav, grav_lio;
        Eigen::Matrix3d rot_lio;
        Eigen::Matrix<double, 9, 9> sqrt_lidar;
        bool no_weight;
};
}
#endif