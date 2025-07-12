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

// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<V3D> pimu_list;
std::vector<PointVector> Nearest_Points; 
std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
std::shared_ptr<IVoxType> ivox_last_ = nullptr;                    // localmap in ivox
std::vector<double> knots_t;
std::vector<V3D> cp_pos;
std::vector<V3D> updatedmap;
std::vector<M3D> cp_rot;
std::vector<V3D> lidar_points;
std::deque<std::unordered_set<Eigen::Matrix<int, 3, 1>, faster_lio::hash_vec<3>>> empty_voxels;
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
bool   point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_output, 24, input_ikfom> kf_output;
double G_m_s2 = 9.81;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;  
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
int scan_count = 0;

Eigen::Matrix<double, 19, 19> process_noise_cov_input()
{
	Eigen::Matrix<double, 19, 19> cov;
	cov.setZero();
	cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
	cov.block<3, 3>(6, 6).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
	cov.block<3, 3>(10, 10).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	cov.block<3, 3>(13, 13).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}

Eigen::Matrix<double, 24, 24> process_noise_cov_output()
{
	Eigen::Matrix<double, 24, 24> cov;
	cov.setZero();
	cov.block<3, 3>(6, 6).diagonal() << vel_cov, vel_cov, vel_cov;
	cov.block<3, 3>(9, 9).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
	cov.block<3, 3>(12, 12).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
	cov.block<3, 3>(18, 18).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	cov.block<3, 3>(21, 21).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}

Eigen::Matrix<double, 24, 1> get_f_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 a_inertial = s.rot * s.acc; // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) = s.omg[i]; 
		res(i + 6) = a_inertial[i] + s.gravity[i]; 
	}
	return res;
}

Eigen::Matrix<double, 24, 24> df_dx_output(state_output &s, const input_ikfom &in, double delta_t)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Identity();

	SO3 delta_R;
	Eigen::MatrixXd Jacob;
	Eigen::Vector3d delta_theta = s.omg * delta_t;
	Eigen::VectorXd delta_theta_ = delta_theta;
	delta_R = SO3::exp(delta_theta);
	delta_R.Jacob_right(delta_theta_, Jacob);

	cov.template block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * delta_t;
	cov.template block<3, 3>(6, 3) = -s.rot * MTK::hat(s.acc) * delta_t; // .normalized().toRotationMatrix()
	cov.template block<3, 3>(6, 12) = s.rot * delta_t; //.normalized().toRotationMatrix();
	cov.template block<3, 3>(3, 9) = Jacob * delta_t;
	cov.template block<3, 3>(3, 3) = delta_R.transpose();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	cov.template block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * delta_t; // grav_matrix; 
	return cov;
}

void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	VF(4) pabcd;
	pabcd.setZero();
	normvec->resize(time_seq[k]);
	int effect_num_k = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		PointType &point_body_j  = feats_down_body->points[idx+j+1];
		PointType &point_world_j = feats_down_world->points[idx+j+1];
		pointBodyToWorld(&point_body_j, &point_world_j); 
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
            ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS); // 
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
			{
				point_selected_surf[idx+j+1] = false;
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					
					if (effect_num_k > 0) continue;
					if (p_norm > match_s * pd2 * pd2)
					{
						point_selected_surf[idx+j+1] = true;
						normvec->points[j].x = pabcd(0);
						normvec->points[j].y = pabcd(1);
						normvec->points[j].z = pabcd(2);
						normvec->points[j].intensity = pabcd(3);
						effect_num_k ++;
					}
				}  
			}
		}
	}
	if (effect_num_k == 0) 
	{
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	// double sqrt_laser_noise = sqrt(laser_point_cov);
	ekfom_data.h_x.resize(effect_num_k, 6);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 6); // 12);
	ekfom_data.z.resize(effect_num_k);
	// ekfom_data.z_R.resize(effect_num_k);
	int m = 0;
	// V3D last_norm_vec = V3D::Zero();
	// if (!p_gnss->norm_vec_holder.empty()) 
	// {
	// 	last_norm_vec = p_gnss->norm_vec_holder.back();
	// }
	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
			
			// {   
				M3D point_crossmat = crossmat_list[idx+j+1];
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);

				if (std::fabs(norm_vec(2)) > 0.9) A = A.normalized(); // if (A.norm() > 1.0 && std::fabs(norm_vec(2)) > 0.9) 
				ekfom_data.h_x.block<1, 6>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A); //, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			// }
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			
			m++;
		}
	}
	effct_feat_num += effect_num_k;
	if (GNSS_ENABLE) p_gnss->norm_vec_num += effect_num_k;
	if (NMEA_ENABLE) p_nmea->norm_vec_num += effect_num_k;
}

void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
    std::memset(ekfom_data.satu_check, false, 6);
	ekfom_data.z_IMU.block<3,1>(0, 0) = angvel_avr - s.omg - s.bg;
	ekfom_data.z_IMU.block<3,1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
    ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;
	if(check_satu)
	{
		if(fabs(angvel_avr(0)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[0] = true; 
			ekfom_data.z_IMU(0) = 0.0;
		}
		
		if(fabs(angvel_avr(1)) >= 0.99 * satu_gyro) 
		{
			ekfom_data.satu_check[1] = true;
			ekfom_data.z_IMU(1) = 0.0;
		}
		
		if(fabs(angvel_avr(2)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[2] = true;
			ekfom_data.z_IMU(2) = 0.0;
		}
		
		if(fabs(acc_avr(0)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[3] = true;
			ekfom_data.z_IMU(3) = 0.0;
		}

		if(fabs(acc_avr(1)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[4] = true;
			ekfom_data.z_IMU(4) = 0.0;
		}

		if(fabs(acc_avr(2)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[5] = true;
			ekfom_data.z_IMU(5) = 0.0;
		}
	}
}

void h_model_GNSS_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	Eigen::Matrix3d res_R = s.rot.transpose() * p_gnss->state_const_.rot;
	Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	ekfom_data.h_GNSS.setIdentity();
	// ekfom_data.h_GNSS *= p_gnss->odo_weight;
	// ekfom_data.h_GNSS(0, 0) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(3, 3) = p_gnss->odo_weight4;
	// ekfom_data.h_GNSS(1, 1) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(4, 4) = p_gnss->odo_weight5;
	// ekfom_data.h_GNSS(0, 0) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(1, 1) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(2, 2) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	ekfom_data.h_GNSS.block<3, 3>(3, 3) = 0.1 * Jacob_right_inv<double>(res_r); // 0.1 *
	// ekfom_data.h_GNSS.block<1, 3>(3, 3) *= p_gnss->odo_weight4;
	// ekfom_data.h_GNSS.block<1, 3>(4, 3) *= p_gnss->odo_weight5;
	// ekfom_data.h_GNSS.block<1, 3>(5, 3) *= p_gnss->odo_weight6;
	ekfom_data.z_GNSS.setZero();
	// ekfom_data.h_GNSS.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.h_GNSS.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.z_GNSS.block<3, 1>(3, 0) = res_r;
	// ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->odo_weight * (p_gnss->state_const_.pos - s.pos); // 
	ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->state_const_.pos - s.pos;
	// ekfom_data.z_GNSS(0) *= 1.5;
	// ekfom_data.z_GNSS(1) *= 1.5;
	// ekfom_data.z_GNSS(2) *= 1.5;
	// ekfom_data.z_GNSS(0) = p_gnss->odo_weight1 * (p_gnss->state_const_.pos(0) - s.pos(0)); // 
	// ekfom_data.z_GNSS(1) = p_gnss->odo_weight2 * (p_gnss->state_const_.pos(1) - s.pos(1)); // 
	// ekfom_data.z_GNSS(2) = p_gnss->odo_weight3 * (p_gnss->state_const_.pos(2) - s.pos(2)); // 
	ekfom_data.z_GNSS.block<3, 1>(3, 0) = 0.1 * res_r; // s.rot.transpose() * p_gnss->state_.rot; // 0.1 *
	// ekfom_data.z_GNSS(3) *= p_gnss->odo_weight4; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(4) *= p_gnss->odo_weight5; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(5) *= p_gnss->odo_weight6; // s.rot.transpose() * p_gnss->state_.rot; //  
	// ekfom_data.z_GNSS(3) = p_gnss->odo_weight1 * (p_gnss->state_const_.vel(0) - s.vel(0)); // 
	// ekfom_data.z_GNSS(4) = p_gnss->odo_weight2 * (p_gnss->state_const_.vel(1) - s.vel(1)); // 
	// ekfom_data.z_GNSS(5) = p_gnss->odo_weight3 * (p_gnss->state_const_.vel(2) - s.vel(2)); // 
	// ekfom_data.z_GNSS.block<3, 1>(6, 0) = p_gnss->state_const_.vel - s.vel;
	// double error_1 = abs(ekfom_data.z_GNSS(0)) - sqrt(abs(cov_p(0, 0)));
	// // error_1 *= error_1;
	// double error_2 = abs(ekfom_data.z_GNSS(1)) - sqrt(abs(cov_p(1, 1)));
	// // error_2 *= error_2;
	// double error_3 = abs(ekfom_data.z_GNSS(2)) - sqrt(abs(cov_p(2, 2)));
	// // error_3 *= error_3;
	// double error_4 = abs(ekfom_data.z_GNSS(3)) - sqrt(abs(cov_R(0, 0)));
	// // error_4 *= error_4;
	// double error_5 = abs(ekfom_data.z_GNSS(4)) - sqrt(abs(cov_R(1, 1)));
	// // error_5 *= error_5;
	// double error_6 = abs(ekfom_data.z_GNSS(5)) - sqrt(abs(cov_R(2, 2)));
	// error_6 *= error_6;
	// double gnss_noise_sqrt = sqrt(gnss_ekf_noise);
	// ekfom_data.R_GNSS(0) = gnss_noise_sqrt > error_1? gnss_ekf_noise : error_1 * error_1;
	// ekfom_data.R_GNSS(1) = gnss_noise_sqrt > error_2? gnss_ekf_noise : error_2 * error_2;
	// ekfom_data.R_GNSS(2) = gnss_noise_sqrt > error_3? gnss_ekf_noise : error_3 * error_3;
	// ekfom_data.R_GNSS(3) = gnss_noise_sqrt > error_4? gnss_ekf_noise : error_4 * error_4;
	// ekfom_data.R_GNSS(4) = gnss_noise_sqrt > error_5? gnss_ekf_noise : error_5 * error_5;
	// ekfom_data.R_GNSS(5) = gnss_noise_sqrt > error_6? gnss_ekf_noise : error_6 * error_6;
	// ekfom_data.R_GNSS(0) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(1) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(2) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(3) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(4) = gnss_ekf_noise;
	// ekfom_data.R_GNSS(5) = gnss_ekf_noise;
	// double max_err = error_1 > error_2? error_1 : error_2;
	// max_err = max_err > error_3? max_err : error_3;
	ekfom_data.M_Noise = gnss_ekf_noise; // > max_err? gnss_ekf_noise : max_err;
}

void h_model_NMEA_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	Eigen::Matrix3d res_R = s.rot.transpose() * p_nmea->state_const_.rot;
	Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	ekfom_data.h_NMEA.setIdentity();
	// ekfom_data.h_GNSS *= p_gnss->odo_weight;
	// ekfom_data.h_GNSS(0, 0) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(3, 3) = p_gnss->odo_weight4;
	// ekfom_data.h_GNSS(1, 1) = p_gnss->odo_weight1; // ekfom_data.h_GNSS(4, 4) = p_gnss->odo_weight5;
	// ekfom_data.h_GNSS(0, 0) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(1, 1) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	// ekfom_data.h_GNSS(2, 2) = 1.5; // p_gnss->odo_weight1; // ekfom_data.h_GNSS(5, 5) = p_gnss->odo_weight6;
	ekfom_data.h_NMEA.block<3, 3>(3, 3) = Jacob_right_inv<double>(res_r); // 0.1 *
	// ekfom_data.h_GNSS.block<1, 3>(3, 3) *= p_gnss->odo_weight4;
	// ekfom_data.h_GNSS.block<1, 3>(4, 3) *= p_gnss->odo_weight5;
	// ekfom_data.h_GNSS.block<1, 3>(5, 3) *= p_gnss->odo_weight6;
	ekfom_data.z_NMEA.setZero();
	// ekfom_data.h_GNSS.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.h_GNSS.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); // Jacob_right_inv<double>(res_r); // 
	// ekfom_data.z_GNSS.block<3, 1>(3, 0) = res_r;
	// ekfom_data.z_GNSS.block<3, 1>(0, 0) = p_gnss->odo_weight * (p_gnss->state_const_.pos - s.pos); // 
	ekfom_data.z_NMEA.block<3, 1>(0, 0) = p_nmea->state_const_.pos - s.pos;
	ekfom_data.z_NMEA.block<3, 1>(6, 0) = p_nmea->state_const_.vel - s.vel;
	// ekfom_data.z_GNSS(0) *= 1.5;
	// ekfom_data.z_GNSS(1) *= 1.5;
	// ekfom_data.z_GNSS(2) *= 1.5;
	// ekfom_data.z_GNSS(0) = p_gnss->odo_weight1 * (p_gnss->state_const_.pos(0) - s.pos(0)); // 
	// ekfom_data.z_GNSS(1) = p_gnss->odo_weight2 * (p_gnss->state_const_.pos(1) - s.pos(1)); // 
	// ekfom_data.z_GNSS(2) = p_gnss->odo_weight3 * (p_gnss->state_const_.pos(2) - s.pos(2)); // 
	ekfom_data.z_NMEA.block<3, 1>(3, 0) = res_r;
	// vect3 so3_deg = s.omg * s.time_diff(0);
	// Eigen::Matrix3d rot_const = MTK::SO3<double>::exp(so3_deg);
	// Eigen::Matrix3d res_R = rot_const.transpose() * s.rot.transpose() * p_nmea->state_const_.rot;
	// Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));
	// ekfom_data.h_NMEA.setIdentity();
	// ekfom_data.h_NMEA.block<3, 3>(3, 3) = Jacob_right_inv<double>(res_r) * rot_const.transpose();
	// ekfom_data.h_NMEA.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(0, 9) = s.vel + s.acc * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 3>(0, 13) = 0.5 * Eigen::Matrix3d::Identity() * s.time_diff(0) * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(3, 9) = Jacob_right_inv<double>(res_r) * s.omg;
	// ekfom_data.h_NMEA.block<3, 3>(3, 10) = Jacob_right_inv<double>(res_r) * s.time_diff(0);
	// ekfom_data.h_NMEA.block<3, 1>(6, 9) = s.acc;
	// ekfom_data.h_NMEA.block<3, 3>(6, 13) = Eigen::Matrix3d::Identity() * s.time_diff(0);
	// // ekfom_data.h_NMEA.block<3, 3>(6, 6) = Eigen::Matrix3d::Zero(); Jacob_right_inv<double>(res_r);
	// ekfom_data.z_NMEA.setZero();
	// ekfom_data.z_NMEA.block<3, 1>(0, 0) = p_nmea->state_const_.pos - s.pos - s.vel * s.time_diff - 0.5 * s.acc * s.time_diff * s.time_diff; // 
	// ekfom_data.z_NMEA.block<3, 1>(6, 0) = p_nmea->state_const_.vel - s.vel - s.acc * s.time_diff; // 
	// ekfom_data.z_NMEA.block<3, 1>(3, 0) = res_r; // s.rot.transpose() * p_gnss->state_.rot; //  
	ekfom_data.M_Noise = gnss_ekf_noise;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{    
    V3D p_body(pi->x, pi->y, pi->z);
    
    V3D p_global;
	{
		// if (!use_imu_as_input)
		{
			p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_output.x_.pos; // .normalized()
		}
	}

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}