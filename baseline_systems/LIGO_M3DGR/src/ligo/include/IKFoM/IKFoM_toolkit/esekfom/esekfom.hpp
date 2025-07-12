/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
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

#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP


#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

#include "../mtk/types/vect.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/types/SEn.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/build_manifold.hpp"
#include "util.hpp"

namespace esekfom {

using namespace Eigen;

template<typename T>
struct dyn_share_modified
{
	bool valid;
	bool converge;
	T M_Noise;
	Eigen::Matrix<T, Eigen::Dynamic, 1> z;
	// Eigen::Matrix<T, Eigen::Dynamic, 1> z_R;
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
	Eigen::Matrix<T, 6, 1> z_IMU;
	Eigen::Matrix<T, 6, 1> z_GNSS;
	Eigen::Matrix<T, 9, 1> z_NMEA;
	Eigen::Matrix<T, 6, 1> R_IMU;
	// Eigen::Matrix<T, 6, 1> R_GNSS;
	Eigen::Matrix<T, 6, 6> h_GNSS;
	Eigen::Matrix<T, 9, 9> h_NMEA;
	bool satu_check[6];
};

template<typename state, int process_noise_dof, typename input = state, typename measurement=state, int measurement_noise_dof=0>
class esekf{

	typedef esekf self;
	enum{
		n = state::DOF, m = state::DIM, l = measurement::DOF
	};

public:
	
	typedef typename state::scalar scalar_type;
	typedef Matrix<scalar_type, n, n> cov;
	typedef Matrix<scalar_type, m, n> cov_;
	typedef SparseMatrix<scalar_type> spMt;
	typedef Matrix<scalar_type, n, 1> vectorized_state;
	typedef Matrix<scalar_type, m, 1> flatted_state;
	typedef flatted_state processModel(state &, const input &);
	typedef Eigen::Matrix<scalar_type, n, n> processMatrix1(state &, const input &, double dt);
	typedef Eigen::Matrix<scalar_type, m, process_noise_dof> processMatrix2(state &, const input &);
	typedef Eigen::Matrix<scalar_type, process_noise_dof, process_noise_dof> processnoisecovariance;

	typedef void measurementModel_dyn_share_modified_cov(state &, Eigen::Matrix3d, Eigen::Matrix3d, dyn_share_modified<scalar_type> &);
	typedef void measurementModel_dyn_share_modified(state &, dyn_share_modified<scalar_type> &);
	typedef Eigen::Matrix<scalar_type ,l, n> measurementMatrix1(state &);
	typedef Eigen::Matrix<scalar_type , Eigen::Dynamic, n> measurementMatrix1_dyn(state &);
	typedef Eigen::Matrix<scalar_type ,l, measurement_noise_dof> measurementMatrix2(state &);
	typedef Eigen::Matrix<scalar_type ,Eigen::Dynamic, Eigen::Dynamic> measurementMatrix2_dyn(state &);
	typedef Eigen::Matrix<scalar_type, measurement_noise_dof, measurement_noise_dof> measurementnoisecovariance;
	typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> measurementnoisecovariance_dyn;

	esekf(const state &x = state(),
		const cov  &P = cov::Identity()): x_(x), P_(P){};

	void init_dyn_share_modified_2h(processModel f_in, processMatrix1 f_x_in, measurementModel_dyn_share_modified_cov h_dyn_share_in1, measurementModel_dyn_share_modified_cov h_dyn_share_in3)
	{
		f = f_in;
		f_x = f_x_in;
		// f_w = f_w_in;
		h_dyn_share_modified_1 = h_dyn_share_in1;
		h_dyn_share_modified_3 = h_dyn_share_in3;
		maximum_iter = 1;
		x_.build_S2_state();
		x_.build_SO3_state();
		x_.build_vect_state();
		x_.build_SEN_state();
	}
	
	void init_dyn_share_modified_3h(processModel f_in, processMatrix1 f_x_in, measurementModel_dyn_share_modified_cov h_dyn_share_in1, measurementModel_dyn_share_modified h_dyn_share_in2,
	 measurementModel_dyn_share_modified_cov h_dyn_share_in3)
	{
		f = f_in;
		f_x = f_x_in;
		// f_w = f_w_in;
		h_dyn_share_modified_1 = h_dyn_share_in1;
		h_dyn_share_modified_2 = h_dyn_share_in2;
		h_dyn_share_modified_3 = h_dyn_share_in3;
		maximum_iter = 1;
		x_.build_S2_state();
		x_.build_SO3_state();
		x_.build_vect_state();
		x_.build_SEN_state();
	}

	// iterated error state EKF propogation
	void predict(double &dt, processnoisecovariance &Q, const input &i_in, bool predict_state, bool prop_cov){
		if (predict_state)
		{
			flatted_state f_ = f(x_, i_in);
			x_.oplus(f_, dt);
		}

		if (prop_cov)
		{
			cov f_x_ = f_x(x_, i_in, dt);
			P_ = f_x_ * P_ * (f_x_).transpose() + Q * (dt * dt);
		}
	}

	bool update_iterated_dyn_share_modified() {
		dyn_share_modified<scalar_type> dyn_share;
		state x_propagated = x_;
		int dof_Measurement;
		double m_noise;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_1(x_, P_.template block<3, 3>(0, 0), P_. template block<3, 3>(3, 3), dyn_share);
			if(! dyn_share.valid)
			{
				return false;
				// continue;
			}
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> z = dyn_share.z;
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x = dyn_share.h_x;
			dof_Measurement = h_x.rows();
			m_noise = dyn_share.M_Noise;

			Matrix<scalar_type, n, Eigen::Dynamic> PHT;
			Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> HPHT;
			Matrix<scalar_type, n, Eigen::Dynamic> K_;
			if(n > dof_Measurement)
			{
				PHT = P_. template block<n, 6>(0, 0) * h_x.transpose();
				HPHT = h_x * PHT.topRows(6);
				for (int m = 0; m < dof_Measurement; m++)
				{
					HPHT(m, m) += m_noise; // dyn_share.m_noise;
				}
				K_= PHT*HPHT.inverse();
			}
			else
			{
				Matrix<scalar_type, 6, 6> HTH = m_noise * h_x.transpose() * h_x; // 
				Matrix<scalar_type, n, n> P_inv = P_.inverse();
				P_inv.template block<6, 6>(0, 0) += HTH;
				P_inv = P_inv.inverse();
				K_ = P_inv.template block<n, 6>(0, 0) * h_x.transpose() * m_noise;
			}
			Matrix<scalar_type, n, 1> dx_ = K_ * z; // - h) + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new; 
			// state x_before = x_;
			// Eigen::Matrix3d L_ = Eigen::Matrix3d::Identity();
			// {
			// 	// // L_.setIdentity();
			// 	Eigen::MatrixXd RJ;
			// 	Eigen::VectorXd JV = dx_.template block<3, 1>(3, 0);
			// 	x_.rot.Jacob_right(JV, RJ);
			// 	L_ = RJ; //.template block<1, 2>(0, 1) = Rr.transpose() * LJ * B_new_r;
			// 	// x_.r.boxplus(dx_.template block<2, 1>(0, 0)); 
			// }
			x_.boxplus(dx_);
			
			{
				P_ = P_ - K_*h_x*P_. template block<6, n>(0, 0);
			}

			// P_.template block<3, n>(3, 0) = L_ * P_.template block<3, n>(3, 0);
			// P_.template block<n, 3>(0, 3) = P_.template block<n, 3>(0, 3) * L_.transpose();		
		}
		return true;
	}
	
	void update_iterated_dyn_share_IMU() {
		
		dyn_share_modified<scalar_type> dyn_share;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_2(x_, dyn_share);

			Matrix<scalar_type, 6, 1> z = dyn_share.z_IMU;

			Matrix<double, 24, 6> PHT;
            Matrix<double, 6, 24> HP;
            Matrix<double, 6, 6> HPHT;
			PHT.setZero();
			HP.setZero();
			HPHT.setZero();
			for (int l_ = 0; l_ < 6; l_++)
			{
				if (!dyn_share.satu_check[l_])
				{
					PHT.col(l_) = P_.col(9+l_) + P_.col(18+l_);
					HP.row(l_) = P_.row(9+l_) + P_.row(18+l_);
				}
			}
			for (int l_ = 0; l_ < 6; l_++)
			{
				if (!dyn_share.satu_check[l_])
				{
					HPHT.col(l_) = HP.col(9+l_) + HP.col(18+l_);
				}
				HPHT(l_, l_) += dyn_share.R_IMU(l_); //, l);
			}
        	Eigen::Matrix<double, 24, 6> K = PHT * HPHT.inverse(); 
                                    
            Matrix<scalar_type, n, 1> dx_ = K * z; 

            P_ -= K * HP;

			// Eigen::Matrix3d L_ = Eigen::Matrix3d::Identity();
			// {
			// 	Eigen::MatrixXd RJ;
			// 	Eigen::VectorXd JV = dx_.template block<3, 1>(3, 0);
			// 	x_.rot.Jacob_right(JV, RJ);
			// 	L_ = RJ; //.template block<1, 2>(0, 1) = Rr.transpose() * LJ * B_new_r;
			// }

			x_.boxplus(dx_);

			// P_.template block<3, n>(3, 0) = L_ * P_.template block<3, n>(3, 0);
			// P_.template block<n, 3>(0, 3) = P_.template block<n, 3>(0, 3) * L_.transpose();
		}
		return;
	}
	
	void update_iterated_dyn_share_GNSS() {
		
		dyn_share_modified<scalar_type> dyn_share;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_3(x_, P_.template block<3, 3>(0, 0), P_. template block<3, 3>(3, 3), dyn_share);

			// Matrix<scalar_type, 9, 1> z = dyn_share.z_GNSS;
		
			// Matrix<double, 6, 6> Hsub_T = dyn_share.h_GNSS.transpose();
			Matrix<double, n, 1> solution;
			Matrix<double, n, 6> PHT;
			Matrix<double, n, 6> K;
			Matrix<double, 6, 6> HPHT;
			Matrix<double, n, 6> KH;
			// KH.setZero();
			
			{
				PHT.template block<n, 1>(0, 0) = P_.template block<n, 1>(0, 0) * dyn_share.h_GNSS(0, 0); // Hsub_T;
				PHT.template block<n, 1>(0, 1) = P_.template block<n, 1>(0, 1) * dyn_share.h_GNSS(1, 1); // Hsub_T;
				PHT.template block<n, 1>(0, 2) = P_.template block<n, 1>(0, 2) * dyn_share.h_GNSS(2, 2); // Hsub_T;
				PHT.template block<n, 3>(0, 3) = P_.template block<n, 3>(0, 3) * dyn_share.h_GNSS.template block<3, 3>(3, 3).transpose();
				// PHT.template block<n, 1>(0, 3) = P_.template block<n, 1>(0, 6) * dyn_share.h_GNSS(3, 3); // Hsub_T;
				// PHT.template block<n, 1>(0, 4) = P_.template block<n, 1>(0, 7) * dyn_share.h_GNSS(4, 4); // Hsub_T;
				// PHT.template block<n, 1>(0, 5) = P_.template block<n, 1>(0, 8) * dyn_share.h_GNSS(5, 5); // Hsub_T;
				// HPHT = dyn_share.h_GNSS * PHT.template block<3, 3>(0, 0); // 
				HPHT.template block<1, 6>(0, 0) = PHT.template block<1, 6>(0, 0) * dyn_share.h_GNSS(0, 0);
				HPHT.template block<1, 6>(1, 0) = PHT.template block<1, 6>(1, 0) * dyn_share.h_GNSS(1, 1);
				HPHT.template block<1, 6>(2, 0) = PHT.template block<1, 6>(2, 0) * dyn_share.h_GNSS(2, 2);
				HPHT.template block<3, 6>(3, 0) = dyn_share.h_GNSS.template block<3, 3>(3, 3) * PHT.template block<3, 6>(3, 0);
				// HPHT.template block<1, 6>(3, 0) = PHT.template block<1, 6>(6, 0) * dyn_share.h_GNSS(3, 3);
				// HPHT.template block<1, 6>(4, 0) = PHT.template block<1, 6>(7, 0) * dyn_share.h_GNSS(4, 4);
				// HPHT.template block<1, 6>(5, 0) = PHT.template block<1, 6>(8, 0) * dyn_share.h_GNSS(5, 5);
				for (int m_ = 0; m_ < 6; m_++)
				{
					HPHT(m_,m_) += dyn_share.M_Noise; // R_GNSS(m_); // 
				}
				K = PHT * HPHT.inverse();
				// KH = K * dyn_share.h_GNSS;
				KH.template block<n, 1>(0, 0) = K.template block<n, 1>(0, 0) * dyn_share.h_GNSS(0, 0);
				KH.template block<n, 1>(0, 1) = K.template block<n, 1>(0, 1) * dyn_share.h_GNSS(1, 1);
				KH.template block<n, 1>(0, 2) = K.template block<n, 1>(0, 2) * dyn_share.h_GNSS(2, 2);
				KH.template block<n, 3>(0, 3) = K.template block<n, 3>(0, 3) * dyn_share.h_GNSS.template block<3, 3>(3, 3);
				// KH.template block<n, 1>(0, 6) = K.template block<n, 1>(0, 3) * dyn_share.h_GNSS(3, 3);
				// KH.template block<n, 1>(0, 7) = K.template block<n, 1>(0, 4) * dyn_share.h_GNSS(4, 4);
				// KH.template block<n, 1>(0, 8) = K.template block<n, 1>(0, 5) * dyn_share.h_GNSS(5, 5);
			}
                                    
            Matrix<scalar_type, n, 1> dx_ = K * dyn_share.z_GNSS;

			// Eigen::Matrix3d L_ = Eigen::Matrix3d::Identity();
			// {
			// 	Eigen::MatrixXd RJ;
			// 	Eigen::VectorXd JV = dx_.template block<3, 1>(3, 0);
			// 	x_.rot.Jacob_right(JV, RJ);
			// 	L_ = RJ; //.template block<1, 2>(0, 1) = Rr.transpose() * LJ * B_new_r;
			// }

			x_.boxplus(dx_);

			P_ = P_ - KH * P_.template block<6, n>(0, 0);
			// x_.boxplus(dx_);
			// P_.template block<3, n>(3, 0) = L_ * P_.template block<3, n>(3, 0);
			// P_.template block<n, 3>(0, 3) = P_.template block<n, 3>(0, 3) * L_.transpose();
            
		}
		
		return;
	}
		
	void update_iterated_dyn_share_NMEA() {
		
		dyn_share_modified<scalar_type> dyn_share;
		for(int i=0; i<maximum_iter; i++)
		{
			dyn_share.valid = true;
			h_dyn_share_modified_3(x_, P_.template block<3, 3>(0, 0), P_. template block<3, 3>(3, 3), dyn_share);

			// Matrix<scalar_type, 9, 1> z = dyn_share.z_GNSS;
		
			// Matrix<double, 6, 6> Hsub_T = dyn_share.h_GNSS.transpose();
			Matrix<double, n, 1> solution;
			Matrix<double, n, 9> PHT;
			Matrix<double, n, 9> K;
			Matrix<double, 9, 9> HPHT;
			Matrix<double, n, 9> KH;
			// KH.setZero();
			
			{
				// PHT.template block<n, 1>(0, 0) = P_.template block<n, 1>(0, 0); // * dyn_share.h_GNSS(0, 0); // Hsub_T;
				// PHT.template block<n, 1>(0, 1) = P_.template block<n, 1>(0, 1); // * dyn_share.h_GNSS(1, 1); // Hsub_T;
				// PHT.template block<n, 1>(0, 2) = P_.template block<n, 1>(0, 2); // * dyn_share.h_GNSS(2, 2); // Hsub_T;
				// PHT.template block<n, 3>(0, 3) = P_.template block<n, 3>(0, 3) * dyn_share.h_NMEA.template block<3, 3>(3, 3).transpose();
				// PHT.template block<n, 1>(0, 6) = P_.template block<n, 1>(0, 6); // * dyn_share.h_GNSS(0, 0); // Hsub_T;
				// PHT.template block<n, 1>(0, 7) = P_.template block<n, 1>(0, 7); // * dyn_share.h_GNSS(1, 1); // Hsub_T;
				// PHT.template block<n, 1>(0, 8) = P_.template block<n, 1>(0, 8); // * dyn_share.h_GNSS(2, 2); // Hsub_T;
				PHT = P_.template block<n, 9>(0, 0) * dyn_share.h_NMEA.transpose();
				// HPHT.template block<1, 9>(0, 0) = PHT.template block<1, 9>(0, 0); // * dyn_share.h_GNSS(0, 0);
				// HPHT.template block<1, 9>(1, 0) = PHT.template block<1, 9>(1, 0); // * dyn_share.h_GNSS(1, 1);
				// HPHT.template block<1, 9>(2, 0) = PHT.template block<1, 9>(2, 0); // * dyn_share.h_GNSS(2, 2);
				// HPHT.template block<3, 9>(3, 0) = dyn_share.h_NMEA.template block<3, 3>(3, 3) * PHT.template block<3, 9>(3, 0);
				// HPHT.template block<1, 9>(6, 0) = PHT.template block<1, 9>(6, 0); // * dyn_share.h_GNSS(0, 0);
				// HPHT.template block<1, 9>(7, 0) = PHT.template block<1, 9>(7, 0); // * dyn_share.h_GNSS(1, 1);
				// HPHT.template block<1, 9>(8, 0) = PHT.template block<1, 9>(8, 0); // * dyn_share.h_GNSS(2, 2);
				HPHT = dyn_share.h_NMEA * PHT.template block<9, 9>(0, 0);
				for (int m_ = 0; m_ < 9; m_++)
				{
					HPHT(m_,m_) += dyn_share.M_Noise;
				}
				K = PHT * HPHT.inverse();
				// KH = K * dyn_share.h_GNSS;
				// KH.template block<n, 1>(0, 0) = K.template block<n, 1>(0, 0); // * dyn_share.h_GNSS(0, 0);
				// KH.template block<n, 1>(0, 1) = K.template block<n, 1>(0, 1); // * dyn_share.h_GNSS(1, 1);
				// KH.template block<n, 1>(0, 2) = K.template block<n, 1>(0, 2); // * dyn_share.h_GNSS(2, 2);
				// KH.template block<n, 3>(0, 3) = K.template block<n, 3>(0, 3) * dyn_share.h_NMEA.template block<3, 3>(3, 3);
				// KH.template block<n, 1>(0, 6) = K.template block<n, 1>(0, 6); // * dyn_share.h_GNSS(0, 0);
				// KH.template block<n, 1>(0, 7) = K.template block<n, 1>(0, 7); // * dyn_share.h_GNSS(1, 1);
				// KH.template block<n, 1>(0, 8) = K.template block<n, 1>(0, 8); // * dyn_share.h_GNSS(2, 2);
				KH = K * dyn_share.h_NMEA;
			}
			
                                    
            Matrix<scalar_type, n, 1> dx_ = K * dyn_share.z_NMEA;

            P_ = P_ - KH * P_.template block<9, n>(0, 0);
			x_.boxplus(dx_);
		}
		
		return;
	}
	
	void change_x(state &input_state)
	{
		x_ = input_state;

		if((!x_.vect_state.size())&&(!x_.SO3_state.size())&&(!x_.S2_state.size())&&(!x_.SEN_state.size()))
		{
			x_.build_S2_state();
			x_.build_SO3_state();
			x_.build_vect_state();
			x_.build_SEN_state();
		}
	}

	void change_P(cov &input_cov)
	{
		P_ = input_cov;
	}

	const state& get_x() const {
		return x_;
	}
	const cov& get_P() const {
		return P_;
	}
	cov P_;
	state x_;
private:
	measurement m_;
	spMt l_;
	spMt f_x_1;
	spMt f_x_2;
	cov F_x1 = cov::Identity();
	cov F_x2 = cov::Identity();
	cov L_ = cov::Identity();

	processModel *f;
	processMatrix1 *f_x;
	processMatrix2 *f_w;

	measurementMatrix1 *h_x;
	measurementMatrix2 *h_v;

	measurementMatrix1_dyn *h_x_dyn;
	measurementMatrix2_dyn *h_v_dyn;

	measurementModel_dyn_share_modified_cov *h_dyn_share_modified_1;

	measurementModel_dyn_share_modified *h_dyn_share_modified_2;

	measurementModel_dyn_share_modified_cov *h_dyn_share_modified_3;

	int maximum_iter = 0;
	scalar_type limit[n];
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace esekfom

#endif //  ESEKFOM_EKF_HPP
