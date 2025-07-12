/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt-headers.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@file
@brief Uniform cumulative B-spline for SO(3)
*/

#pragma once

#include "../utils/sophus_utils.hpp"
#include "assert.h"
#include "spline_common.h"

#include <Eigen/Dense>
#include <array>
#include <sophus_lib/so3.hpp>

#include <cstdint>
#include <iomanip>

namespace cocolic {

/// @brief Uniform cummulative B-spline for SO(3) of order N
///
/// For example, in the particular case scalar values and order N=5, for a time
/// \f$t \in [t_i, t_{i+1})\f$ the value of \f$p(t)\f$ depends only on 5 control
/// points at \f$[t_i, t_{i+1}, t_{i+2}, t_{i+3}, t_{i+4}]\f$. To
/// simplify calculations we transform time to uniform representation \f$s(t) =
/// (t - t_0)/\Delta t \f$, such that control points transform into \f$ s_i \in
/// [0,..,N] \f$. We define function \f$ u(t) = s(t)-s_i \f$ to be a time since
/// the start of the segment. Following the cummulative matrix representation of
/// De Boor - Cox formula, the value of the function can be evaluated as
/// follows: \f{align}{
///    R(u(t)) &= R_i
///    \prod_{j=1}^{4}{\exp(k_{j}\log{(R_{i+j-1}^{-1}R_{i+j})})},
///    \\ \begin{pmatrix} k_0 \\ k_1 \\ k_2 \\ k_3 \\ k_4 \end{pmatrix}^T &=
///    M_{c5} \begin{pmatrix} 1 \\ u \\ u^2 \\ u^3 \\ u^4
///    \end{pmatrix},
/// \f}
/// where \f$ R_{i} \in SO(3) \f$ are knots and \f$ M_{c5} \f$ is a cummulative
/// blending matrix computed using \ref computeBlendingMatrix \f{align}{
///    M_{c5} = \frac{1}{4!}
///    \begin{pmatrix} 24 & 0 & 0 & 0 & 0 \\ 23 & 4 & -6 & 4 & -1 \\ 12 & 16 & 0
///    & -8 & 3 \\ 1 & 4 & 6 & 4 & -3 \\ 0 & 0 & 0 & 0 & 1 \end{pmatrix}.
/// \f}
///
/// See [[arXiv:1911.08860]](https://arxiv.org/abs/1911.08860) for more details.
template <int _N, typename _Scalar = double>
class So3Spline {
 public:
  static constexpr int N = _N;        ///< Order of the spline.
  static constexpr int DEG = _N - 1;  ///< Degree of the spline.

  using MatN = Eigen::Matrix<_Scalar, _N, _N>;
  using VecN = Eigen::Matrix<_Scalar, _N, 1>;

  using Vec3 = Eigen::Matrix<_Scalar, 3, 1>;
  using Mat3 = Eigen::Matrix<_Scalar, 3, 3>;

  using SO3 = Sophus::SO3<_Scalar>;

  static constexpr _Scalar NS_TO_S = 1e-9;  ///< Nanosecond to second conversion
  static constexpr _Scalar S_TO_NS = 1e9;   ///< Second to nanosecond conversion

  /// @brief Struct to store the Jacobian of the spline
  ///
  /// Since B-spline of order N has local support (only N knots infuence the
  /// value) the Jacobian is zero for all knots except maximum N for value and
  /// all derivatives.
  struct JacobianStruct {
    size_t start_idx;
    std::array<Mat3, _N> d_val_d_knot;
  };

  /// @brief Constructor with knot interval and start time
  ///
  /// @param[in] time_interval knot time interval
  /// @param[in] start_time start time of the spline
  So3Spline(int64_t time_interval_ns, int64_t start_time_ns = 0)
      : dt_ns_(time_interval_ns), start_t_ns_(start_time_ns) {
    pow_inv_dt_[0] = 1.0;
    pow_inv_dt_[1] = S_TO_NS / dt_ns_;
    pow_inv_dt_[2] = pow_inv_dt_[1] * pow_inv_dt_[1];
    pow_inv_dt_[3] = pow_inv_dt_[2] * pow_inv_dt_[1];
  }

  std::pair<double, size_t> computeTIndexNs(int64_t time_ns) const {
    BASALT_ASSERT_STREAM(time_ns >= start_t_ns_, " timestamp  " << time_ns
                                                                << " start_t "
                                                                << start_t_ns_);
    int64_t st_ns = (time_ns - start_t_ns_);
    int64_t s = st_ns / dt_ns_;
    double u = double(st_ns % dt_ns_) / double(dt_ns_);

    BASALT_ASSERT_STREAM(s >= 0, "s " << s);
    BASALT_ASSERT_STREAM(size_t(s + N) <= knots.size(),
                         "s " << s << " N " << N << " knots.size() "
                              << knots.size() << " timestamp  " << time_ns
                              << " start_t " << start_t_ns_ << " maxTimeNs "
                              << maxTimeNs());
    return std::make_pair(u, s);
  }

  int64_t maxTimeNs() const {
    return start_t_ns_ + (knots.size() - N + 1) * dt_ns_ - 1;
  }

  /// @brief Minimum time represented by spline
  ///
  /// @return minimum time represented by spline in nanoseconds
  int64_t minTimeNs() const { return start_t_ns_; }

  /// @brief Gererate random trajectory
  ///
  /// @param[in] n number of knots to generate
  /// @param[in] static_init if true the first N knots will be the same
  /// resulting in static initial condition
  void genRandomTrajectory(int n, bool static_init = false) {
    if (static_init) {
      Vec3 rnd = Vec3::Random() * M_PI;
      for (int i = 0; i < N; i++) knots.push_back(SO3::exp(rnd));
      for (int i = 0; i < n - N; i++)
        knots.push_back(SO3::exp(Vec3::Random() * M_PI));
    } else {
      for (int i = 0; i < n; i++)
        knots.push_back(SO3::exp(Vec3::Random() * M_PI));
    }
  }

  // void genRandomKnot( double mean_x, double sigma_x,
  //                              double mean_y, double sigma_y,
  //                              double mean_z, double sigma_z) {
  //     Vec3 random_vec;

  //     static std::random_device rd;
  //     static std::default_random_engine generator_(rd());
  //     static std::normal_distribution<double> noise_x(mean_x, sigma_x);
  //     static std::normal_distribution<double> noise_y(mean_y, sigma_y);
  //     static std::normal_distribution<double> noise_z(mean_z, sigma_z);

  //     static std::random_device rd_amp;
  //     static std::default_random_engine generator_amp_(rd_amp());
      
  //     knots.push_back(SO3::exp(random_vec * M_PI));
  // }

  /// @brief Set start time for spline
  ///
  /// @param[in] start_time start time of the spline
  inline void setStartTimeNs(int64_t start_time_ns) {
    start_t_ns_ = start_time_ns;
  }

  /// @brief Add knot to the end of the spline
  ///
  /// @param[in] knot knot to add
  inline void knots_push_back(const SO3& knot) { knots.push_back(knot); }

  /// @brief Remove knot from the back of the spline
  inline void knots_pop_back() { knots.pop_back(); }

  /// @brief Return the first knot of the spline
  ///
  /// @return first knot of the spline
  inline const SO3& knots_front() const { return knots.front(); }

  /// @brief Remove first knot of the spline and increase the start time
  inline void knots_pop_front() {
    start_t_ns_ += dt_ns_;
    knots.pop_front();
  }

  /// @brief Resize containter with knots
  ///
  /// @param[in] n number of knots
  inline void resize(size_t n) { knots.resize(n); }

  /// @brief Return reference to the knot with index i
  ///
  /// @param i index of the knot
  /// @return reference to the knot
  inline SO3& getKnot(int i) { return knots[i]; }

  /// @brief Return const reference to the knot with index i
  ///
  /// @param i index of the knot
  /// @return const reference to the knot
  inline const SO3& getKnot(int i) const { return knots[i]; }

  /// @brief Return const reference to deque with knots
  ///
  /// @return const reference to deque with knots
  const Eigen::aligned_deque<SO3>& getKnots() const { return knots; }

  /// @brief Return time interval
  ///
  /// @return time interval
  int64_t getTimeIntervalNs() const { return dt_ns_; }

  /// @brief Evaluate SO(3) B-spline
  ///
  /// @param[in] time_ time for evaluating the value of the spline
  /// @param[out] J if not nullptr, return the Jacobian of the value with
  /// respect to knots
  /// @return SO(3) value of the spline
  SO3 evaluate(int64_t time_ns, JacobianStruct* J = nullptr) const {
    std::pair<double, size_t> ui = computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blending_matrix_ * p;

    SO3 res = knots[s];

    Mat3 J_helper;

    if (J) {
      J->start_idx = s;
      J_helper.setIdentity();
    }

    for (int i = 0; i < DEG; i++) {
      const SO3& p0 = knots[s + i];
      const SO3& p1 = knots[s + i + 1];

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();
      Vec3 kdelta = delta * coeff[i + 1];

      if (J) {
        Mat3 Jl_inv_delta, Jl_k_delta;

        Sophus::leftJacobianInvSO3(delta, Jl_inv_delta);
        Sophus::leftJacobianSO3(kdelta, Jl_k_delta);

        J->d_val_d_knot[i] = J_helper;
        J_helper = coeff[i + 1] * res.matrix() * Jl_k_delta * Jl_inv_delta *
                   p0.inverse().matrix();
        J->d_val_d_knot[i] -= J_helper;
      }
      res *= SO3::exp(kdelta);
    }

    if (J) J->d_val_d_knot[DEG] = J_helper;

    return res;
  }

  SO3 evaluateNURBS(size_t s, double u, 
                                              const Eigen::Matrix4d& blend_mat) const {
    // std::pair<double, size_t> ui = computeTIndex(time);
    // size_t s = ui.second;
    // double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blend_mat * p;

    SO3 res = knots[s];

    for (int i = 0; i < DEG; i++) 
    {
      const SO3& p0 = knots[s + i];
      const SO3& p1 = knots[s + i + 1];

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();
      Vec3 kdelta = delta * coeff[i + 1];

      res *= SO3::exp(kdelta);
    }

    return res;
  }
  
  Vec3 velocityBody(int64_t time_ns) const {
    std::pair<double, size_t> ui = computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);
    VecN coeff = blending_matrix_ * p;

    baseCoeffsWithTime<1>(p, u);
    VecN dcoeff = pow_inv_dt_[1] * blending_matrix_ * p;

    Vec3 rot_vel;
    rot_vel.setZero();

    for (int i = 0; i < DEG; i++) {
      const SO3& p0 = knots[s + i];
      const SO3& p1 = knots[s + i + 1];

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();

      rot_vel = SO3::exp(-delta * coeff[i + 1]) * rot_vel;
      rot_vel += delta * dcoeff[i + 1];
    }

    return rot_vel;
  }

  Vec3 velocityBodyNURBS(size_t s, double u, double delta_t,
                                              const Eigen::Matrix4d& blend_mat) const {
    // std::pair<double, size_t> ui = computeTIndexNs(time_ns);
    // size_t s = ui.second;
    // double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);
    VecN coeff = blend_mat * p;

    baseCoeffsWithTime<1>(p, u);
    VecN dcoeff = 1.0 / delta_t * blend_mat * p;

    Vec3 rot_vel;
    rot_vel.setZero();

    for (int i = 0; i < DEG; i++) {
      const SO3& p0 = knots[s + i];
      const SO3& p1 = knots[s + i + 1];

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();

      rot_vel = SO3::exp(-delta * coeff[i + 1]) * rot_vel;
      rot_vel += delta * dcoeff[i + 1];
    }

    return rot_vel;
  }

  Vec3 accelerationBody(int64_t time_ns, Vec3* vel_body = nullptr) const {
    std::pair<double, size_t> ui = computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);
    VecN coeff = blending_matrix_ * p;

    baseCoeffsWithTime<1>(p, u);
    VecN dcoeff = pow_inv_dt_[1] * blending_matrix_ * p;

    baseCoeffsWithTime<2>(p, u);
    VecN ddcoeff = pow_inv_dt_[2] * blending_matrix_ * p;

    SO3 r_accum;

    Vec3 rot_vel;
    rot_vel.setZero();

    Vec3 rot_accel;
    rot_accel.setZero();

    for (int i = 0; i < DEG; i++) {
      const SO3& p0 = knots[s + i];
      const SO3& p1 = knots[s + i + 1];

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();

      SO3 rot = SO3::exp(-delta * coeff[i + 1]);

      rot_vel = rot * rot_vel;
      Vec3 vel_current = dcoeff[i + 1] * delta;
      rot_vel += vel_current;

      rot_accel = rot * rot_accel;
      rot_accel += ddcoeff[i + 1] * delta + rot_vel.cross(vel_current);
    }

    if (vel_body) *vel_body = rot_vel;
    return rot_accel;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  /// @brief Vector of derivatives of time polynomial.
  ///
  /// Computes a derivative of \f$ \begin{bmatrix}1 & t & t^2 & \dots &
  /// t^{N-1}\end{bmatrix} \f$ with repect to time. For example, the first
  /// derivative would be \f$ \begin{bmatrix}0 & 1 & 2 t & \dots & (N-1)
  /// t^{N-2}\end{bmatrix} \f$.
  /// @param Derivative derivative to evaluate
  /// @param[out] res_const vector to store the result
  /// @param[in] t
  template <int Derivative, class Derived>
  static void baseCoeffsWithTime(const Eigen::MatrixBase<Derived>& res_const,
                                 _Scalar t) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
    Eigen::MatrixBase<Derived>& res =
        const_cast<Eigen::MatrixBase<Derived>&>(res_const);

    res.setZero();

    if (Derivative < N) {
      res[Derivative] = base_coefficients_(Derivative, Derivative);

      _Scalar _t = t;
      for (int j = Derivative + 1; j < N; j++) {
        res[j] = base_coefficients_(Derivative, j) * _t;
        _t = _t * t;
      }
    }
  }

  ///< Blending matrix. See \ref computeBlendingMatrix.
  static const MatN blending_matrix_;

  static const MatN base_coefficients_;  ///< Base coefficients matrix.
  ///< See \ref computeBaseCoefficients.

  Eigen::aligned_deque<SO3> knots;     ///< Knots
  int64_t dt_ns_;                      ///< Knot interval in nanoseconds
  int64_t start_t_ns_;                 ///< Start time in nanoseconds
  std::array<_Scalar, 4> pow_inv_dt_;  ///< Array with inverse powers of dt

};  // namespace basalt

template <int _N, typename _Scalar>
const typename So3Spline<_N, _Scalar>::MatN
    So3Spline<_N, _Scalar>::base_coefficients_ =
        computeBaseCoefficients<_N, _Scalar>();

template <int _N, typename _Scalar>
const typename So3Spline<_N, _Scalar>::MatN
    So3Spline<_N, _Scalar>::blending_matrix_ =
        computeBlendingMatrix<_N, _Scalar, true>();

}  // namespace cocolic
