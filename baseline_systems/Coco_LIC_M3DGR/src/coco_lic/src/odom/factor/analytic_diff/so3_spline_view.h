/*
 * Coco-LIC: Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <spline/spline_common.h>
#include <utils/sophus_utils.hpp>

#include <Eigen/Dense>
#include <sophus_lib/so3.hpp>

#include <array>

namespace cocolic {
namespace analytic_derivative {

class So3SplineView {
 public:
  static constexpr int N = SplineOrder;  ///< Order of the spline.
  static constexpr int DEG = N - 1;      // Degree of the spline.

  using MatN = Eigen::Matrix<double, N, N>;
  using VecN = Eigen::Matrix<double, N, 1>;
  using Vec3 = Eigen::Matrix<double, 3, 1>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;

  using SO3 = Sophus::SO3<double>;

  /// @brief Struct to store the Jacobian of the spline
  ///
  /// Since B-spline of order N has local support (only N knots infuence the
  /// value) the Jacobian is zero for all knots except maximum N for value and
  /// all derivatives.
  struct JacobianStruct {
    size_t start_idx;
    std::array<Mat3, N> d_val_d_knot;
  };

  So3SplineView() {}

  /// @brief Evaluate SO(3) B-spline
  ///
  /// R(t)
  ///
  /// @param[in] time_ns for evaluating the value of the spline
  /// @param[out] J if not nullptr, return the Jacobian of the value with
  /// respect to knots
  /// @return SO(3) value of the spline
  static SO3 EvaluateRotation(const int64_t time_ns,
                              const SplineSegmentMeta<N>& splne_meta,
                              double const* const* knots,
                              JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blending_matrix_ * p;

    Eigen::Map<SO3 const> p0(knots[s]);
    SO3 res = p0;

    Mat3 J_helper;

    if (J) {
      J->start_idx = s;
      J_helper = res.matrix();  // 
    }

    Mat3 R_tmp[DEG];
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];
    for (int i = 0; i < DEG; i++) {
      Eigen::Map<SO3 const> p0(knots[s + i]);
      Eigen::Map<SO3 const> p1(knots[s + i + 1]);

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();
      Vec3 kdelta = delta * coeff[i + 1];

      res *= SO3::exp(kdelta);

      Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
      Sophus::rightJacobianSO3(kdelta, Jr_kdelta[i]);
      R_tmp[i] = res.matrix();
    }

    if (J) {
      /// d_val_d_knot 
      J->d_val_d_knot[0] = J_helper;

      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * R_tmp[i] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  static SO3 EvaluateRotationNURBS(const std::pair<int, double>& su,
                              const Eigen::Matrix4d& cumu_blending_matrix,
                              double const* const* knots,
                              JacobianStruct* J = nullptr) {
    size_t s = 0;
    double u = su.second;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = cumu_blending_matrix * p;

    Eigen::Map<SO3 const> p0(knots[s]);
    SO3 res = p0;

    Mat3 J_helper;

    if (J) {
      J->start_idx = s;
      J_helper = res.matrix();  // 
    }

    Mat3 R_tmp[DEG];
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];
    for (int i = 0; i < DEG; i++) {
      Eigen::Map<SO3 const> p0(knots[s + i]);
      Eigen::Map<SO3 const> p1(knots[s + i + 1]);

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();
      Vec3 kdelta = delta * coeff[i + 1];

      res *= SO3::exp(kdelta);

      Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
      Sophus::rightJacobianSO3(kdelta, Jr_kdelta[i]);
      R_tmp[i] = res.matrix();
    }

    if (J) {
      for (int i = 0; i < N; i++) {
        J->d_val_d_knot[i].setZero();
      }

      /// d_val_d_knot 
      J->d_val_d_knot[0] = J_helper;

      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * R_tmp[i] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  /**
   * @brief   (1) r = Lhs * R(t) * rhs
   *          (2) r = Log( Lhs * R(t) * Rhs )
   * Jacobians in J are omitted left multiply term:
   *  (1) -Lhs * R(t) * (rhs)_{\wedge}
   *  (1) Jr^{-1}(r)
   * @return  R(t)
   * */
  static SO3 EvaluateRp(const int64_t time_ns,
                        const SplineSegmentMeta<N>& splne_meta,
                        double const* const* knots,
                        // const Vec3& rhs,
                        JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blending_matrix_ * p;

    if (J) {
      J->start_idx = s;
    }

    SO3 A_accum_inv;           // A_3_inv * A2_inv * A1_inv
    Mat3 A_post_inv[DEG + 1];  // A_3_inv*A2_inv*A1_inv,A_3_inv*A2_inv,A_3_inv,I
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];

    A_post_inv[DEG] = A_accum_inv.matrix();  // Identity Matrix

    for (int i = DEG - 1; i >= 0; i--) {
      Eigen::Map<SO3 const> R0(knots[s + i]);
      Eigen::Map<SO3 const> R1(knots[s + i + 1]);

      Vec3 delta = (R0.inverse() * R1).log();
      Vec3 kdelta = delta * coeff[i + 1];

      A_accum_inv *= SO3::exp(-kdelta);

      if (J) {
        Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
        Sophus::rightJacobianSO3(kdelta, Jr_kdelta[i]);
        A_post_inv[i] = A_accum_inv.matrix();
      }
    }

    Eigen::Map<SO3 const> Ri(knots[s]);
    SO3 res = Ri * A_accum_inv.inverse();

    if (J) {
      Mat3 J_helper = A_post_inv[0];
      J->d_val_d_knot[0] = J_helper;

      /// d_val_d_knot 
      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * A_post_inv[i + 1] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  static SO3 EvaluateRpNURBS(const std::pair<int, double>& su,
                        const Eigen::Matrix4d& cumu_blending_matrix,
                        double const* const* knots,
                        JacobianStruct* J = nullptr) {
    double u = su.second;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = cumu_blending_matrix * p;

    if (J) {
      J->start_idx = 0;
      for (int i = 0; i < N; i++) {
        J->d_val_d_knot[i].setZero();
      }
    }

    SO3 A_accum_inv;           // A_3_inv * A2_inv * A1_inv
    Mat3 A_post_inv[DEG + 1];  // A_3_inv*A2_inv*A1_inv,A_3_inv*A2_inv,A_3_inv,I
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];

    A_post_inv[DEG] = A_accum_inv.matrix();  // Identity Matrix

    for (int i = DEG - 1; i >= 0; i--) {
      Eigen::Map<SO3 const> R0(knots[i]);
      Eigen::Map<SO3 const> R1(knots[i + 1]);

      Vec3 delta = (R0.inverse() * R1).log();
      Vec3 kdelta = delta * coeff[i + 1];

      A_accum_inv *= SO3::exp(-kdelta);

      if (J) {
        Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
        Sophus::rightJacobianSO3(kdelta, Jr_kdelta[i]);
        A_post_inv[i] = A_accum_inv.matrix();
      }
    }

    Eigen::Map<SO3 const> Ri(knots[0]);
    SO3 res = Ri * A_accum_inv.inverse();

    if (J) {
      Mat3 J_helper = A_post_inv[0];
      J->d_val_d_knot[0] = J_helper;

      /// d_val_d_knot 
      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * A_post_inv[i + 1] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  /**
   * @brief   (1) r = Lhs * R(t)^T * rhs
   *          (2) r = Log( Lhs * R(t)^T * Rhs )
   * Jacobians in J are omitted left multiply term:
   *  (1) Lhs * R(t)^T * (rhs)_{\wedge}
   *  (2) -Jr^{-1}(r) * Rhs^T
   * @return  R(t)^T
   * */
  static SO3 EvaluateRTp(const int64_t time_ns,
                         const SplineSegmentMeta<N>& splne_meta,
                         double const* const* knots,
                         // const Vec3& rhs,
                         JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blending_matrix_ * p;

    if (J) {
      J->start_idx = s;
    }
    Eigen::Map<SO3 const> Ri(knots[s]);

    SO3 Si_A_pre[DEG + 1];
    Mat3 Ri_A_pre[DEG + 1];  // Ri, Ri*A1, Ri*A1*A2, Ri*A1*A2*A3
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];

    Si_A_pre[0] = Ri;
//    Ri_A_pre[0] = Ri.matrix();
    for (int i = 0; i < DEG; i++) {  // 0 1 2
      Eigen::Map<SO3 const> R0(knots[s + i]);
      Eigen::Map<SO3 const> R1(knots[s + i + 1]);

      Vec3 delta = (R0.inverse() * R1).log();
      Vec3 kdelta = delta * coeff[i + 1];

//      Ri_A_pre[i + 1] = Ri_A_pre[i] * SO3::exp(kdelta).matrix();
      Si_A_pre[i + 1] = Si_A_pre[i] * SO3::exp(kdelta);

      if (J) {
        Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
        Sophus::rightJacobianSO3(-kdelta, Jr_kdelta[i]);
      }
    }


    for (int i = 0; i < DEG + 1; i++) {  // 0 1 2
        Ri_A_pre[i] = Si_A_pre[i].matrix();
    }


//    SO3 res(Ri_A_pre[DEG].transpose());  // R^T
    SO3 res = Si_A_pre[DEG].inverse(); // R^T

    if (J) {
      Mat3 J_helper = Ri_A_pre[0];
      J->d_val_d_knot[0] = J_helper;

      /// d_val_d_knot 
      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * Ri_A_pre[i] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  /**
   * @brief   r = Log(R(t)^T * R).
   * Jacobians in J are omitted left multiply term: -Jl^{-1}(r)
   * @return  R(t)^T
   * */
  static SO3 EvaluateLogRT(const int64_t time_ns,
                           const SplineSegmentMeta<N>& splne_meta,
                           double const* const* knots,
                           JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);

    VecN coeff = blending_matrix_ * p;

    if (J) {
      J->start_idx = s;
    }
    Eigen::Map<SO3 const> Ri(knots[s]);

    SO3 A_accum_inv;  // A_3_inv * A2_inv * A1_inv
    // A_3_inv * A2_inv * A1_inv, A_3_inv * A2_inv, A_3_inv, I
    Mat3 A_post_inv[DEG + 1];
    Mat3 Jr_inv_delta[DEG], Jr_kdelta[DEG];

    A_post_inv[DEG] = Mat3::Identity();   // 3
    for (int i = DEG - 1; i >= 0; i--) {  // 2 1 0
      Eigen::Map<SO3 const> R0(knots[s + i]);
      Eigen::Map<SO3 const> R1(knots[s + i + 1]);

      Vec3 delta = (R0.inverse() * R1).log();
      Vec3 kdelta = delta * coeff[i + 1];

      A_accum_inv *= SO3::exp(-kdelta);

      if (J) {
        Sophus::rightJacobianInvSO3(delta, Jr_inv_delta[i]);
        Sophus::rightJacobianSO3(-kdelta, Jr_kdelta[i]);
        A_post_inv[i] = A_accum_inv.matrix();
      }
    }

    Mat3 R_T = A_post_inv[0] * Ri.inverse().matrix();
    Eigen::Quaterniond q_t = Eigen::Quaterniond(R_T);
    q_t.normalize();
    SO3 res(q_t.toRotationMatrix());  // R^T

    if (J) {
      Mat3 J_helper = A_post_inv[0];
      J->d_val_d_knot[0] = J_helper;

      /// d_val_d_knot 
      for (int i = 0; i < DEG; i++) {
        J_helper = coeff[i + 1] * A_post_inv[i] * Jr_kdelta[i];

        J->d_val_d_knot[i] -= (J_helper * Jr_inv_delta[i].transpose());
        J->d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
      }
    }

    return res;
  }

  /// @brief Evaluate rotational velocity (first time derivative) of SO(3)
  /// B-spline in the body frame
  ///
  /// @param[in] time_ns time for evaluating velocity of the spline
  /// @param[out] J if not nullptr, return the Jacobian of the rotational
  /// velocity in body frame with respect to knots
  /// @return rotational velocity (3x1 vector)
  static Vec3 VelocityBody(const int64_t time_ns,
                           const SplineSegmentMeta<N>& splne_meta,
                           double const* const* knots,
                           JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);
    VecN coeff = blending_matrix_ * p;

    baseCoeffsWithTime<1>(p, u);
    VecN dcoeff = splne_meta.pow_inv_dt[1] * blending_matrix_ * p;

    Vec3 delta_vec[DEG];

    Mat3 R_tmp[DEG];
    SO3 accum;
    SO3 exp_k_delta[DEG];

    Mat3 Jr_delta_inv[DEG], Jr_kdelta[DEG];

    for (int i = DEG - 1; i >= 0; i--) {
      Eigen::Map<SO3 const> p0(knots[s + i]);
      Eigen::Map<SO3 const> p1(knots[s + i + 1]);

      SO3 r01 = p0.inverse() * p1;
      delta_vec[i] = r01.log();

      Sophus::rightJacobianInvSO3(delta_vec[i], Jr_delta_inv[i]);
      Jr_delta_inv[i] *= p1.inverse().matrix();

      Vec3 k_delta = coeff[i + 1] * delta_vec[i];
      Sophus::rightJacobianSO3(-k_delta, Jr_kdelta[i]);

      R_tmp[i] = accum.matrix();
      exp_k_delta[i] = Sophus::SO3d::exp(-k_delta);
      accum *= exp_k_delta[i];
    }

    Mat3 d_vel_d_delta[DEG];

    d_vel_d_delta[0] = dcoeff[1] * R_tmp[0] * Jr_delta_inv[0];
    Vec3 rot_vel = delta_vec[0] * dcoeff[1];
    for (int i = 1; i < DEG; i++) {
      d_vel_d_delta[i] =
          R_tmp[i - 1] * SO3::hat(rot_vel) * Jr_kdelta[i] * coeff[i + 1] +
          R_tmp[i] * dcoeff[i + 1];
      d_vel_d_delta[i] *= Jr_delta_inv[i];

      rot_vel = exp_k_delta[i] * rot_vel + delta_vec[i] * dcoeff[i + 1];
    }

    if (J) {
      J->start_idx = s;
      for (int i = 0; i < N; i++) J->d_val_d_knot[i].setZero();
      for (int i = 0; i < DEG; i++) {
        J->d_val_d_knot[i] -= d_vel_d_delta[i];
        J->d_val_d_knot[i + 1] += d_vel_d_delta[i];
      }
    }

    return rot_vel;
  }

  static Vec3 accelerationBody(const int64_t time_ns,
                               const SplineSegmentMeta<N>& splne_meta,
                               double const* const* knots) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTime<0>(p, u);
    VecN coeff = blending_matrix_ * p;

    baseCoeffsWithTime<1>(p, u);
    VecN dcoeff = splne_meta.pow_inv_dt[1] * blending_matrix_ * p;

    baseCoeffsWithTime<2>(p, u);
    VecN ddcoeff = splne_meta.pow_inv_dt[2] * blending_matrix_ * p;

    SO3 r_accum;

    Vec3 rot_vel;
    rot_vel.setZero();

    Vec3 rot_accel;
    rot_accel.setZero();

    for (int i = 0; i < DEG; i++) {
      Eigen::Map<SO3 const> p0(knots[s + i]);
      Eigen::Map<SO3 const> p1(knots[s + i + 1]);

      SO3 r01 = p0.inverse() * p1;
      Vec3 delta = r01.log();

      SO3 rot = SO3::exp(-delta * coeff[i + 1]);

      rot_vel = rot * rot_vel;
      Vec3 vel_current = dcoeff[i + 1] * delta;
      rot_vel += vel_current;

      rot_accel = rot * rot_accel;
      rot_accel += ddcoeff[i + 1] * delta + rot_vel.cross(vel_current);
    }

    return rot_accel;
  }

  // protected:
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
                                 double t) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
    Eigen::MatrixBase<Derived>& res =
        const_cast<Eigen::MatrixBase<Derived>&>(res_const);

    res.setZero();

    if (Derivative < N) {
      res[Derivative] = base_coefficients_(Derivative, Derivative);

      double _t = t;
      for (int j = Derivative + 1; j < N; j++) {
        res[j] = base_coefficients_(Derivative, j) * _t;
        _t = _t * t;
      }
    }
  }

  ///< Blending matrix. See \ref computeBlendingMatrix.
  static inline MatN base_coefficients_ =
      computeBaseCoefficients<SplineOrder, double>();

  static inline MatN blending_matrix_ =
      computeBlendingMatrix<SplineOrder, double,
                            true>();  ///< Base coefficients matrix.
};

// typename So3SplineView::MatN So3SplineView::base_coefficients_ =
//    cocolic::computeBaseCoefficients<SplineOrder, double>();

// typename So3SplineView::MatN So3SplineView::blending_matrix_ =
//    cocolic::computeBlendingMatrix<SplineOrder, double, true>();

}  // namespace analytic_derivative
}  // namespace cocolic
