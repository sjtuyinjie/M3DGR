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

class RdSplineView {
 public:
  static constexpr int N = SplineOrder;  ///< Order of the spline.
  static constexpr int DIM = 3;  ///< Dimension of euclidean vector space.

  using MatN = Eigen::Matrix<double, N, N>;
  using VecN = Eigen::Matrix<double, N, 1>;
  using VecD = Eigen::Matrix<double, DIM, 1>;

  /// @brief Struct to store the Jacobian of the spline
  struct JacobianStruct {
    size_t
        start_idx;  ///< Start index of the non-zero elements of the Jacobian.
    std::array<double, N> d_val_d_knot;  ///< Value of nonzero Jacobians.
  };

  RdSplineView() {}

  /// @brief Evaluate value or derivative of the spline
  ///
  /// @param Derivative derivative to evaluate (0 for value)
  /// @param[in] time_ns time for evaluating of the spline
  /// @param[out] J if not nullptr, return the Jacobian of the value with
  /// respect to knots
  /// @return value of the spline or derivative. Euclidean vector of dimention
  /// DIM.
  template <int Derivative = 0>
  static VecD evaluate(const int64_t time_ns,
                       const SplineSegmentMeta<N>& splne_meta,
                       double const* const* knots,
                       JacobianStruct* J = nullptr) {
    std::pair<double, size_t> ui = splne_meta.computeTIndexNs(time_ns);
    size_t s = ui.second;
    double u = ui.first;

    VecN p;
    baseCoeffsWithTimeR3<Derivative>(p, u);

    VecN coeff = splne_meta.pow_inv_dt[Derivative] * (blending_matrix_ * p);

    VecD res;
    res.setZero();

    for (int i = 0; i < N; i++) {
      Eigen::Map<VecD const> p(knots[s + i]);
      res += coeff[i] * p;

      if (J) J->d_val_d_knot[i] = coeff[i];
    }

    if (J) J->start_idx = s;

    return res;
  }

  template <int Derivative = 0>
  static VecD evaluateNURBS(const std::pair<int, double>& su,
                        const Eigen::Matrix4d& blending_matrix,
                        double const* const* knots,
                       JacobianStruct* J = nullptr) {
    double u = su.second;

    VecN p;
    baseCoeffsWithTimeR3<Derivative>(p, u);

    VecN coeff = blending_matrix * p;

    VecD res;
    res.setZero();

    for (int i = 0; i < N; i++) {
      Eigen::Map<VecD const> p(knots[i]);
      res += coeff[i] * p;

      if (J) J->d_val_d_knot[i] = coeff[i];
    }

    if (J) J->start_idx = 0;

    return res;
  }

  /// @brief Alias for first derivative of spline. See \ref evaluate.
  static VecD velocity(const int64_t time_ns,
                       const SplineSegmentMeta<N>& splne_meta,
                       double const* const* knots,
                       JacobianStruct* J = nullptr) {
    return evaluate<1>(time_ns, splne_meta, knots, J);
  }

  /// @brief Alias for second derivative of spline. See \ref evaluate.
  static VecD acceleration(const int64_t time_ns,
                           const SplineSegmentMeta<N>& splne_meta,
                           double const* const* knots,
                           JacobianStruct* J = nullptr) {
    return evaluate<2>(time_ns, splne_meta, knots, J);
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
  static void baseCoeffsWithTimeR3(const Eigen::MatrixBase<Derived>& res_const,
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

  ///< Base coefficients matrix. See \ref computeBaseCoefficients.
  static inline MatN base_coefficients_ =
      computeBaseCoefficients<SplineOrder, double>();

  ///< Blending matrix. See \ref computeBlendingMatrix.
  static inline MatN blending_matrix_ =
      computeBlendingMatrix<SplineOrder, double, false>();
};

// typename RdSplineView::MatN RdSplineView::base_coefficients_ =
//    cocolic::computeBaseCoefficients<SplineOrder, double>();

// typename RdSplineView::MatN RdSplineView::blending_matrix_ =
//    cocolic::computeBlendingMatrix<SplineOrder, double, false>();

}  // namespace analytic_derivative
}  // namespace cocolic
