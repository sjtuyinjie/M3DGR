/*
 * CLINS: Continuous-Time Trajectory Estimation for LiDAR-Inertial System
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Kewei Hu
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

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace mathutils {

template <typename T>
inline T RadToDeg(T rad) {
  return rad * 180.0 / M_PI;
}

template <typename T>
inline T DegToRad(T deg) {
  return deg / 180.0 * M_PI;
}

///< Matrix calculations

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
    const Eigen::MatrixBase<Derived> &v3d) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
  m << typename Derived::Scalar(0), -v3d.z(), v3d.y(), v3d.z(),
      typename Derived::Scalar(0), -v3d.x(), -v3d.y(), v3d.x(),
      typename Derived::Scalar(0);
  return m;
}

// adapted from VINS-mono
inline Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r =
      atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(
    const Eigen::MatrixBase<Derived> &ypr) {
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t y = ypr(0) / 180.0 * M_PI;
  Scalar_t p = ypr(1) / 180.0 * M_PI;
  Scalar_t r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

  return Rz * Ry * Rx;
}

inline Eigen::Quaterniond g2R(Eigen::Vector3d gravity) {
  // Get z axis, which alines with -g (z_in_G=0,0,1)
  Eigen::Vector3d z_axis = -gravity / gravity.norm();

  // Create an x_axis
  Eigen::Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Eigen::Matrix<double, 3, 1> y_axis = SkewSymmetric(z_axis) * x_axis;

  // From these axes get rotation
  Eigen::Matrix<double, 3, 3> Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  Eigen::Quaterniond q0(Ro);
  q0.normalize();
  return q0;
}

inline float Normalize(Eigen::Vector3f v) {
  float result;
  result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  return result;
}

inline Eigen::Matrix3f RotationMatrix(float angle, Eigen::Vector3f u) {
  float norm = Normalize(u);
  Eigen::Matrix3f rotatinMatrix;

  u(0) = u(0) / norm;
  u(1) = u(1) / norm;
  u(2) = u(2) / norm;

  rotatinMatrix(0, 0) = cos(angle) + u(0) * u(0) * (1 - cos(angle));
  rotatinMatrix(0, 1) = u(0) * u(1) * (1 - cos(angle)) - u(2) * sin(angle);
  rotatinMatrix(0, 2) = u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));

  rotatinMatrix(1, 0) = u(2) * sin(angle) + u(0) * u(1) * (1 - cos(angle));
  rotatinMatrix(1, 1) = cos(angle) + u(1) * u(1) * (1 - cos(angle));
  rotatinMatrix(1, 2) = -u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));

  rotatinMatrix(2, 0) = -u(1) * sin(angle) + u(0) * u(2) * (1 - cos(angle));
  rotatinMatrix(2, 1) = u(0) * sin(angle) + u(1) * u(2) * (1 - cos(angle));
  rotatinMatrix(2, 2) = cos(angle) + u(2) * u(2) * (1 - cos(angle));

  return rotatinMatrix;
}

}  // namespace mathutils

#endif  // LIO_MATH_UTILS_H_
