#pragma once

// #include "sophus_utils.hpp"
#include "parameter_struct.h"

namespace cocolic {

inline PoseData XYThetaToPoseData(double x, double y, double theta,
                                  double timestamp = 0) {
  PoseData pose;
  Eigen::Vector3d p(x, y, 0);
  Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q(rotation_vector);
  pose.timestamp = timestamp;
  pose.position = p;
  pose.orientation.setQuaternion(q);

  return pose;
}

inline PoseData SE3ToPoseData(SE3d se3_pose, double time = 0) {
  PoseData pose;
  pose.timestamp = time;
  pose.position = se3_pose.translation();
  pose.orientation = se3_pose.so3();
  return pose;
}

inline SE3d Matrix4fToSE3d(Eigen::Matrix4f matrix) {
  Eigen::Vector3d trans(matrix(0, 3), matrix(1, 3), matrix(2, 3));
  Eigen::Quaterniond q(matrix.block<3, 3>(0, 0).cast<double>());
  q.normalize();
  return SE3d(q, trans);
}

inline void SE3dToPositionEuler(SE3d se3_pose, Eigen::Vector3d &position,
                                Eigen::Vector3d &euler) {
  position = se3_pose.translation();
  Eigen::Quaterniond q = se3_pose.unit_quaternion();
  euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  euler *= 180 / M_PI;
}

// static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R) {
//   Eigen::Vector3d n = R.col(0);
//   Eigen::Vector3d o = R.col(1);
//   Eigen::Vector3d a = R.col(2);

//   Eigen::Vector3d ypr(3);
//   double y = atan2(n(1), n(0));
//   double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
//   double r =
//       atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
//   ypr(0) = y;
//   ypr(1) = p;
//   ypr(2) = r;

//   return ypr / M_PI * 180.0;
// }

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(
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

}  // namespace cocolic
