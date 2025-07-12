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

#include <ceres/ceres.h>
#include <spline/spline_segment.h>
#include <utils/parameter_struct.h>

#include "split_spline_view.h"

namespace cocolic {

namespace analytic_derivative {

class GravityFactor : public ceres::SizedCostFunction<3, 3> {
 public:
  GravityFactor(const Eigen::Vector3d& gravity,
                const Eigen::Vector3d& sqrt_info)
      : gravity_(gravity), sqrt_info_(sqrt_info) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    Eigen::Map<Eigen::Vector3d const> gravity_now(parameters[0]);

    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = sqrt_info_.asDiagonal() * (gravity_now - gravity_);

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_grav(
            jacobians[0]);
        jac_grav.setZero();
        jac_grav.block<3, 3>(0, 0) =
            sqrt_info_.asDiagonal() * Eigen::Matrix3d::Identity();
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d gravity_;
  Eigen::Vector3d sqrt_info_;
};

// bias_gyr_i, bias_gyr_j, bias_acc_i, bias_acc_j
class BiasFactor : public ceres::SizedCostFunction<6, 3, 3, 3, 3> {
 public:
  BiasFactor(double dt, const Eigen::Matrix<double, 6, 1>& sqrt_info) {
    // double sqrt_dt = std::sqrt(dt);
    // sqrt_info_.setZero();
    // sqrt_info_.diagonal() = sqrt_info / sqrt_dt;
    sqrt_info_.setZero();
    sqrt_info_.diagonal() = sqrt_info;
  }
  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    Eigen::Map<Vec3d const> bias_gyr_i(parameters[0]);
    Eigen::Map<Vec3d const> bias_gyr_j(parameters[1]);
    Eigen::Map<Vec3d const> bias_acc_i(parameters[2]);
    Eigen::Map<Vec3d const> bias_acc_j(parameters[3]);

    Vec6d res;
    res.block<3, 1>(0, 0) = bias_gyr_j - bias_gyr_i;
    res.block<3, 1>(3, 0) = bias_acc_j - bias_acc_i;

    Eigen::Map<Vec6d> residual(residuals);
    residual = sqrt_info_ * res;

    if (jacobians) {
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_i(
            jacobians[0]);
        jac_bg_i.setZero();
        jac_bg_i.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        jac_bg_i.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bg_j(
            jacobians[1]);
        jac_bg_j.setZero();
        jac_bg_j.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        jac_bg_j.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_i(
            jacobians[2]);
        jac_ba_i.setZero();
        jac_ba_i.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
        jac_ba_i.applyOnTheLeft(sqrt_info_);
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba_j(
            jacobians[3]);
        jac_ba_j.setZero();
        jac_ba_j.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        jac_ba_j.applyOnTheLeft(sqrt_info_);
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d acc_i_, acc_j_;
  Eigen::Vector3d gyr_i_, gyr_j_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

class IMUFactor : public ceres::CostFunction, SplitSpineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMUFactor(int64_t time_ns, const IMUData& imu_data,
            const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
            const Vec6d& info_vec)
      : time_ns_(time_ns),
        imu_data_(imu_data),
        spline_segment_meta_(spline_segment_meta),

        info_vec_(info_vec) {
    /// 
    set_num_residuals(6);

    /// 
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(3);  // gyro bias
    mutable_parameter_block_sizes()->push_back(3);  // accel bias
    mutable_parameter_block_sizes()->push_back(3);  // gravity
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_rot_w;
    typename SO3View::JacobianStruct J_rot_a;
    typename R3View::JacobianStruct J_pos;
    Mat3d J_gravity;

    size_t knot_num = this->spline_segment_meta_.NumParameters();
    Eigen::Map<Vec3d const> gyro_bias(parameters[2 * knot_num]);
    Eigen::Map<Vec3d const> accel_bias(parameters[2 * knot_num + 1]);
    Eigen::Map<Vec3d const> gravity(parameters[2 * knot_num + 2]);
    double time_offset_in_ns = parameters[2 * knot_num + 3][0];

    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;
    typename SplitView::SplineIMUData spline_data;
    if (jacobians) {
      spline_data =
          SplitView::Evaluate(t_corrected, spline_segment_meta_, parameters,
                              gravity, &J_rot_w, &J_rot_a, &J_pos);
    } else {
      spline_data = SplitView::Evaluate(t_corrected, spline_segment_meta_,
                                        parameters, gravity);
    }

    Eigen::Map<Vec6d> residual(residuals);
    residual.block<3, 1>(0, 0) =
        spline_data.gyro - (imu_data_.gyro - gyro_bias);
    residual.block<3, 1>(3, 0) =
        spline_data.accel - (imu_data_.accel - accel_bias);

    residual = (info_vec_.asDiagonal() * residual).eval();

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i + spline_data.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_knot_R(
            jacobians[idx]);
        jac_knot_R.setZero();
        /// for gyro residual
        jac_knot_R.block<3, 3>(0, 0) = J_rot_w.d_val_d_knot[i];
        /// for accel residual
        jac_knot_R.block<3, 3>(3, 0) = J_rot_a.d_val_d_knot[i];

        jac_knot_R = (info_vec_.asDiagonal() * jac_knot_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i + spline_data.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_knot_p(
            jacobians[idx]);
        jac_knot_p.setZero();

        /// for accel residual
        jac_knot_p.block<3, 3>(3, 0) =
            J_pos.d_val_d_knot[i] * spline_data.R_inv.matrix();
        jac_knot_p = (info_vec_.asDiagonal() * jac_knot_p).eval();
      }
    }

    /// bias
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bw(
          jacobians[2 * knot_num]);
      jac_bw.setZero();
      jac_bw.block<3, 3>(0, 0).diagonal() = info_vec_.head(3);
    }

    if (jacobians[2 * knot_num + 1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba(
          jacobians[2 * knot_num + 1]);
      jac_ba.setZero();
      jac_ba.block<3, 3>(3, 0).diagonal() = info_vec_.tail(3);
    }

    /// gravity
    if (jacobians[2 * knot_num + 2]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_gravity(
          jacobians[2 * knot_num + 2]);
      jac_gravity.setZero();
      jac_gravity.block<3, 3>(3, 0) = spline_data.R_inv.matrix();
      jac_gravity = (info_vec_.asDiagonal() * jac_gravity).eval();
    }

    // Time_offset
    if (jacobians[2 * knot_num + 3]) {
      Eigen::Map<Eigen::Matrix<double, 6, 1>> jac_t_offset(
          jacobians[2 * knot_num + 3]);
      jac_t_offset.setZero();

      Eigen::Vector3d rot_accel = SO3View::accelerationBody(
          t_corrected, spline_segment_meta_, parameters);
      Eigen::Vector3d jerk_R3 = R3View::evaluate<3>(
          t_corrected, spline_segment_meta_, parameters + knot_num);
      Eigen::Matrix3d rot = spline_data.R_inv.inverse().matrix();

      Eigen::Matrix3d gyro_hat = SO3d::hat(spline_data.gyro);
      Eigen::Matrix3d rot_dot = rot * gyro_hat;

      jac_t_offset.block<3, 1>(0, 0) =
          SO3d::vee(rot_dot.transpose() * rot_dot) +
          SO3d::vee(rot.transpose() *
                    (rot_dot * gyro_hat + rot * SO3d::hat(rot_accel)));
      jac_t_offset.block<3, 1>(3, 0) =
          rot_dot.transpose() * rot * spline_data.accel +
          rot.transpose() * jerk_R3;
      jac_t_offset = (1e-9 * info_vec_.asDiagonal() * jac_t_offset).eval();
    }

    return true;
  }

 private:
  int64_t time_ns_;
  IMUData imu_data_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  Vec6d info_vec_;
};

class IMUFactorNURBS : public ceres::CostFunction, SplitSpineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMUFactorNURBS(int64_t time_ns, const IMUData& imu_data,
            const Vec3d& gravity, const Vec6d& info_vec,
            const std::vector<int64_t>& knts, const std::pair<int, double>& su,
            const Eigen::Matrix4d& blending_matrix,
            const Eigen::Matrix4d& cumulative_blending_matrix)
      : time_ns_(time_ns),
        imu_data_(imu_data),
        gravity_(gravity),
        info_vec_(info_vec),
        knts_(knts),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix) {
    /// 
    set_num_residuals(6);
    /// 
    size_t knot_num = 4;
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(3);  // gyro bias
    mutable_parameter_block_sizes()->push_back(3);  // accel bias
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_rot_w;  //gyro residual w.r.t. R
    typename SO3View::JacobianStruct J_rot_a;  //accel residual w.r.t. R
    typename R3View::JacobianStruct J_pos;  //accel w.r.t. P
    typename SplitView::SplineIMUData spline_data;

    if (jacobians) {
      spline_data = SplitView::EvaluateNURBS(time_ns_,
                                                                                           parameters, gravity_, knts_, su_,
                                                                                           blending_matrix_,
                                                                                           cumulative_blending_matrix_,
                                                                                           &J_rot_w, &J_rot_a, &J_pos);
    }
    else {
      spline_data = SplitView::EvaluateNURBS(time_ns_,
                                                                                           parameters, gravity_, knts_, su_,
                                                                                           blending_matrix_,
                                                                                           cumulative_blending_matrix_);
    }

    size_t knot_num = 4;
    Eigen::Map<Vec3d const> gyro_bias(parameters[2 * knot_num]);
    Eigen::Map<Vec3d const> accel_bias(parameters[2 * knot_num + 1]);

    Eigen::Map<Vec6d> residual(residuals);
    residual.block<3, 1>(0, 0) =
        spline_data.gyro - (imu_data_.gyro - gyro_bias);
    residual.block<3, 1>(3, 0) =
        spline_data.accel - (imu_data_.accel - accel_bias);
    residual = (info_vec_.asDiagonal() * residual).eval();

    if (!jacobians) {
      return true;
    }
    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_knot_R(
            jacobians[idx]);
        jac_knot_R.setZero();
        /// for gyro residual
        jac_knot_R.block<3, 3>(0, 0) = J_rot_w.d_val_d_knot[i];
        /// for accel residual
        jac_knot_R.block<3, 3>(3, 0) = J_rot_a.d_val_d_knot[i];

        jac_knot_R = (info_vec_.asDiagonal() * jac_knot_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_knot_p(
            jacobians[idx]);
        jac_knot_p.setZero();

        /// for accel residual
        jac_knot_p.block<3, 3>(3, 0) =
            J_pos.d_val_d_knot[i] * spline_data.R_inv.matrix();
        jac_knot_p = (info_vec_.asDiagonal() * jac_knot_p).eval();
      }
    }

    /// bias
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_bw(
          jacobians[2 * knot_num]);
      jac_bw.setZero();
      jac_bw.block<3, 3>(0, 0).diagonal() = info_vec_.head(3);
    }

    if (jacobians[2 * knot_num + 1]) {
      Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_ba(
          jacobians[2 * knot_num + 1]);
      jac_ba.setZero();
      jac_ba.block<3, 3>(3, 0).diagonal() = info_vec_.tail(3);
    }

    return true;
  }

 private:
  int64_t time_ns_;
  IMUData imu_data_;
  Vec3d gravity_;
  Vec6d info_vec_;
  std::vector<int64_t> knts_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
};

class IMUPoseFactor : public ceres::CostFunction {
 public:
  IMUPoseFactor(int64_t time_ns, const PoseData& pose_data,
                const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
                const Eigen::Matrix<double, 6, 1>& info_vec)
      : time_ns_(time_ns),
        pose_data_(pose_data),
        spline_segment_meta_(spline_segment_meta),
        info_vec_(info_vec) {
    /// 
    set_num_residuals(6);
    /// 
    size_t kont_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;

    size_t knot_num = spline_segment_meta_.NumParameters();
    size_t P_offset = knot_num;

    double time_offset_in_ns = parameters[2 * knot_num][0];
    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    if (jacobians) {
      S_ItoG = so3_spline_view.EvaluateRotation(
          t_corrected, spline_segment_meta_, parameters, &J_R);
      p_IinG = r3_spline_view.evaluate(t_corrected, spline_segment_meta_,
                                       parameters + P_offset, &J_p);
    } else {
      S_ItoG = so3_spline_view.EvaluateRotation(
          t_corrected, spline_segment_meta_, parameters);
      p_IinG = r3_spline_view.evaluate(t_corrected, spline_segment_meta_,
                                       parameters + P_offset);
    }

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) =
        (S_ItoG * pose_data_.orientation.inverse()).log();
    residual.block<3, 1>(3, 0) = p_IinG - pose_data_.position;

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix<double, 3, 1> res = residual.block<3, 1>(0, 0);
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res, Jrot);

      for (size_t i = 0; i < SplineOrder; i++) {
        // 
        size_t idx = J_R.start_idx + i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>
              jacobian_kont_R(jacobians[idx]);
          jacobian_kont_R.setZero();
          /// for rotation residual
          jacobian_kont_R.block<3, 3>(0, 0) = Jrot * J_R.d_val_d_knot[i];
          /// L*J
          jacobian_kont_R = (info_vec_.asDiagonal() * jacobian_kont_R).eval();
        }
      }

      for (size_t i = 0; i < SplineOrder; i++) {
        size_t idx = J_R.start_idx + i;
        if (jacobians[idx + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>
              jacobian_kont_P(jacobians[idx + knot_num]);
          jacobian_kont_P.setZero();

          /// for position residual
          jacobian_kont_P.block<3, 3>(3, 0) =
              J_p.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
          /// L*J
          jacobian_kont_P = (info_vec_.asDiagonal() * jacobian_kont_P).eval();
        }
      }

      // Time_offset
      if (jacobians[2 * knot_num]) {
        Eigen::Map<Eigen::Matrix<double, 6, 1>> jac_t_offset(
            jacobians[2 * knot_num]);
        jac_t_offset.setZero();

        Eigen::Vector3d gyro = So3SplineView::VelocityBody(
            t_corrected, spline_segment_meta_, parameters);
        Eigen::Vector3d vel = RdSplineView::velocity(
            t_corrected, spline_segment_meta_, parameters + knot_num);

        Eigen::Matrix3d gyro_hat = SO3d::hat(gyro);
        SO3d rot_dot = S_ItoG * SO3d(Eigen::Quaterniond(gyro_hat).normalized());
        jac_t_offset.block<3, 1>(0, 0) =
            (rot_dot * pose_data_.orientation.inverse()).log();  //

        jac_t_offset.block<3, 1>(3, 0) = vel;
        jac_t_offset = (1e-9 * info_vec_.asDiagonal() * jac_t_offset).eval();
      }
    }

    residual = (info_vec_.asDiagonal() * residual).eval();
    return true;
  }

 private:
  int64_t time_ns_;
  PoseData pose_data_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  Eigen::Matrix<double, 6, 1> info_vec_;
};

class IMUPoseFactorNURBS : public ceres::CostFunction {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMUPoseFactorNURBS(int64_t time_ns, const PoseData& pose_data,
                const Vec6d& info_vec,
                const std::vector<int64_t>& knts, const std::pair<int, double>& su,
                const Eigen::Matrix4d& blending_matrix,
                const Eigen::Matrix4d& cumulative_blending_matrix)
      : time_ns_(time_ns),
        pose_data_(pose_data),
        info_vec_(info_vec),
        knts_(knts),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix) {
    /// 
    set_num_residuals(6);
    ///
    size_t kont_num = 4;
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;

    size_t knot_num = 4;
    size_t P_offset = knot_num;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    if (jacobians) {
      S_ItoG = so3_spline_view.EvaluateRotationNURBS(
          su_, cumulative_blending_matrix_, parameters, &J_R);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                       parameters + P_offset, &J_p);
    } else {
      S_ItoG = so3_spline_view.EvaluateRotationNURBS(
          su_, cumulative_blending_matrix_, parameters);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                       parameters + P_offset);
    }

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.block<3, 1>(0, 0) =
        (S_ItoG * pose_data_.orientation.inverse()).log();
    residual.block<3, 1>(3, 0) = p_IinG - pose_data_.position;

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix<double, 3, 1> res = residual.block<3, 1>(0, 0);
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res, Jrot);

      for (size_t i = 0; i < SplineOrder; i++) {
        // 
        // size_t idx = J_R.start_idx + i;
        size_t idx = i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>
              jacobian_kont_R(jacobians[idx]);
          jacobian_kont_R.setZero();
          /// for rotation residual
          jacobian_kont_R.block<3, 3>(0, 0) = Jrot * J_R.d_val_d_knot[i];  //
          /// L*J
          jacobian_kont_R = (info_vec_.asDiagonal() * jacobian_kont_R).eval();
        }
      }

      for (size_t i = 0; i < SplineOrder; i++) {
        // size_t idx = J_R.start_idx + i;
        size_t idx = i;
        if (jacobians[idx + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>
              jacobian_kont_P(jacobians[idx + knot_num]);
          jacobian_kont_P.setZero();

          /// for position residual
          jacobian_kont_P.block<3, 3>(3, 0) =
              J_p.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
          /// L*J
          jacobian_kont_P = (info_vec_.asDiagonal() * jacobian_kont_P).eval();
        }
      }
    }

    residual = (info_vec_.asDiagonal() * residual).eval();
    return true;
  }

 private:
  int64_t time_ns_;
  PoseData pose_data_;
  Vec6d info_vec_;
  std::vector<int64_t> knts_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
};

class IMURelativePoseFactorNURBS : public ceres::CostFunction {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;
  using SplitView = SplitSpineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  IMURelativePoseFactorNURBS(int64_t time_ns, const PoseData& pose_data,
                const SE3d& pose_init,
                const Vec6d& info_vec,
                const std::vector<int64_t>& knts, const std::pair<int, double>& su,
                const Eigen::Matrix4d& blending_matrix,
                const Eigen::Matrix4d& cumulative_blending_matrix)
      : time_ns_(time_ns),
        pose_data_(pose_data),
        pose_init_(pose_init),
        info_vec_(info_vec),
        knts_(knts),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix) {
    /// 
    set_num_residuals(6);
    /// 
    size_t kont_num = 4;
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename So3SplineView::JacobianStruct J_R;
    typename RdSplineView::JacobianStruct J_p;

    size_t knot_num = 4;
    size_t P_offset = knot_num;

    So3SplineView so3_spline_view;
    RdSplineView r3_spline_view;
    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
    if (jacobians) {
      S_ItoG = so3_spline_view.EvaluateRotationNURBS(
          su_, cumulative_blending_matrix_, parameters, &J_R);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                       parameters + P_offset, &J_p);
    } else {
      S_ItoG = so3_spline_view.EvaluateRotationNURBS(
          su_, cumulative_blending_matrix_, parameters);
      p_IinG = r3_spline_view.evaluateNURBS(su_, blending_matrix_,
                                       parameters + P_offset);
    }

    SO3d rot_init = pose_init_.so3();
    Eigen::Vector3d pos_init = pose_init_.translation();
    SO3d relative_R = rot_init.inverse() * S_ItoG;
    Eigen::Vector3d relative_t = rot_init.inverse() * (p_IinG - pos_init);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    // residual.block<3, 1>(0, 0) = (S_ItoG * pose_data_.orientation.inverse()).log();
    // residual.block<3, 1>(3, 0) = p_IinG - pose_data_.position;
    residual.block<3, 1>(0, 0) = (relative_R * pose_data_.orientation.inverse()).log();
    residual.block<3, 1>(3, 0) = relative_t - pose_data_.position;

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix<double, 3, 1> res = residual.block<3, 1>(0, 0);
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res, Jrot);
      // Sophus::rightJacobianInvSO3(res, Jrot);

      for (size_t i = 0; i < SplineOrder; i++) {
        // 没有 timeoffset, J_R.start_idx 应该为 0, knot_num == N
        // size_t idx = J_R.start_idx + i;
        size_t idx = i;
        if (jacobians[idx]) {
          Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>
              jacobian_kont_R(jacobians[idx]);
          jacobian_kont_R.setZero();
          /// [for rotation residual]
          jacobian_kont_R.block<3, 3>(0, 0) = Jrot * rot_init.inverse().matrix() * J_R.d_val_d_knot[i];  //
          // jacobian_kont_R.block<3, 3>(0, 0) = Jrot * J_R.d_val_d_knot[i];
          // jacobian_kont_R.block<3, 3>(0, 0) = Jrot * pose_data_.orientation.unit_quaternion().toRotationMatrix() * J_R.d_val_d_knot[i];
          /// L*J
          jacobian_kont_R = (info_vec_.asDiagonal() * jacobian_kont_R).eval();
        }
      }

      for (size_t i = 0; i < SplineOrder; i++) {
        // size_t idx = J_R.start_idx + i;
        size_t idx = i;
        if (jacobians[idx + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>
              jacobian_kont_P(jacobians[idx + knot_num]);
          jacobian_kont_P.setZero();

          /// [for position residual]
          // jacobian_kont_P.block<3, 3>(3, 0) =
          //     J_p.d_val_d_knot[i] * Eigen::Matrix3d::Identity();
          jacobian_kont_P.block<3, 3>(3, 0) =
              J_p.d_val_d_knot[i] * rot_init.inverse().matrix() * Eigen::Matrix3d::Identity();  //
          /// L*J
          jacobian_kont_P = (info_vec_.asDiagonal() * jacobian_kont_P).eval();
        }
      }
    }

    residual = (info_vec_.asDiagonal() * residual).eval();
    return true;
  }

 private:
  int64_t time_ns_;
  PoseData pose_data_;
  SE3d pose_init_;
  Vec6d info_vec_;
  std::vector<int64_t> knts_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
};

class LocalVelocityFactor : public ceres::CostFunction,
                            So3SplineView,
                            RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using SO3d = Sophus::SO3<double>;

  LocalVelocityFactor(int64_t time_ns, const Eigen::Vector3d& local_velocity,
                      const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
                      double weight)
      : time_ns_(time_ns),
        local_velocity_(local_velocity),
        spline_segment_meta_(spline_segment_meta),
        weight_(weight) {
    /// 
    set_num_residuals(3);

    /// 
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    SO3d S_ItoG;
    Vec3d v_inG = Vec3d::Zero();

    size_t knot_num = spline_segment_meta_.NumParameters();
    double time_offset_in_ns = parameters[2 * knot_num][0];

    int64_t t_corrected = time_ns_ + (int64_t)time_offset_in_ns;

    if (jacobians) {
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_segment_meta_,
                                   parameters, &J_R);
      v_inG = R3View::velocity(t_corrected, spline_segment_meta_,
                               parameters + knot_num, &J_p);
    } else {
      S_ItoG = SO3View::EvaluateRp(t_corrected, spline_segment_meta_,
                                   parameters, nullptr);
      v_inG = R3View::velocity(t_corrected, spline_segment_meta_,
                               parameters + knot_num, nullptr);
    }

    Eigen::Map<Vec3d> residual(residuals);
    residual = S_ItoG * local_velocity_ - v_inG;

    residual = (weight_ * residual).eval();

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < knot_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + knot_num]) {
          Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + knot_num]);
          jac_kont_p.setZero();
        }
      }
    }

    Eigen::Matrix3d jac_lhs_R;
    jac_lhs_R = -S_ItoG.matrix() * SO3::hat(local_velocity_);

    /// Rotation control point
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = i + J_R.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 3*3 3*3
        jac_kont_R.block<3, 3>(0, 0) = jac_lhs_R * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    Eigen::Matrix3d jac_lhs_P = -1 * Eigen::Matrix3d::Identity();
    for (size_t i = 0; i < SplineOrder; i++) {
      size_t idx = knot_num + i + J_p.start_idx;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 3*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }

    // Time_offset
    if (jacobians[2 * knot_num]) {
      Eigen::Map<Eigen::Matrix<double, 3, 1>> jac_t_offset(
          jacobians[2 * knot_num]);
      jac_t_offset.setZero();

      Eigen::Vector3d gyro = So3SplineView::VelocityBody(
          t_corrected, spline_segment_meta_, parameters);
      Eigen::Vector3d accel = RdSplineView::acceleration(
          t_corrected, spline_segment_meta_, parameters + knot_num);

      Eigen::Matrix3d gyro_hat = SO3d::hat(gyro);
      Eigen::Matrix3d rot_dot = S_ItoG.matrix() * gyro_hat;

      jac_t_offset = rot_dot * local_velocity_ - accel;
      jac_t_offset = (1e-9 * weight_ * jac_t_offset).eval();
    }

    return true;
  }

 private:
  int64_t time_ns_;
  Eigen::Vector3d local_velocity_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  double weight_;
};

class Local6DoFVelocityFactor : public ceres::CostFunction,
                                So3SplineView,
                                RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using SO3d = Sophus::SO3<double>;

  Local6DoFVelocityFactor(
      int64_t time_ns, const Eigen::Matrix<double, 6, 1>& local_velocity,
      const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
      const Eigen::Matrix<double, 6, 1>& sqrt_info)
      : time_ns_(time_ns),
        local_velocity_(local_velocity),
        spline_segment_meta_(spline_segment_meta),
        sqrt_info_(sqrt_info) {
    /// 
    set_num_residuals(6);

    /// 
    size_t knot_num = this->spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(1);  // time_offset
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    return false;
  }

 private:
  int64_t time_ns_;
  Eigen::Matrix<double, 6, 1> local_velocity_;

  SplineSegmentMeta<SplineOrder> spline_segment_meta_;

  Eigen::Matrix<double, 6, 1> sqrt_info_;
};

class RelativeOrientationFactor : public ceres::CostFunction, So3SplineView {
 public:
  using SO3View = So3SplineView;
  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using SO3d = Sophus::SO3<double>;

  RelativeOrientationFactor(const SO3d& S_BtoA, int64_t ta_ns, int64_t tb_ns,
                            const SplineMeta<SplineOrder>& spline_meta,
                            const Eigen::Vector3d& info_vec)
      : S_BtoA_(S_BtoA),
        ta_ns_(ta_ns),
        tb_ns_(tb_ns),
        spline_meta_(spline_meta) {
    /// 
    set_num_residuals(3);

    /// 
    size_t knot_num = spline_meta_.NumParameters();
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }

    sqrt_info = Eigen::Matrix3d::Zero();
    sqrt_info.diagonal() = info_vec;
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R[2];

    size_t R_offset[2] = {0, 0};
    size_t seg_idx[2] = {0, 0};
    {
      double u;
      spline_meta_.ComputeSplineIndex(ta_ns_, R_offset[0], u);
      spline_meta_.ComputeSplineIndex(tb_ns_, R_offset[1], u);

      // 
      size_t segment0_knot_num = spline_meta_.segments.at(0).NumParameters();
      for (int i = 0; i < 2; ++i) {
        if (R_offset[i] >= segment0_knot_num) {
          seg_idx[i] = 1;
          R_offset[i] = segment0_knot_num;
        } else {
          R_offset[i] = 0;
        }
      }
    }

    SO3d S_IatoG, S_GtoIb;
    if (jacobians) {
      S_IatoG = SO3View::EvaluateRotation(ta_ns_,
                                          spline_meta_.segments.at(seg_idx[0]),
                                          parameters + R_offset[0], &J_R[0]);
      S_GtoIb =
          SO3View::EvaluateLogRT(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + R_offset[1], &J_R[1]);
    } else {
      S_IatoG = SO3View::EvaluateRotation(ta_ns_,
                                          spline_meta_.segments.at(seg_idx[0]),
                                          parameters + R_offset[0], nullptr);
      S_GtoIb =
          SO3View::EvaluateLogRT(tb_ns_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + R_offset[1], nullptr);
    }

    Eigen::Map<Vec3d> residual(residuals);
    Vec3d res_R = (S_GtoIb * S_IatoG * S_BtoA_).log();
    residual.block<3, 1>(0, 0) = sqrt_info * res_R;

    size_t kont_num = spline_meta_.NumParameters();
    // initialize as zero
    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix3d jac_lhs_dR_dR[2];
      Eigen::Matrix3d Jrot;
      Sophus::leftJacobianInvSO3(res_R, Jrot);
      jac_lhs_dR_dR[0] = Jrot * S_GtoIb.matrix();  // ta
      jac_lhs_dR_dR[1] = -Jrot;                    // tb

      for (int seg = 0; seg < 2; ++seg) {
        for (size_t i = 0; i < SplineOrder; i++) {
          size_t idx = R_offset[seg] + J_R[seg].start_idx + i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            // 3*3 3*3
            jac_kont_R.block<3, 3>(0, 0) +=
                sqrt_info * jac_lhs_dR_dR[seg] * J_R[seg].d_val_d_knot[i];
          }
        }
      }
    }

    return true;
  }

  Eigen::Matrix3d sqrt_info;

  SO3d S_BtoA_;
  double ta_ns_, tb_ns_;
  SplineMeta<SplineOrder> spline_meta_;
};

}  // namespace analytic_derivative

}  // namespace cocolic
