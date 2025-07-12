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

#include <odom/factor/analytic_diff/split_spline_view.h>

namespace cocolic {
namespace analytic_derivative {

class LoamFeatureFactor : public ceres::CostFunction,
                          So3SplineView,
                          RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  LoamFeatureFactor(int64_t t_point_ns, const PointCorrespondence& pc,
                    const SplineSegmentMeta<SplineOrder>& spline_segment_meta,
                    const SO3d& S_GtoM, const Vec3d& p_GinM, const SO3d& S_LtoI,
                    const Vec3d& p_LinI, double weight)
      : t_point_ns_(t_point_ns),
        pc_(pc),
        spline_segment_meta_(spline_segment_meta),
        S_GtoM_(S_GtoM),
        p_GinM_(p_GinM),
        S_LtoI_(S_LtoI),
        p_LinI_(p_LinI),
        weight_(weight) {
    /// 
    set_num_residuals(1);

    ///
    size_t kont_num = spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    Vec3d p_Lk = pc_.point;
    Vec3d p_IK = S_LtoI_ * p_Lk + p_LinI_;

    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

    size_t kont_num = spline_segment_meta_.NumParameters();
    if (jacobians) {
      S_ItoG = SO3View::EvaluateRp(t_point_ns_, spline_segment_meta_,
                                   parameters, &J_R);
      p_IinG = R3View::evaluate(t_point_ns_, spline_segment_meta_,
                                parameters + kont_num, &J_p);
    } else {
      S_ItoG = SO3View::EvaluateRp(t_point_ns_, spline_segment_meta_,
                                   parameters, nullptr);
      p_IinG = R3View::evaluate(t_point_ns_, spline_segment_meta_,
                                parameters + kont_num, nullptr);
    }
    Vec3d p_M = S_GtoM_ * (S_ItoG * p_IK + p_IinG) + p_GinM_;

    Vec3d J_pi = Vec3d::Zero();
    if (GeometryType::Plane == pc_.geo_type) {
      residuals[0] = p_M.transpose() * pc_.geo_plane.head(3) + pc_.geo_plane[3];

      J_pi = pc_.geo_plane.head(3);
    } else {
      // omit item 1 =: 1.0 / measurement_.geo_normal.norm()
      Vec3d dist_vec = (p_M - pc_.geo_point).cross(pc_.geo_normal);
      residuals[0] = dist_vec.norm();

      J_pi = -dist_vec.transpose() / residuals[0] * SO3d::hat(pc_.geo_normal);
    }
    residuals[0] *= weight_;

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + kont_num]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + kont_num]);
          jac_kont_p.setZero();
        }
      }
    }

    Mat3d J_Xm_R = -S_GtoM_.matrix() * S_ItoG.matrix() * SO3::hat(p_IK);
    Vec3d jac_lhs_R = Vec3d::Zero();
    Vec3d jac_lhs_P = Vec3d::Zero();

    jac_lhs_R = J_pi.transpose() * J_Xm_R;
    jac_lhs_P = J_pi.transpose() * S_GtoM_.matrix();

    /// Rotation control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 1*3 3*3
        jac_kont_R.block<1, 3>(0, 0) =
            jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = kont_num + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 1*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }
    return true;
  }

 private:
  int64_t t_point_ns_;
  PointCorrespondence pc_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;
  SO3d S_GtoM_;
  Vec3d p_GinM_;

  SO3d S_LtoI_;
  Vec3d p_LinI_;

  double weight_;
};

class LoamFeatureFactorNURBS : public ceres::CostFunction,
                          So3SplineView,
                          RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  LoamFeatureFactorNURBS(int64_t t_point_ns, const PointCorrespondence& pc,
                    const std::pair<int, double>& su,
                    const Eigen::Matrix4d& blending_matrix,
                    const Eigen::Matrix4d& cumulative_blending_matrix,
                    const SO3d& S_GtoM, const Vec3d& p_GinM, const SO3d& S_LtoI,
                    const Vec3d& p_LinI, double weight)
      : t_point_ns_(t_point_ns),
        pc_(pc),
        su_(su),
        blending_matrix_(blending_matrix),
        cumulative_blending_matrix_(cumulative_blending_matrix),
        S_GtoM_(S_GtoM),
        p_GinM_(p_GinM),
        S_LtoI_(S_LtoI),
        p_LinI_(p_LinI),
        weight_(weight) {
    ///
    set_num_residuals(1);
    ///
    size_t knot_num = 4;
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < knot_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    Vec3d p_Lk = pc_.point;
    Vec3d p_IK = S_LtoI_ * p_Lk + p_LinI_;

    SO3d S_ItoG;
    Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();

    if (jacobians) {
      S_ItoG = SO3View::EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                   parameters, &J_R);
      p_IinG = R3View::evaluateNURBS(su_, blending_matrix_,
                                parameters + 4, &J_p);
    } else {
      S_ItoG = SO3View::EvaluateRpNURBS(su_, cumulative_blending_matrix_,
                                   parameters, nullptr);
      p_IinG = R3View::evaluateNURBS(su_, blending_matrix_,
                                parameters + 4, nullptr);
    }
    Vec3d p_M = S_GtoM_ * (S_ItoG * p_IK + p_IinG) + p_GinM_;

    Vec3d J_pi = Vec3d::Zero();
    if (GeometryType::Plane == pc_.geo_type) {
      residuals[0] = p_M.transpose() * pc_.geo_plane.head(3) + pc_.geo_plane[3];

      J_pi = pc_.geo_plane.head(3);
    } else {
      // omit item 1 =: 1.0 / measurement_.geo_normal.norm()
      Vec3d dist_vec = (p_M - pc_.geo_point).cross(pc_.geo_normal);
      residuals[0] = dist_vec.norm();

      J_pi = -dist_vec.transpose() / residuals[0] * SO3d::hat(pc_.geo_normal);
    }
    residuals[0] *= weight_;

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < 4; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + 4]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + 4]);
          jac_kont_p.setZero();
        }
      }
    }

    Mat3d J_Xm_R = -S_GtoM_.matrix() * S_ItoG.matrix() * SO3::hat(p_IK);
    Vec3d jac_lhs_R = Vec3d::Zero();
    Vec3d jac_lhs_P = Vec3d::Zero();

    jac_lhs_R = J_pi.transpose() * J_Xm_R;
    jac_lhs_P = J_pi.transpose() * S_GtoM_.matrix();

    /// Rotation control point
    for (size_t i = 0; i < 4; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 1*3 3*3
        jac_kont_R.block<1, 3>(0, 0) =
            jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < 4; i++) {
      size_t idx = 4 + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 1*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }
    return true;
  }

 private:
  int64_t t_point_ns_;
  PointCorrespondence pc_;
  std::pair<int, double> su_;
  Eigen::Matrix4d blending_matrix_;
  Eigen::Matrix4d cumulative_blending_matrix_;
  SO3d S_GtoM_;
  Vec3d p_GinM_;

  SO3d S_LtoI_;
  Vec3d p_LinI_;

  double weight_;
};

class LoamFeatureOptMapPoseFactor : public ceres::CostFunction,
                                    So3SplineView,
                                    RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using Mat3d = Eigen::Matrix<double, 3, 3>;
  using SO3d = Sophus::SO3<double>;

  LoamFeatureOptMapPoseFactor(
      int64_t t_point_ns, const PointCorrespondence& pc,
      const SplineSegmentMeta<SplineOrder>& spline_segment_meta, double weight)
      : t_point_ns_(t_point_ns),
        pc_(pc),
        spline_segment_meta_(spline_segment_meta),
        weight_(weight) {
    assert(init_flag && "LoamFeatureOptMapPoseFactor not init param");
    /// 
    set_num_residuals(1);

    /// 
    size_t kont_num = spline_segment_meta_.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
    mutable_parameter_block_sizes()->push_back(4);  // map rotation
    mutable_parameter_block_sizes()->push_back(3);  // map position
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R;
    typename R3View::JacobianStruct J_p;

    size_t kont_num = spline_segment_meta_.NumParameters();

    SO3d S_IktoG;
    Eigen::Vector3d p_IkinG = Eigen::Vector3d::Zero();
    if (jacobians) {
      S_IktoG = SO3View::EvaluateRp(t_point_ns_, spline_segment_meta_,
                                    parameters, &J_R);
      p_IkinG = R3View::evaluate(t_point_ns_, spline_segment_meta_,
                                 parameters + kont_num, &J_p);
    } else {
      S_IktoG = SO3View::EvaluateRp(t_point_ns_, spline_segment_meta_,
                                    parameters, nullptr);
      p_IkinG = R3View::evaluate(t_point_ns_, spline_segment_meta_,
                                 parameters + kont_num, nullptr);
    }
    Eigen::Map<SO3d const> S_ImtoG(parameters[kont_num * 2]);
    Eigen::Map<Vec3d const> p_IminG(parameters[kont_num * 2 + 1]);

    SO3d S_ItoL = S_LtoI.inverse();
    SO3d S_GtoM = S_ItoL * S_ImtoG.inverse();
    Vec3d p_GinM = S_ItoL * (S_ImtoG.inverse() * (-p_IminG) - p_LinI);

    Vec3d p_IK = S_LtoI * pc_.point + p_LinI;
    Vec3d p_G = S_IktoG * p_IK + p_IkinG;
    Vec3d p_M = S_GtoM * p_G + p_GinM;

    Vec3d J_pi = Vec3d::Zero();
    if (GeometryType::Plane == pc_.geo_type) {
      residuals[0] = p_M.transpose() * pc_.geo_plane.head(3) + pc_.geo_plane[3];

      J_pi = pc_.geo_plane.head(3);
    } else {
      // omit item 1 =: 1.0 / measurement_.geo_normal.norm()
      Vec3d dist_vec = (p_M - pc_.geo_point).cross(pc_.geo_normal);
      residuals[0] = dist_vec.norm();

      J_pi = -dist_vec.transpose() / residuals[0] * SO3d::hat(pc_.geo_normal);
    }
    residuals[0] *= weight_;

    if (!jacobians) {
      return true;
    }

    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + kont_num]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + kont_num]);
          jac_kont_p.setZero();
        }
      }
    }

    Mat3d J_Xm_R = -S_GtoM.matrix() * S_IktoG.matrix() * SO3::hat(p_IK);
    Vec3d jac_lhs_R = Vec3d::Zero();
    Vec3d jac_lhs_P = Vec3d::Zero();

    jac_lhs_R = J_pi.transpose() * J_Xm_R;
    jac_lhs_P = J_pi.transpose() * S_GtoM.matrix();

    /// Rotation control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
            jacobians[idx]);
        jac_kont_R.setZero();

        /// 1*3 3*3
        jac_kont_R.block<1, 3>(0, 0) =
            jac_lhs_R.transpose() * J_R.d_val_d_knot[i];
        jac_kont_R = (weight_ * jac_kont_R).eval();
      }
    }

    /// position control point
    for (size_t i = 0; i < kont_num; i++) {
      size_t idx = kont_num + i;
      if (jacobians[idx]) {
        Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
            jacobians[idx]);
        jac_kont_p.setZero();

        /// 1*1 1*3
        jac_kont_p = J_p.d_val_d_knot[i] * jac_lhs_P;
        jac_kont_p = (weight_ * jac_kont_p).eval();
      }
    }

    // map rotation
    if (jacobians[kont_num * 2]) {
      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_Map_R(
          jacobians[kont_num * 2]);
      jac_Map_R.setZero();

      Vec3d p_Im = S_ImtoG.inverse() * (p_G - p_IminG);
      jac_Map_R.block<1, 3>(0, 0) =
          J_pi.transpose() * (S_ItoL.matrix() * SO3d::hat(p_Im));
      jac_Map_R = (weight_ * jac_Map_R).eval();
    }
    // map position
    if (jacobians[kont_num * 2 + 1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_Map_p(
          jacobians[kont_num * 2 + 1]);
      jac_Map_p.setZero();
      jac_Map_p = -J_pi.transpose() * (S_ItoL * S_ImtoG.inverse()).matrix();
      jac_Map_p = (weight_ * jac_Map_p).eval();
    }

    return true;
  }

  static void SetParam(SO3d _S_LtoI, Vec3d _p_LinI) {
    init_flag = true;
    S_LtoI = _S_LtoI;
    p_LinI = _p_LinI;

    // std::cout << "[LoamFeatureOptMapPoseFactor] p_LinI: " << p_LinI.transpose()
    //           << "\n";
  }

 private:
  static inline bool init_flag = false;
  static inline SO3d S_LtoI;
  static inline Vec3d p_LinI;

  int64_t t_point_ns_;
  PointCorrespondence pc_;
  SplineSegmentMeta<SplineOrder> spline_segment_meta_;

  double weight_;
};

class RalativeLoamFeatureFactor : public ceres::CostFunction,
                                  So3SplineView,
                                  RdSplineView {
 public:
  using SO3View = So3SplineView;
  using R3View = RdSplineView;

  using Vec3d = Eigen::Matrix<double, 3, 1>;
  using SO3d = Sophus::SO3<double>;

  RalativeLoamFeatureFactor(int64_t t_point_ns, int64_t t_map_ns,
                            const PointCorrespondence& pc,
                            const SplineMeta<SplineOrder>& spline_meta,
                            double weight)
      : t_point_ns_(t_point_ns),
        t_map_ns_(t_map_ns),
        pc_(pc),
        spline_meta_(spline_meta),
        weight_(weight) {
    assert(init_flag && "RalativeLoamFeatureFactor not init param");

    /// 
    set_num_residuals(1);

    ///
    size_t kont_num = spline_meta.NumParameters();
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(4);
    }
    for (size_t i = 0; i < kont_num; ++i) {
      mutable_parameter_block_sizes()->push_back(3);
    }
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const {
    typename SO3View::JacobianStruct J_R[2];
    typename R3View::JacobianStruct J_p[2];

    size_t kont_num = spline_meta_.NumParameters();

    // tj = t_map
    // ti = t_point
    double t_i_ = t_point_ns_;
    double t_j_ = t_map_ns_;

    size_t R_offset[2] = {0, 0};
    size_t P_offset[2] = {0, 0};
    size_t seg_idx[2] = {0, 0};
    {
      double u;
      spline_meta_.ComputeSplineIndex(t_i_, R_offset[0], u);
      spline_meta_.ComputeSplineIndex(t_j_, R_offset[1], u);

      // 
      size_t segment0_knot_num = spline_meta_.segments.at(0).NumParameters();
      for (int i = 0; i < 2; ++i) {
        if (R_offset[i] >= segment0_knot_num) {
          seg_idx[i] = 1;
          R_offset[i] = segment0_knot_num;
        } else {
          R_offset[i] = 0;
        }
        P_offset[i] = R_offset[i] + kont_num;
      }
    }

    /// 
    Vec3d p_Ii = S_LtoI * pc_.point + p_LinI;

    SO3d S_IitoG;
    Vec3d p_IiinG = Vec3d::Zero();
    if (jacobians) {
      // rhs = p_Ii
      S_IitoG = SO3View::EvaluateRp(t_i_, spline_meta_.segments.at(seg_idx[0]),
                                    parameters + R_offset[0], &J_R[0]);
      p_IiinG = R3View::evaluate(t_i_, spline_meta_.segments.at(seg_idx[0]),
                                 parameters + P_offset[0], &J_p[0]);
    } else {
      S_IitoG = SO3View::EvaluateRp(t_i_, spline_meta_.segments.at(seg_idx[0]),
                                    parameters + R_offset[0], nullptr);
      p_IiinG = R3View::evaluate(t_i_, spline_meta_.segments.at(seg_idx[0]),
                                 parameters + P_offset[0], nullptr);
    }
    /// 
    Vec3d p_G = S_IitoG * p_Ii + p_IiinG;
    SO3d S_GtoIj;
    Vec3d p_IjinG = Vec3d::Zero();
    if (jacobians) {
      // rhs = p_G - p_IjinG
      S_GtoIj = SO3View::EvaluateRTp(t_j_, spline_meta_.segments.at(seg_idx[1]),
                                     parameters + R_offset[1], &J_R[1]);
      p_IjinG = R3View::evaluate(t_j_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + P_offset[1], &J_p[1]);
    } else {
      S_GtoIj = SO3View::EvaluateRTp(t_j_, spline_meta_.segments.at(seg_idx[1]),
                                     parameters + R_offset[1], nullptr);
      p_IjinG = R3View::evaluate(t_j_, spline_meta_.segments.at(seg_idx[1]),
                                 parameters + P_offset[1], nullptr);
    }
    SO3d S_ItoC = S_LtoI.inverse();
    SO3d S_GtoCj = S_ItoC * S_GtoIj;
    Vec3d x_j = S_GtoCj * (p_G - p_IjinG) - S_ItoC * p_LinI;

    Vec3d J_pi = Vec3d::Zero();
    if (GeometryType::Plane == pc_.geo_type) {
      residuals[0] = x_j.transpose() * pc_.geo_plane.head(3) + pc_.geo_plane[3];

      J_pi = pc_.geo_plane.head(3);
    } else {
      // omit item 1 =: 1.0 / measurement_.geo_normal.norm()
      Vec3d dist_vec = (x_j - pc_.geo_point).cross(pc_.geo_normal);
      residuals[0] = dist_vec.norm();

      J_pi = -dist_vec.transpose() / residuals[0] * SO3d::hat(pc_.geo_normal);
    }
    residuals[0] *= weight_;

    if (jacobians) {
      for (size_t i = 0; i < kont_num; ++i) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
              jacobians[i]);
          jac_kont_R.setZero();
        }
        if (jacobians[i + kont_num]) {
          Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
              jacobians[i + kont_num]);
          jac_kont_p.setZero();
        }
      }
    }

    if (jacobians) {
      Eigen::Matrix<double, 1, 3> jac_lhs_R[2];
      Eigen::Matrix<double, 1, 3> jac_lhs_P[2];

      // 
      jac_lhs_R[0] =
          -J_pi.transpose() * (S_GtoCj * S_IitoG).matrix() * SO3::hat(p_Ii);
      jac_lhs_P[0] = J_pi.transpose() * S_GtoCj.matrix();

      // 
      jac_lhs_R[1] =
          J_pi.transpose() * S_GtoCj.matrix() * SO3::hat(p_G - p_IjinG);
      jac_lhs_P[1] = -J_pi.transpose() * S_GtoCj.matrix();

      ///[step1] jacobians of control points
      for (int seg = 0; seg < 2; ++seg) {
        /// Rotation control point
        size_t pre_idx_R = R_offset[seg] + J_R[seg].start_idx;
        for (size_t i = 0; i < SplineOrder; i++) {
          size_t idx = pre_idx_R + i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jac_kont_R(
                jacobians[idx]);
            Eigen::Matrix<double, 1, 3, Eigen::RowMajor> J_temp;
            /// 1*3 3*3
            J_temp = jac_lhs_R[seg] * J_R[seg].d_val_d_knot[i];
            J_temp = (weight_ * J_temp).eval();

            jac_kont_R.block<1, 3>(0, 0) += J_temp;
          }
        }

        /// position control point
        size_t pre_idx_P = P_offset[seg] + J_p[seg].start_idx;
        for (size_t i = 0; i < SplineOrder; i++) {
          size_t idx = pre_idx_P + i;
          if (jacobians[idx]) {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jac_kont_p(
                jacobians[idx]);

            Eigen::Matrix<double, 1, 3, Eigen::RowMajor> J_temp;
            /// 1*1 1*3
            J_temp = J_p[seg].d_val_d_knot[i] * jac_lhs_P[seg];
            J_temp = (weight_ * J_temp).eval();

            jac_kont_p += J_temp;
          }
        }
      }
    }

    return true;
  }

  static void SetParam(SO3d _S_LtoI, Vec3d _p_LinI) {
    init_flag = true;
    S_LtoI = _S_LtoI;
    p_LinI = _p_LinI;

    // std::cout << "[RalativeLoamFeatureFactor] p_LinI: " << p_LinI.transpose()
    //           << "\n";
  }

 private:
  static inline bool init_flag = false;
  static inline SO3d S_LtoI;
  static inline Vec3d p_LinI;

  int64_t t_point_ns_;
  int64_t t_map_ns_;
  PointCorrespondence pc_;
  SplineMeta<SplineOrder> spline_meta_;

  double weight_;
};

}  // namespace analytic_derivative

}  // namespace cocolic
