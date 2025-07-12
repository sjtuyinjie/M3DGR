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

#ifndef AUTO_DIFF_LIDAR_FEATURE_FACTOR
#define AUTO_DIFF_LIDAR_FEATURE_FACTOR

#include <basalt/spline/ceres_spline_helper_jet.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
// #include <feature/lidar_feature.h>
// #include <sensor_data/imu_data.h>
#include <Eigen/Core>
#include <memory>
#include <sophus/so3.hpp>

namespace curvefitter {

struct PoseData {
  PoseData()
      : timestamp(0),
        position(Eigen::Vector3d(0, 0, 0)),
        orientation(Sophus::SO3d(Eigen::Quaterniond::Identity())) {}

  double timestamp;
  Eigen::Vector3d position;
  Sophus::SO3d orientation;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using namespace basalt;

template <int _N>
class LiDARPoseFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LiDARPoseFactor(const PoseData& pose_data, const SplineMeta<_N>& spline_meta,
                  double pos_weight, double rot_weight)
      : pose_data_(pose_data),
        spline_meta_(spline_meta),
        pos_weight_(pos_weight),
        rot_weight_(rot_weight) {
    inv_dt_ = 1.0 / spline_meta_.segments.begin()->dt;
  }

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Vec6T = Eigen::Matrix<T, 6, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    // size_t Kont_offset = 2 * spline_meta_.NumParameters();

    // T t_offset = sKnots[Kont_offset + 2][0];
    T t = T(pose_data_.timestamp); // + t_offset;

    size_t R_offset;  // should be zero if not estimate time offset
    size_t P_offset;
    T u;
    // spline_meta_.ComputeSplineIndex(t, R_offset, u);
    P_offset = spline_meta_.NumParameters();

    SO3T R_IkToG;
    CeresSplineHelperJet<T, _N>::template evaluate_lie<Sophus::SO3>(
        sKnots, u, inv_dt_, &R_IkToG);

    Vec3T p_IkinG;
    CeresSplineHelperJet<T, _N>::template evaluate<3, 0>(sKnots + P_offset, u,
                                                         inv_dt_, &p_IkinG);

    // Eigen::Map<SO3T const> const R_LtoI(sKnots[Kont_offset]);
    // Eigen::Map<Vec3T const> const p_LinI(sKnots[Kont_offset + 1]);

    // SO3T R_LkToG = R_IkToG * R_LtoI;
    // Vec3T p_LkinG = R_IkToG * p_LinI + p_IkinG;

    Eigen::Map<Vec6T> residuals(sResiduals);
    residuals.template block<3, 1>(0, 0) =
        T(rot_weight_) * (R_IkToG * pose_data_.orientation.inverse()).log();

    residuals.template block<3, 1>(3, 0) =
        T(pos_weight_) * (p_IkinG - pose_data_.position);
    return true;
  }

 private:
  PoseData pose_data_;
  SplineMeta<_N> spline_meta_;
  double pos_weight_;
  double rot_weight_;
  double inv_dt_;
};

template <int _N>
class SO3KnotFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SO3KnotFactor(const Sophus::SO3d& so3_knots, double so3_knot_weight)
      : so3_knots_(so3_knots), so3_knot_weight_(so3_knot_weight) {}

  template <class T>
  bool operator()(T const* const* sKnots, T* sResiduals) const {
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using SO3T = Sophus::SO3<T>;
    using Tangent = typename Sophus::SO3<T>::Tangent;

    Eigen::Map<SO3T const> const knot(sKnots[0]);
    Eigen::Map<SO3T const> const R_LtoI(sKnots[1]);

    Eigen::Map<Tangent> residuals(sResiduals);
    residuals =
        T(so3_knot_weight_) * ((knot * R_LtoI * so3_knots_.inverse()).log());

    return true;
  }

 private:
  Sophus::SO3d so3_knots_;
  double so3_knot_weight_;
};

}  // namespace clins

#endif
