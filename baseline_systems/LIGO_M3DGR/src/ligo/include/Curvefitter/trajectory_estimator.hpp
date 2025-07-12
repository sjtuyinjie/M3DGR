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

#ifndef TRAJECTORY_ESTIMATOR_HPP
#define TRAJECTORY_ESTIMATOR_HPP

#include <ceres/ceres.h>
#include <ceres/covariance.h>
// #include <factor/auto_diff/imu_factor.h>
#include <Curvefitter/lidar_feature_factor.h>
// #include <factor/auto_diff/loop_closure_factor.h>
// #include <factor/auto_diff/motion_factor.h>
// #include <factor/auto_diff/vicon_factor.h>
#include <utils/ceres_callbacks.h>
#include <basalt/spline/ceres_local_param.hpp>
#include <memory>
#include <thread>
#include <Curvefitter/se3_trajectory.hpp>

// #include <feature/lidar_feature.h>
// #include <sensor_data/imu_data.h>
// #include <sensor_data/loop_closure_data.h>
// #include <sensor_data/calibration.hpp>

namespace curvefitter {

template <int _N>
class TrajectoryEstimator {
  static ceres::Problem::Options DefaultProblemOptions() {
    ceres::Problem::Options options;
    options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    return options;
  }

 public:
  TrajectoryEstimator(std::shared_ptr<Trajectory<_N>> trajectory) //,
                    //   CalibParamManager::Ptr calib_param)
      : trajectory_(trajectory), traj_locked_(false) {
    problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());
    local_parameterization = new basalt::LieLocalParameterization<Sophus::SO3<double> >();
  }

  void SetProblem(std::shared_ptr<ceres::Problem> problem_in) {
    problem_ = problem_in;
  }

  void SetTrajectory(std::shared_ptr<Trajectory<_N>> trajectory_in) {
    trajectory_ = trajectory_in;
  }

  void AddControlPoints(const SplineMeta<_N>& spline_meta, // can be used
                        std::vector<double*>& vec, bool addPosKont = false);

//   void AddLoamMeasurement(const PointCorrespondence& pc, double weight,
                        //   double huber_loss = 5);

  void AddLiDARPoseMeasurement(const PoseData& pose_data, double rot_weight, // can be used
                               double pos_weight);

  void SetTrajectorControlPointVariable(double min_time, double max_time);

//   void SetKeyScanConstant(double max_time);

  ceres::Solver::Summary Solve(int max_iterations = 50, bool progress = true,
                               int num_threads = -1);

//   bool getCovariance();

  bool IsLocked() const { return traj_locked_; }

  void LockTrajectory(bool lock) { traj_locked_ = lock; }

 private:
//   CalibParamManager::Ptr calib_param_;

  std::shared_ptr<Trajectory<_N>> trajectory_;
  std::shared_ptr<ceres::Problem> problem_;
  // ceres::Manifold* local_parameterization;
  basalt::LieLocalParameterization<Sophus::SO3d>* local_parameterization;

  bool traj_locked_;

  int fixed_control_point_index_ = -1;

  bool callback_needs_state_;
  std::vector<std::unique_ptr<ceres::IterationCallback>> callbacks_;
};

template <int _N>
void TrajectoryEstimator<_N>::AddControlPoints(
    const SplineMeta<_N>& spline_meta, std::vector<double*>& vec,
    bool addPosKont) {
  for (auto const& seg : spline_meta.segments) {
    // size_t start_idx = trajectory_->computeTIndex(seg.t0 + 1e-9).second;
    size_t start_idx = trajectory_->computeTIndex(seg.t0).second;
    for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i) {
      if (addPosKont) {
        vec.emplace_back(trajectory_->getKnotPos(i).data());
        problem_->AddParameterBlock(trajectory_->getKnotPos(i).data(), 3);
      } else {
        vec.emplace_back(trajectory_->getKnotSO3(i).data());
        problem_->AddParameterBlock(trajectory_->getKnotSO3(i).data(), 4, //
                                    local_parameterization);
      }
      // if (IsLocked() || (fixed_control_point_index_ >= 0 &&
      //                    i <= fixed_control_point_index_)) {
      //   problem_->SetParameterBlockConstant(vec.back());
      // }
    }
  }
}

template <int _N>
void TrajectoryEstimator<_N>::AddLiDARPoseMeasurement(const PoseData& pose_data,
                                                      double rot_weight,
                                                      double pos_weight) {
  SplineMeta<_N> spline_meta;
  trajectory_->CaculateSplineMeta({{pose_data.timestamp, pose_data.timestamp}},
                                  spline_meta);

  using Functor = LiDARPoseFactor<_N>;
  Functor* functor =
      new Functor(pose_data, spline_meta, rot_weight, pos_weight);
  auto* cost_function =
      new ceres::DynamicAutoDiffCostFunction<Functor>(functor);

  /// add so3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(4);
  }
  /// add vec3 knots
  for (int i = 0; i < spline_meta.NumParameters(); i++) {
    cost_function->AddParameterBlock(3);
  }
  std::vector<double*> vec;
  AddControlPoints(spline_meta, vec);
  AddControlPoints(spline_meta, vec, true);

  cost_function->SetNumResiduals(6);
  problem_->AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), vec);
}

template <int _N>
void TrajectoryEstimator<_N>::SetTrajectorControlPointVariable(
    double min_time, double max_time) {
  size_t min_s = trajectory_->computeTIndex(min_time).second;
  size_t max_s = trajectory_->computeTIndex(max_time).second;

  std::cout << "[SetTrajectorControlPointVariable]: " << min_s << ", "
            << max_s + _N - 1 << "\n";
  for (size_t i = min_s; i < (max_s + _N); i++) {
    problem_->AddParameterBlock(trajectory_->getKnotSO3(i).data(), 4,
                                local_parameterization);
    problem_->SetParameterBlockVariable(trajectory_->getKnotSO3(i).data());

    problem_->AddParameterBlock(trajectory_->getKnotPos(i).data(), 3);
    problem_->SetParameterBlockVariable(trajectory_->getKnotPos(i).data());
  }
}

template <int _N>
ceres::Solver::Summary TrajectoryEstimator<_N>::Solve(int max_iterations,
                                                      bool progress,
                                                      int num_threads) {
  ceres::Solver::Options options;

  options.minimizer_type = ceres::TRUST_REGION;
  //  options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  //  options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  //    options.trust_region_strategy_type = ceres::DOGLEG;
  //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

  //    options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  options.minimizer_progress_to_stdout = progress;

  if (num_threads < 1) {
    num_threads = std::thread::hardware_concurrency();
  }
  options.num_threads = num_threads;
  options.max_num_iterations = max_iterations;

  if (callbacks_.size() > 0) {
    for (auto& cb : callbacks_) {
      options.callbacks.push_back(cb.get());
    }

    if (callback_needs_state_) options.update_state_every_iteration = true;
  }

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem_.get(), &summary);

  // update state
//   calib_param_->UpdateExtrinicParam();
//   calib_param_->UpdateGravity();
  //  getCovariance();
  return summary;
}
}  // namespace curvefitter

#endif
