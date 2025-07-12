
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

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <ceres/dynamic_cost_function.h>

#include <odom/trajectory_estimator.h>
#include <utils/ceres_callbacks.h>

#include <iostream>
#include <memory>
#include <thread>
#include <variant>

namespace cocolic
{

  using namespace opt_param;

  TrajectoryEstimator::TrajectoryEstimator(Trajectory::Ptr trajectory,
                                           TrajectoryEstimatorOptions &option,
                                           std::string descri)
      : trajectory_(trajectory), fixed_control_point_index_(-1)
  {
    this->options = option;
    problem_ = std::make_shared<ceres::Problem>(DefaultProblemOptions());

    residual_summary_.descri_info = descri;


    auto_diff_local_parameterization_ = nullptr;
    analytic_local_parameterization_ =
        new LieAnalyticLocalParameterization<SO3d>();

    // For gravity
    homo_vec_local_parameterization_ =
        new ceres::HomogeneousVectorParameterization(3);

    marginalization_info_ = std::make_shared<MarginalizationInfo>(); // nullptr

    for (auto &ep : trajectory_->GetSensorEPs())
    {
      t_offset_ns_opt_params_[ep.first] = &ep.second.t_offset_ns;
    }
  }

  void TrajectoryEstimator::AddControlPointsNURBS(
      size_t start_idx, std::vector<double *> &vec, bool addPosKnot)
  {
    for (size_t i = start_idx; i < start_idx + 4; ++i)
    {
      if (addPosKnot)
      {
        vec.emplace_back(trajectory_->getKnotPos(i).data());
        problem_->AddParameterBlock(vec.back(), 3);
        if (options.lock_tran)
        {
          problem_->SetParameterBlockConstant(vec.back());
        }
      }
      else
      {
        vec.emplace_back(trajectory_->getKnotSO3(i).data());
        problem_->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
      }
      if (options.lock_traj || (fixed_control_point_index_ >= 0 &&
                                i <= size_t(fixed_control_point_index_)))
      {
        problem_->SetParameterBlockConstant(vec.back());
      }
    }
  }

  void TrajectoryEstimator::AddControlPointsNURBS(
      size_t start_idx1, size_t start_idx2, std::vector<double *> &vec, bool addPosKnot)
  {
    if (start_idx2 - start_idx1 < 4)
    {
      for (size_t i = start_idx1; i < start_idx2 + 4; ++i)
      {
        if (addPosKnot)
        {
          vec.emplace_back(trajectory_->getKnotPos(i).data());
          problem_->AddParameterBlock(vec.back(), 3);
          if (options.lock_tran)
          {
            problem_->SetParameterBlockConstant(vec.back());
          }
        }
        else
        {
          vec.emplace_back(trajectory_->getKnotSO3(i).data());
          problem_->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
        }
        if (options.lock_traj || (fixed_control_point_index_ >= 0 &&
                                  i <= size_t(fixed_control_point_index_)))
        {
          problem_->SetParameterBlockConstant(vec.back());
        }
      }
    }
    else
    {
      for (size_t i = start_idx1; i < start_idx1 + 4; ++i)
      {
        if (addPosKnot)
        {
          vec.emplace_back(trajectory_->getKnotPos(i).data());
          problem_->AddParameterBlock(vec.back(), 3);
        }
        else
        {
          vec.emplace_back(trajectory_->getKnotSO3(i).data());
          problem_->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
        }
        if (options.lock_traj || (fixed_control_point_index_ >= 0 &&
                                  i <= size_t(fixed_control_point_index_)))
        {
          problem_->SetParameterBlockConstant(vec.back());
        }
      }
      for (size_t i = start_idx2; i < start_idx2 + 4; ++i)
      {
        if (addPosKnot)
        {
          vec.emplace_back(trajectory_->getKnotPos(i).data());
          problem_->AddParameterBlock(vec.back(), 3);
        }
        else
        {
          vec.emplace_back(trajectory_->getKnotSO3(i).data());
          problem_->AddParameterBlock(vec.back(), 4, analytic_local_parameterization_);
        }
        if (options.lock_traj || (fixed_control_point_index_ >= 0 &&
                                  i <= size_t(fixed_control_point_index_)))
        {
          problem_->SetParameterBlockConstant(vec.back());
        }
      }
    }
  }

  void TrajectoryEstimator::AddMarginalizationFactor(
      MarginalizationInfo::Ptr &last_marginalization_info,
      std::vector<double *> &last_marginalization_parameter_blocks)
  {
    MarginalizationFactor *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem_->AddResidualBlock(marginalization_factor, NULL,
                               last_marginalization_parameter_blocks);

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualInfo(RType_Prior, marginalization_factor,
                                        last_marginalization_parameter_blocks);
    }
  }

  void TrajectoryEstimator::AddIMUMeasurementAnalyticNURBS(
      const IMUData &imu_data, double *gyro_bias, double *accel_bias,
      double *gravity, const Eigen::Matrix<double, 6, 1> &info_vec,
      bool marg_this_factor)
  {
    int64_t time_ns = imu_data.timestamp;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    // std::cout << "[time_ns | maxTime] " << time_ns << " " << trajectory_->maxTimeNsNURBS() << std::endl;

    Eigen::Vector3d gw(gravity[0], gravity[1], gravity[2]);
    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    ceres::CostFunction *cost_function = new analytic_derivative::IMUFactorNURBS(
        time_ns, imu_data, gw, info_vec, trajectory_->knts, su,
        blending_matrix, cumulative_blending_matrix);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);
    vec.emplace_back(gyro_bias);
    vec.emplace_back(accel_bias);

    if (options.lock_wb)
    {
      problem_->AddParameterBlock(gyro_bias, 3);
      problem_->SetParameterBlockConstant(gyro_bias);
    }
    if (options.lock_ab)
    {
      problem_->AddParameterBlock(accel_bias, 3);
      problem_->SetParameterBlockConstant(accel_bias);
    }

    // double cauchy_loss = marg_this_factor ? 3 : 10;
    ceres::LossFunction *loss_function = NULL;
    // [jerry debug]
    if (marg_this_factor)
    {
      loss_function = new ceres::CauchyLoss(10);
    }
    problem_->AddResidualBlock(cost_function, loss_function, vec);
  }

  void TrajectoryEstimator::AddLoamMeasurementAnalyticNURBS(
      const PointCorrespondence &pc, const SO3d &S_GtoM,
      const Eigen::Vector3d &p_GinM, const SO3d &S_LtoI,
      const Eigen::Vector3d &p_LinI, double weight, bool marg_this_factor)
  {
    int64_t time_ns = pc.t_point;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    using Functor = analytic_derivative::LoamFeatureFactorNURBS;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, pc, su,
                    blending_matrix,
                    cumulative_blending_matrix,
                    S_GtoM, p_GinM,
                    S_LtoI, p_LinI, weight);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);

    ceres::LossFunction *loss_function = NULL;
    problem_->AddResidualBlock(cost_function, loss_function, vec);
  }

  void TrajectoryEstimator::AddPnPMeasurementAnalyticNURBS(const Eigen::Vector3d &visual_map_point,
                                                           const Eigen::Vector2d &pixel_obs,
                                                           int64_t cur_img_timestamp,
                                                           const SO3d &S_VtoI, const Eigen::Vector3d &p_VinI, const Eigen::Matrix3d &K,
                                                           double img_weight)
  {
    int64_t time_ns = cur_img_timestamp;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    using Functor = analytic_derivative::PnPFactorNURBS;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, su,
                    blending_matrix, cumulative_blending_matrix,
                    visual_map_point, pixel_obs,
                    S_VtoI, p_VinI, K, img_weight);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);

    ceres::LossFunction *loss_function = NULL;
    loss_function = new ceres::CauchyLoss(10.0); // adopt from vins-mono  //[default: 10.0]
    problem_->AddResidualBlock(cost_function, loss_function, vec);
  }

  void TrajectoryEstimator::AddPhotometricMeasurementAnalyticNURBS(
      // const double &prev_pixel_intensity,
      int patch_size_half, int scale, int level,
      float *patch,
      const Eigen::Vector3d &visual_map_point,
      const cv::Mat &cur_img, int64_t cur_img_timestamp,
      const SO3d &S_VtoI, const Eigen::Vector3d &p_VinI, const Eigen::Matrix3d &K,
      double img_weight)
  {
    int64_t time_ns = cur_img_timestamp;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    using Functor = analytic_derivative::PhotometricFactorNURBS;
    ceres::CostFunction *cost_function =
        new Functor(patch_size_half, scale, level,
                    time_ns, su,
                    blending_matrix, cumulative_blending_matrix,
                    // prev_pixel_intensity,
                    patch,
                    visual_map_point, cur_img,
                    S_VtoI, p_VinI, K, img_weight);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);

    ceres::LossFunction *loss_function = NULL;
    problem_->AddResidualBlock(cost_function, loss_function, vec);
  }

  void TrajectoryEstimator::AddBiasFactor(
      double *bias_gyr_i, double *bias_gyr_j, double *bias_acc_i,
      double *bias_acc_j, double dt, const Eigen::Matrix<double, 6, 1> &info_vec,
      bool marg_this_factor, bool marg_all_bias)
  {
    analytic_derivative::BiasFactor *cost_function =
        new analytic_derivative::BiasFactor(dt, info_vec);

    std::vector<double *> vec;
    vec.emplace_back(bias_gyr_i);
    vec.emplace_back(bias_gyr_j);
    vec.emplace_back(bias_acc_i);
    vec.emplace_back(bias_acc_j);

    if (options.lock_wb)
    {
      problem_->AddParameterBlock(bias_gyr_i, 3);
      problem_->AddParameterBlock(bias_gyr_j, 3);
      problem_->SetParameterBlockConstant(bias_gyr_i);
      problem_->SetParameterBlockConstant(bias_gyr_j);
    }
    if (options.lock_ab)
    {
      problem_->AddParameterBlock(bias_acc_i, 3);
      problem_->AddParameterBlock(bias_acc_j, 3);
      problem_->SetParameterBlockConstant(bias_acc_i);
      problem_->SetParameterBlockConstant(bias_acc_j);
    }

    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  void TrajectoryEstimator::AddPoseMeasurementAnalyticDiffNURBS(const PoseData &pose_data,
                                                                double pos_weight, double rot_weight)
  {
    int64_t time_ns = pose_data.timestamp;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    Eigen::Matrix<double, 6, 1> info_vec;
    info_vec << rot_weight, rot_weight, rot_weight, pos_weight, pos_weight, pos_weight;

    ceres::CostFunction *cost_function = new analytic_derivative::IMUPoseFactorNURBS(
        time_ns, pose_data, info_vec, trajectory_->knts, su,
        blending_matrix, cumulative_blending_matrix);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);

    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  void TrajectoryEstimator::AddRelativePoseMeasurementAnalyticDiffNURBS(const PoseData &pose_data,
                                                                        double pos_weight, double rot_weight)
  {
    int64_t time_ns = pose_data.timestamp;
    std::pair<int, double> su; // i u
    trajectory_->GetIdxT(time_ns, su);

    Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
    Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];

    Eigen::Matrix<double, 6, 1> info_vec;
    info_vec << rot_weight, rot_weight, rot_weight, pos_weight, pos_weight, pos_weight;

    static SE3d pose_init = trajectory_->GetIMUPoseNsNURBS(0);
    ceres::CostFunction *cost_function = new analytic_derivative::IMURelativePoseFactorNURBS(
        time_ns, pose_data, pose_init, info_vec, trajectory_->knts, su,
        blending_matrix, cumulative_blending_matrix);

    std::vector<double *> vec;
    AddControlPointsNURBS(su.first - 3, vec);
    AddControlPointsNURBS(su.first - 3, vec, true);

    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  void TrajectoryEstimator::AddCallback(
      const std::vector<std::string> &descriptions,
      const std::vector<size_t> &block_size, std::vector<double *> &param_block)
  {
    // Add callback for debug
    std::unique_ptr<CheckStateCallback> cb =
        std::make_unique<CheckStateCallback>();
    for (int i = 0; i < (int)block_size.size(); ++i)
    {
      cb->addCheckState(descriptions[i], block_size[i], param_block[i]);
    }

    callbacks_.push_back(std::move(cb));
    // If any callback requires state, the flag must be set
    callback_needs_state_ = true;
  }

  ceres::Solver::Summary TrajectoryEstimator::PreSolve(int max_iterations,
                                                       bool progress,
                                                       int num_threads)
  {
    ceres::Solver::Options options;

    // options.minimizer_type = ceres::TRUST_REGION;
    // options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    // options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;

    //    options.trust_region_strategy_type = ceres::DOGLEG;
    //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

    //    options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    options.minimizer_progress_to_stdout = progress;

    if (num_threads < 1)
    {
      num_threads = 1; // std::thread::hardware_concurrency(); // mine is 8
    }
    options.num_threads = num_threads;
    options.max_num_iterations = max_iterations;

    if (callbacks_.size() > 0)
    {
      for (auto &cb : callbacks_)
      {
        options.callbacks.push_back(cb.get());
      }

      if (callback_needs_state_)
        options.update_state_every_iteration = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_.get(), &summary);

    trajectory_->UpdateExtrinsics();
    // trajectory_->UpdateTimeOffset(t_offset_ns_opt_params_);

    if (this->options.show_residual_summary)
    {
      residual_summary_.PrintSummary(trajectory_->minTimeNs(),
                                     trajectory_->getDtNs(),
                                     fixed_control_point_index_);
    }

    return summary;
  }

  ceres::Solver::Summary TrajectoryEstimator::Solve(int max_iterations,
                                                    bool progress,
                                                    int num_threads)
  {
    ceres::Solver::Options options;

    options.minimizer_type = ceres::TRUST_REGION;
    // options.gradient_tolerance = 0.01 * Sophus::Constants<double>::epsilon();
    // options.function_tolerance = 0.01 * Sophus::Constants<double>::epsilon();

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;

    //    options.trust_region_strategy_type = ceres::DOGLEG;
    //    options.dogleg_type = ceres::SUBSPACE_DOGLEG;

    //    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    options.minimizer_progress_to_stdout = progress;

    if (num_threads < 1)
    {
      num_threads = 1; // std::thread::hardware_concurrency(); // mine is 8
    }
    options.num_threads = num_threads;
    options.max_num_iterations = max_iterations;

    if (callbacks_.size() > 0)
    {
      for (auto &cb : callbacks_)
      {
        options.callbacks.push_back(cb.get());
      }

      if (callback_needs_state_)
        options.update_state_every_iteration = true;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem_.get(), &summary);

    trajectory_->UpdateExtrinsics();
    // trajectory_->UpdateTimeOffset(t_offset_ns_opt_params_);

    if (this->options.show_residual_summary)
    {
      residual_summary_.PrintSummary(trajectory_->minTimeNs(),
                                     trajectory_->getDtNs(),
                                     fixed_control_point_index_);
    }

    return summary;
  }

  /// not used
  void ResidualSummary::AddResidualInfo(ResidualType r_type,
                                        const ceres::CostFunction *cost_function,
                                        const std::vector<double *> &param_vec)
  {
    int num_residuals = cost_function->num_residuals();
    Eigen::MatrixXd residuals;
    residuals.setZero(num_residuals, 1);
    cost_function->Evaluate(param_vec.data(), residuals.data(), nullptr);

    auto &error_sum = err_type_sum[r_type];
    // initial error as 0
    while ((int)error_sum.size() < num_residuals)
    {
      error_sum.push_back(0);
    }
    for (int i = 0; i < num_residuals; i++)
    {
      error_sum[i] += std::fabs(residuals(i, 0));
    }
    err_type_number[r_type]++;

    // if (RType_PreIntegration == r_type) {
    //   auto&& log = COMPACT_GOOGLE_LOG_INFO;
    //   log.stream() << "imu_residuals :";
    //   for (int i = 0; i < num_residuals; i++) {
    //     log.stream() << std::fabs(residuals(i, 0)) << ", ";
    //   }
    //   log.stream() << "\n";
    // }
  }

  void ResidualSummary::PrintSummary(int64_t t0_ns, int64_t dt_ns,
                                     int fixed_ctrl_idx) const
  {
    if (err_type_sum.empty())
      return;

    auto &&log = COMPACT_GOOGLE_LOG_INFO;
    log.stream() << "ResidualSummary :" << descri_info << "\n";
    // look through every residual info
    for (auto typ = RType_Pose; typ <= RType_Prior; typ = ResidualType(typ + 1))
    {
      double num = err_type_number.at(typ);
      if (num > 0)
      {
        log.stream() << "\t- " << ResidualTypeStr[int(typ)] << ": ";
        log.stream() << "num = " << num << "; err_ave = ";

        auto &error_sum = err_type_sum.at(typ);
        if (RType_Prior == typ)
        {
          log.stream() << "(dim = " << error_sum.size() << ") ";
        }
        for (int i = 0; i < (int)error_sum.size(); ++i)
        {
          log.stream() << error_sum[i] / num << ", ";
          if ((i + 1) % 15 == 0)
            log.stream() << "\n\t\t\t\t";
        }
        log.stream() << std::endl;
      }
    }

    for (auto typ = RType_Pose; typ <= RType_Prior; typ = ResidualType(typ + 1))
    {
      double num = err_type_number.at(typ);
      auto const &t_span_ns = err_type_duration.at(typ);
      if (num > 0 && t_span_ns.first > 0)
      {
        log.stream() << "\t- " << ResidualTypeStr[int(typ)] << ": "
                     << GetCtrlString(t_span_ns.first, t_span_ns.second, t0_ns,
                                      dt_ns)
                     << "\n";
      }
    }

    for (int k = 0; k < 2; k++)
    {
      if (k == 0)
        log.stream() << "\t- Pos ctrl       : ";
      else
        log.stream() << "\t- Rot ctrl       : ";

      auto const &knot_span = opt_knot.at(k);
      log.stream() << GetTimeString(knot_span.first, knot_span.second, t0_ns,
                                    dt_ns)
                   << "\n";
    }
    if (fixed_ctrl_idx > 0)
    {
      int64_t time_ns =
          (fixed_ctrl_idx - SplineOrder + 1) * dt_ns + t0_ns + dt_ns;

      log.stream() << "\t- fixed ctrl idx: " << fixed_ctrl_idx << "; opt time ["
                   << time_ns * NS_TO_S << ", ~]\n";
    }
  }

  std::string ResidualSummary::GetTimeString(int64_t knot_min, int64_t knot_max,
                                             int64_t t0_ns, int64_t dt_ns) const
  {
    int64_t t_min_ns = (knot_min - SplineOrder + 1) * dt_ns + t0_ns;
    int64_t t_max_ns = (knot_max - SplineOrder + 1) * dt_ns + t0_ns + (dt_ns);

    std::stringstream ss;
    ss << "[" << t_min_ns * NS_TO_S << ", " << t_max_ns * NS_TO_S << "] in ["
       << knot_min << ", " << knot_max << "]";
    std::string segment_info = ss.str();
    return segment_info;
  }

  std::string ResidualSummary::GetCtrlString(int64_t t_min_ns, int64_t t_max_ns,
                                             int64_t t0_ns, int64_t dt_ns) const
  {
    int64_t knot_min = (t_min_ns - t0_ns) / dt_ns;
    int64_t knot_max = (t_max_ns - t0_ns) / dt_ns + (SplineOrder - 1);
    std::stringstream ss;
    ss << "[" << t_min_ns * NS_TO_S << ", " << t_max_ns * NS_TO_S << "] in ["
       << knot_min << ", " << knot_max << "]";
    std::string segment_info = ss.str();
    return segment_info;
  }

  void TrajectoryEstimator::AddControlPoints(
      const SplineMeta<SplineOrder> &spline_meta, std::vector<double *> &vec,
      bool addPosKnot)
  {
    for (auto const &seg : spline_meta.segments)
    {
      size_t start_idx = trajectory_->GetCtrlIndex(seg.t0_ns);
      for (size_t i = start_idx; i < (start_idx + seg.NumParameters()); ++i)
      {
        if (addPosKnot)
        {
          vec.emplace_back(trajectory_->getKnotPos(i).data());
          problem_->AddParameterBlock(vec.back(), 3);
        }
        else
        {
          vec.emplace_back(trajectory_->getKnotSO3(i).data());
          if (options.use_auto_diff)
          {
            problem_->AddParameterBlock(vec.back(), 4,
                                        auto_diff_local_parameterization_);
          }
          else
          {
            problem_->AddParameterBlock(vec.back(), 4,
                                        analytic_local_parameterization_);
          }
        }
        if (options.lock_traj || (int)i <= fixed_control_point_index_)
        {
          problem_->SetParameterBlockConstant(vec.back());
        }
        residual_summary_.AddKnotIdx(i, addPosKnot);
      }
    }
  }

  void TrajectoryEstimator::PrepareMarginalizationInfo(
      ResidualType r_type, ceres::CostFunction *cost_function,
      ceres::LossFunction *loss_function, std::vector<double *> &parameter_blocks,
      std::vector<int> &drop_set)
  {
    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
        r_type, cost_function, NULL, parameter_blocks, drop_set);
    marginalization_info_->addResidualBlockInfo(residual_block_info);
  }

  void TrajectoryEstimator::PrepareMarginalizationInfo(
      ResidualType r_type, const SplineMeta<SplineOrder> &spline_meta,
      ceres::CostFunction *cost_function, ceres::LossFunction *loss_function,
      std::vector<double *> &parameter_blocks,
      std::vector<int> &drop_set_wo_ctrl_point)
  {
    // add contrl point id to drop set
    std::vector<int> drop_set = drop_set_wo_ctrl_point;
    if (options.ctrl_to_be_opt_later > options.ctrl_to_be_opt_now)
    {
      std::vector<int> ctrl_id;
      trajectory_->GetCtrlIdxs(spline_meta, ctrl_id);
      for (int i = 0; i < (int)ctrl_id.size(); ++i)
      {
        if (ctrl_id[i] < options.ctrl_to_be_opt_later)
        {
          drop_set.emplace_back(i);
          drop_set.emplace_back(i + spline_meta.NumParameters());
        }
      }
    }

    if (drop_set.size() > 0)
    {
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          r_type, cost_function, loss_function, parameter_blocks, drop_set);
      marginalization_info_->addResidualBlockInfo(residual_block_info);
    }
  }

  void TrajectoryEstimator::PrepareMarginalizationInfo(
      ResidualType r_type,
      ceres::CostFunction *cost_function, ceres::LossFunction *loss_function,
      std::vector<double *> &parameter_blocks,
      std::vector<int> &drop_set_w_ctrl_point, bool is_lidar_inertial)
  {
    std::vector<int> drop_set = drop_set_w_ctrl_point;

    if (drop_set.size() > 0)
    {
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
          r_type, cost_function, loss_function, parameter_blocks, drop_set);
      marginalization_info_->addResidualBlockInfo(residual_block_info);
    }
    else
    {
      
    }
  }

  void TrajectoryEstimator::SaveMarginalizationInfo(
      MarginalizationInfo::Ptr &marg_info_out,
      std::vector<double *> &marg_param_blocks_out)
  {
    // prepare the schur complement
    marginalization_info_->preMarginalize();
    bool ret = marginalization_info_->marginalize();

    if (ret)
    {
      marg_info_out = marginalization_info_;
      marg_param_blocks_out = marginalization_info_->getParameterBlocks();
    }
    else
    {
      marg_info_out = nullptr;
      marg_param_blocks_out.clear();
    }
  }

  void TrajectoryEstimator::SetKeyScanConstant(double max_time)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(LiDARSensor, max_time, time_ns))
      return;

    std::pair<double, size_t> max_i_s = trajectory_->computeTIndexNs(time_ns);

    int index = 0;
    if (max_i_s.first < 0.5)
    {
      index = max_i_s.second + SplineOrder - 2;
    }
    else
    {
      index = max_i_s.second + SplineOrder - 1;
    }
    if (fixed_control_point_index_ < index)
    {
      fixed_control_point_index_ = index;
    }

    LOG(INFO) << "fixed_control_point_index: " << index << "/"
              << trajectory_->numKnots() << "; max_time: " << max_time
              << std::endl;
  }

  bool TrajectoryEstimator::MeasuredTimeToNs(const SensorType &sensor_type,
                                             const double &timestamp,
                                             int64_t &time_ns) const
  {
    time_ns = timestamp * S_TO_NS;

    int64_t t_min_traj_ns = trajectory_->minTime(sensor_type) * S_TO_NS;
    int64_t t_max_traj_ns = trajectory_->maxTime(sensor_type) * S_TO_NS;

    if (!options.lock_EPs.at(sensor_type).lock_t_offset)
    {
      // |____|______________________________|____|
      //    t_min                          t_max
      if (time_ns - options.t_offset_padding_ns < t_min_traj_ns ||
          time_ns + options.t_offset_padding_ns >= t_max_traj_ns)
        return false;
    }
    else
    {
      if (time_ns < t_min_traj_ns || time_ns >= t_max_traj_ns)
        return false;
    }

    return true;
  }

  bool TrajectoryEstimator::IsParamUpdated(const double *values) const
  {
    if (problem_->HasParameterBlock(values) &&
        !problem_->IsParameterBlockConstant(const_cast<double *>(values)))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void TrajectoryEstimator::SetTimeoffsetState()
  {
    for (auto &sensor_t : t_offset_ns_opt_params_)
    {
      if (problem_->HasParameterBlock(sensor_t.second))
      {
        if (options.lock_EPs.at(sensor_t.first).lock_t_offset)
          problem_->SetParameterBlockConstant(sensor_t.second);
      }
    }
  }

  void TrajectoryEstimator::AddStartTimePose(const PoseData &pose)
  {
    PoseData pose_temp = pose;
    pose_temp.timestamp = trajectory_->minTime(LiDARSensor);

    double pos_weight = 100;
    double rot_weight = 100;
    Eigen::Matrix<double, 6, 1> info_vec;
    info_vec.head(3) = rot_weight * Eigen::Vector3d::Ones();
    info_vec.tail(3) = pos_weight * Eigen::Vector3d::Ones();
    AddPoseMeasurementAnalytic(pose_temp, info_vec);
  }

  // =========== IMU =========== //
  void TrajectoryEstimator::AddPoseMeasurementAnalytic(
      const PoseData &pose_data, const Eigen::Matrix<double, 6, 1> &info_vec)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(IMUSensor, pose_data.timestamp, time_ns))
      return;
    double *t_offset_ns = t_offset_ns_opt_params_[IMUSensor];
    int64_t t_corrected_ns = time_ns + (*t_offset_ns);

    const auto &option_Ep = options.lock_EPs.at(IMUSensor);
    SplineMeta<SplineOrder> spline_meta;
    if (option_Ep.lock_t_offset)
    {
      trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                      spline_meta);
    }
    else
    {
      trajectory_->CaculateSplineMeta(
          {{t_corrected_ns - options.t_offset_padding_ns,
            t_corrected_ns + options.t_offset_padding_ns}},
          spline_meta);
    }

    ceres::CostFunction *cost_function = new analytic_derivative::IMUPoseFactor(
        time_ns, pose_data, spline_meta.segments.at(0), info_vec);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.push_back(t_offset_ns); // time_offset
    problem_->AddResidualBlock(cost_function, NULL, vec);

    problem_->AddParameterBlock(t_offset_ns, 1);
    if (option_Ep.lock_t_offset)
    {
      problem_->SetParameterBlockConstant(t_offset_ns);
    }
    else
    {
      double t_ns = options.t_offset_padding_ns;
      problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
      problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Pose, t_corrected_ns);
      residual_summary_.AddResidualInfo(RType_Pose, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddIMUMeasurementAnalytic(
      const IMUData &imu_data, double *gyro_bias, double *accel_bias,
      double *gravity, const Eigen::Matrix<double, 6, 1> &info_vec,
      bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(IMUSensor, imu_data.timestamp, time_ns))
    {
      return;
    }
    double *t_offset_ns = t_offset_ns_opt_params_[IMUSensor];
    int64_t t_corrected_ns = time_ns + (*t_offset_ns);

    const auto &option_Ep = options.lock_EPs.at(IMUSensor);
    SplineMeta<SplineOrder> spline_meta;
    if (option_Ep.lock_t_offset)
    {
      trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                      spline_meta);
    }
    else
    {
      trajectory_->CaculateSplineMeta(
          {{t_corrected_ns - options.t_offset_padding_ns,
            t_corrected_ns + options.t_offset_padding_ns}},
          spline_meta);
    }
    // Eigen::Matrix<double, 6, 1> m_info_vec;
    // m_info_vec<< 28, 28, 28, 1, 1, 1;
    ceres::CostFunction *cost_function = new analytic_derivative::IMUFactor(
        time_ns, imu_data, spline_meta.segments.at(0), info_vec);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.emplace_back(gyro_bias);
    vec.emplace_back(accel_bias);
    vec.emplace_back(gravity);
    vec.push_back(t_offset_ns); // time_offset
    problem_->AddParameterBlock(gravity, 3, homo_vec_local_parameterization_);

    if (options.lock_wb)
    {
      problem_->AddParameterBlock(gyro_bias, 3);
      problem_->SetParameterBlockConstant(gyro_bias);
    }
    if (options.lock_ab)
    {
      problem_->AddParameterBlock(accel_bias, 3);
      problem_->SetParameterBlockConstant(accel_bias);
    }
    if (options.lock_g)
    {
      problem_->SetParameterBlockConstant(gravity);
    }

    problem_->AddParameterBlock(t_offset_ns, 1);
    if (option_Ep.lock_t_offset)
    {
      problem_->SetParameterBlockConstant(t_offset_ns);
    }
    else
    {
      double t_ns = options.t_offset_padding_ns;
      problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
      problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
    }

    double cauchy_loss = marg_this_factor ? 3 : 10;
    ceres::LossFunction *loss_function = NULL;
    // loss_function = new ceres::HuberLoss(1.0); // marg factor
    loss_function = new ceres::CauchyLoss(cauchy_loss); // adopt from vins-mono

    if (options.is_marg_state && marg_this_factor)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = 2 * spline_meta.NumParameters();
      // two bias
      if (options.marg_bias_param)
      {
        drop_set_wo_ctrl_point.emplace_back(Knot_size);     // gyro_bias
        drop_set_wo_ctrl_point.emplace_back(Knot_size + 1); // accel_bias
      }
      if (options.marg_gravity_param)
      {
        drop_set_wo_ctrl_point.emplace_back(Knot_size + 2); // gravity
      }
      if (options.marg_t_offset_param)
        drop_set_wo_ctrl_point.emplace_back(Knot_size + 3); // t_offset
      PrepareMarginalizationInfo(RType_IMU, spline_meta, cost_function,
                                 loss_function, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, loss_function, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_IMU, t_corrected_ns);
      residual_summary_.AddResidualInfo(RType_IMU, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddGravityFactor(double *gravity,
                                             const Eigen::Vector3d &info_vec,
                                             bool marg_this_factor)
  {
    Eigen::Map<Eigen::Vector3d const> gravity_now(gravity);

    analytic_derivative::GravityFactor *cost_function =
        new analytic_derivative::GravityFactor(gravity_now, info_vec);

    std::vector<double *> vec;
    vec.emplace_back(gravity);
    problem_->AddParameterBlock(gravity, 3, homo_vec_local_parameterization_);
    if (options.lock_g)
    {
      problem_->SetParameterBlockConstant(gravity);
    }

    if (options.is_marg_state && marg_this_factor)
    {
      std::vector<int> drop_set = {0};
      if (!options.marg_gravity_param)
        drop_set.clear();
      PrepareMarginalizationInfo(RType_Gravity, cost_function, NULL, vec,
                                 drop_set);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualInfo(RType_Gravity, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddRelativeRotationAnalytic(
      double ta, double tb, const SO3d &S_BtoA, const Eigen::Vector3d &info_vec)
  {
    int64_t time_a_ns, time_b_ns;
    if (!MeasuredTimeToNs(IMUSensor, ta, time_a_ns))
      return;
    if (!MeasuredTimeToNs(IMUSensor, tb, time_b_ns))
      return;

    int64_t t_min = std::min(time_a_ns, time_b_ns);
    int64_t t_max = std::max(time_a_ns, time_b_ns);
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{t_min, t_min}, {t_max, t_max}},
                                    spline_meta);

    using Functor = analytic_derivative::RelativeOrientationFactor;
    ceres::CostFunction *cost_function =
        new Functor(S_BtoA, time_a_ns, time_b_ns, spline_meta, info_vec);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    problem_->AddResidualBlock(cost_function, NULL, vec);
  }

  void TrajectoryEstimator::AddLocalVelocityMeasurementAnalytic(
      const double timestamp, const Eigen::Vector3d &local_v, double weight)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(IMUSensor, timestamp, time_ns))
      return;
    double *t_offset_ns = t_offset_ns_opt_params_[IMUSensor];
    int64_t t_corrected_ns = time_ns + (*t_offset_ns);

    const auto &option_Ep = options.lock_EPs.at(IMUSensor);
    SplineMeta<SplineOrder> spline_meta;
    if (option_Ep.lock_t_offset)
    {
      trajectory_->CaculateSplineMeta({{t_corrected_ns, t_corrected_ns}},
                                      spline_meta);
    }
    else
    {
      trajectory_->CaculateSplineMeta(
          {{t_corrected_ns - options.t_offset_padding_ns,
            t_corrected_ns + options.t_offset_padding_ns}},
          spline_meta);
    }

    using Functor = analytic_derivative::LocalVelocityFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, local_v, spline_meta.segments.at(0), weight);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.push_back(t_offset_ns); // time_offset
    problem_->AddResidualBlock(cost_function, NULL, vec);

    problem_->AddParameterBlock(t_offset_ns, 1);
    if (option_Ep.lock_t_offset)
    {
      problem_->SetParameterBlockConstant(t_offset_ns);
    }
    else
    {
      double t_ns = options.t_offset_padding_ns;
      problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
      problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_LocalVelocity, t_corrected_ns);
      residual_summary_.AddResidualInfo(RType_LocalVelocity, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddLocal6DoFVelocityAnalytic(
      const double timestamp, const Eigen::Matrix<double, 6, 1> &local_v,
      const Eigen::Matrix<double, 6, 1> &info_vec)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(IMUSensor, timestamp, time_ns))
      return;

    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    using Functor = analytic_derivative::Local6DoFVelocityFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, local_v, spline_meta.segments.at(0), info_vec);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);

    problem_->AddResidualBlock(cost_function, NULL, vec);

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Local6DoFVel, time_ns);
      residual_summary_.AddResidualInfo(RType_Local6DoFVel, cost_function, vec);
    }
  }

  // =========== LidAR =========== //

  void TrajectoryEstimator::AddLoamMeasurementAnalytic(
      const PointCorrespondence &pc, const SO3d &S_GtoM,
      const Eigen::Vector3d &p_GinM, const SO3d &S_LtoI,
      const Eigen::Vector3d &p_LinI, double weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(LiDARSensor, pc.t_point, time_ns))
      return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    using Functor = analytic_derivative::LoamFeatureFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, pc, spline_meta.segments.at(0), S_GtoM, p_GinM,
                    S_LtoI, p_LinI, weight);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);

    if (options.is_marg_state && marg_this_factor)
    {
      int num_residuals = cost_function->num_residuals();
      Eigen::MatrixXd residuals;
      residuals.setZero(num_residuals, 1);

      cost_function->Evaluate(vec.data(), residuals.data(), nullptr);
      double dist = (residuals / weight).norm();
      if (dist < 0.05)
      {
        std::vector<int> drop_set_wo_ctrl_point;
        PrepareMarginalizationInfo(RType_LiDAR, spline_meta, cost_function, NULL,
                                   vec, drop_set_wo_ctrl_point);
      }
    }
    else
    {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_LiDAR, time_ns);
      residual_summary_.AddResidualInfo(RType_LiDAR, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddRalativeLoamFeatureAnalytic(
      const PointCorrespondence &pc, double weight, bool marg_this_factor)
  {
    int64_t time_map_ns, time_point_ns;
    if (!MeasuredTimeToNs(LiDARSensor, pc.t_map, time_map_ns))
      return;
    if (!MeasuredTimeToNs(LiDARSensor, pc.t_point, time_point_ns))
      return;
    SplineMeta<SplineOrder> spline_meta;

    trajectory_->CaculateSplineMeta(
        {{time_map_ns, time_map_ns}, {time_point_ns, time_point_ns}},
        spline_meta);

    using Functor = analytic_derivative::RalativeLoamFeatureFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_point_ns, time_map_ns, pc, spline_meta, weight);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);

    if (options.is_marg_state && marg_this_factor)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      PrepareMarginalizationInfo(RType_LiDAR_Relative, spline_meta, cost_function,
                                 NULL, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_LiDAR_Relative, time_point_ns);
      residual_summary_.AddResidualInfo(RType_LiDAR_Relative, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddLoamFeatureOptMapPoseAnalytic(
      const PointCorrespondence &pc, double *S_ImtoG, double *p_IminG,
      double weight, bool marg_this_factor)
  {
    int64_t time_ns;
    if (!MeasuredTimeToNs(LiDARSensor, pc.t_point, time_ns))
      return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_ns, time_ns}}, spline_meta);

    using Functor = analytic_derivative::LoamFeatureOptMapPoseFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_ns, pc, spline_meta.segments.at(0), weight);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.emplace_back(S_ImtoG); // Rotation: I_{map} to global
    vec.emplace_back(p_IminG); // Position: I_{map} in global

    problem_->AddParameterBlock(S_ImtoG, 4, analytic_local_parameterization_);
    // problem_->SetParameterBlockConstant(S_ImtoG);

    if (options.is_marg_state && marg_this_factor)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      drop_set_wo_ctrl_point.emplace_back(vec.size() - 2); // map rotation
      drop_set_wo_ctrl_point.emplace_back(vec.size() - 1); // map position
      PrepareMarginalizationInfo(RType_LiDAROpt, spline_meta, cost_function, NULL,
                                 vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, NULL, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_LiDAROpt, time_ns);
      residual_summary_.AddResidualInfo(RType_LiDAROpt, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddImageFeatureAnalytic(
      const double ti, const Eigen::Vector3d &pi, const double tj,
      const Eigen::Vector3d &pj, double *inv_depth, bool fixed_depth,
      bool marg_this_fearure)
  {
    int64_t time_i_ns, time_j_ns;
    if (!MeasuredTimeToNs(CameraSensor, ti, time_i_ns))
      return;
    if (!MeasuredTimeToNs(CameraSensor, tj, time_j_ns))
      return;
    double *t_offset_ns = t_offset_ns_opt_params_[CameraSensor];
    int64_t ti_corrected_ns = time_i_ns + (*t_offset_ns);
    int64_t tj_corrected_ns = time_j_ns + (*t_offset_ns);

    int64_t t_min = std::min(ti_corrected_ns, tj_corrected_ns);
    int64_t t_max = std::max(ti_corrected_ns, tj_corrected_ns);

    const auto &option_Ep = options.lock_EPs.at(CameraSensor);
    SplineMeta<SplineOrder> spline_meta;
    if (option_Ep.lock_t_offset)
    {
      trajectory_->CaculateSplineMeta({{t_min, t_min}, {t_max, t_max}},
                                      spline_meta);
    }
    else
    {
      trajectory_->CaculateSplineMeta({{t_min - options.t_offset_padding_ns,
                                        t_min + options.t_offset_padding_ns},
                                       {t_max - options.t_offset_padding_ns,
                                        t_max + options.t_offset_padding_ns}},
                                      spline_meta);
    }

    using Functor = analytic_derivative::ImageFeatureFactor;
    ceres::CostFunction *cost_function =
        new Functor(time_i_ns, pi, time_j_ns, pj, spline_meta);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.emplace_back(inv_depth);
    vec.emplace_back(t_offset_ns); // time_offset

    if (fixed_depth)
    {
      problem_->AddParameterBlock(inv_depth, 1);
      problem_->SetParameterBlockConstant(inv_depth);
    }
    // 1 / 0.01 = 100m (max)
    //  problem_->AddParameterBlock(inv_depth, 1);
    //  problem_->SetParameterLowerBound(inv_depth, 0, 0.01);

    problem_->AddParameterBlock(t_offset_ns, 1);
    if (option_Ep.lock_t_offset)
    {
      problem_->SetParameterBlockConstant(t_offset_ns);
    }
    else
    {
      double t_ns = options.t_offset_padding_ns;
      problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
      problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
    }

    double cauchy_loss = marg_this_fearure ? 1 : 2;
    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0); // marg factor
    loss_function = new ceres::CauchyLoss(cauchy_loss); // adopt from vins-mono

    if (options.is_marg_state && marg_this_fearure)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = 2 * spline_meta.NumParameters();
      // inverse depth position
      drop_set_wo_ctrl_point.emplace_back(Knot_size);
      if (options.marg_t_offset_param)
        drop_set_wo_ctrl_point.emplace_back(Knot_size + 1); // t_offset
      PrepareMarginalizationInfo(RType_Image, spline_meta, cost_function,
                                 loss_function, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, loss_function, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Image, tj_corrected_ns);
      residual_summary_.AddResidualInfo(RType_Image, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddImage3D2DAnalytic(const double ti,
                                                 const Eigen::Vector3d &pi,
                                                 double *p_G, bool fixed_p_G,
                                                 bool marg_this_fearure)
  {
    int64_t time_i_ns;
    if (!MeasuredTimeToNs(CameraSensor, ti, time_i_ns))
      return;
    double *t_offset_ns = t_offset_ns_opt_params_[CameraSensor];
    int64_t ti_corrected_ns = time_i_ns + (*t_offset_ns);

    const auto &option_Ep = options.lock_EPs.at(CameraSensor);
    SplineMeta<SplineOrder> spline_meta;
    if (option_Ep.lock_t_offset)
    {
      trajectory_->CaculateSplineMeta({{ti_corrected_ns, ti_corrected_ns}},
                                      spline_meta);
    }
    else
    {
      trajectory_->CaculateSplineMeta(
          {{ti_corrected_ns - options.t_offset_padding_ns,
            ti_corrected_ns + options.t_offset_padding_ns}},
          spline_meta);
    }

    using Functor = analytic_derivative::Image3D2DFactor;
    ceres::CostFunction *cost_function = new Functor(time_i_ns, pi, spline_meta);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.emplace_back(p_G);
    vec.emplace_back(t_offset_ns); // time_offset

    if (fixed_p_G)
    {
      problem_->AddParameterBlock(p_G, 3);
      problem_->SetParameterBlockConstant(p_G);
    }
    problem_->AddParameterBlock(t_offset_ns, 1);
    if (option_Ep.lock_t_offset)
    {
      problem_->SetParameterBlockConstant(t_offset_ns);
    }
    else
    {
      double t_ns = options.t_offset_padding_ns;
      problem_->SetParameterLowerBound(t_offset_ns, 0, *t_offset_ns - t_ns);
      problem_->SetParameterUpperBound(t_offset_ns, 0, *t_offset_ns + t_ns);
    }

    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0); // marg factor
    loss_function = new ceres::CauchyLoss(1.0); // adopt from vins-mono

    if (options.is_marg_state && marg_this_fearure)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      int Knot_size = 2 * spline_meta.NumParameters();
      // p_G
      drop_set_wo_ctrl_point.emplace_back(Knot_size);
      if (options.marg_t_offset_param)
        drop_set_wo_ctrl_point.emplace_back(Knot_size + 1); // t_offset
      PrepareMarginalizationInfo(RType_Image, spline_meta, cost_function,
                                 loss_function, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, loss_function, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Image, time_i_ns);
      residual_summary_.AddResidualInfo(RType_Image, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddImageFeatureOnePoseAnalytic(
      const Eigen::Vector3d &p_i, const SO3d &S_IitoG,
      const Eigen::Vector3d &p_IiinG, const double t_j,
      const Eigen::Vector3d &p_j, double *inv_depth, bool fixed_depth,
      bool marg_this_fearure)
  {
    int64_t time_j_ns;
    if (!MeasuredTimeToNs(CameraSensor, t_j, time_j_ns))
      return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_j_ns, time_j_ns}}, spline_meta);

    using Functor = analytic_derivative::ImageFeatureOnePoseFactor;
    ceres::CostFunction *cost_function =
        new Functor(p_i, S_IitoG, p_IiinG, time_j_ns, p_j, spline_meta);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);
    vec.emplace_back(inv_depth);

    if (fixed_depth)
    {
      problem_->AddParameterBlock(inv_depth, 1);
      problem_->SetParameterBlockConstant(inv_depth);
    }

    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0); // adopt from vins-mono

    if (options.is_marg_state && marg_this_fearure)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      // inverse depth position
      drop_set_wo_ctrl_point.emplace_back(vec.size() - 1);
      PrepareMarginalizationInfo(RType_Image, spline_meta, cost_function,
                                 loss_function, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, loss_function, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Image, time_j_ns);
      residual_summary_.AddResidualInfo(RType_Image, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddImageDepthAnalytic(const Eigen::Vector3d &p_i,
                                                  const Eigen::Vector3d &p_j,
                                                  const SO3d &S_CitoCj,
                                                  const Eigen::Vector3d &p_CiinCj,
                                                  double *inv_depth)
  {
    using Functor = analytic_derivative::ImageDepthFactor;
    ceres::CostFunction *cost_function =
        new Functor(p_i, p_j, S_CitoCj, p_CiinCj);

    std::vector<double *> vec;
    vec.emplace_back(inv_depth);

    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0); // adopt from vins-mono

    problem_->AddResidualBlock(cost_function, loss_function, vec);

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualInfo(RType_Image, cost_function, vec);
    }
  }

  void TrajectoryEstimator::AddEpipolarFactorAnalytic(
      const double t_i, const Eigen::Vector3d &x_i, const Eigen::Vector3d &x_k,
      const SO3d &S_GtoCk, const Eigen::Vector3d &p_CkinG, double weight,
      bool marg_this_fearure)
  {
    int64_t time_i_ns;
    if (!MeasuredTimeToNs(CameraSensor, t_i, time_i_ns))
      return;
    SplineMeta<SplineOrder> spline_meta;
    trajectory_->CaculateSplineMeta({{time_i_ns, time_i_ns}}, spline_meta);

    using Functor = analytic_derivative::EpipolarFactor;
    ceres::CostFunction *cost_function =
        new Functor(t_i, x_i, x_k, S_GtoCk, p_CkinG, spline_meta, weight);

    std::vector<double *> vec;
    AddControlPoints(spline_meta, vec);
    AddControlPoints(spline_meta, vec, true);

    ceres::LossFunction *loss_function;
    // loss_function = new ceres::HuberLoss(1.0); // marg factor
    loss_function = new ceres::CauchyLoss(1.0); // adopt from vins-mono

    if (options.is_marg_state && marg_this_fearure)
    {
      std::vector<int> drop_set_wo_ctrl_point;
      PrepareMarginalizationInfo(RType_Epipolar, spline_meta, cost_function,
                                 loss_function, vec, drop_set_wo_ctrl_point);
    }
    else
    {
      problem_->AddResidualBlock(cost_function, loss_function, vec);
    }

    if (options.show_residual_summary)
    {
      residual_summary_.AddResidualTimestamp(RType_Epipolar, time_i_ns);
      residual_summary_.AddResidualInfo(RType_Epipolar, cost_function, vec);
    }
  }
} // namespace cocolic
