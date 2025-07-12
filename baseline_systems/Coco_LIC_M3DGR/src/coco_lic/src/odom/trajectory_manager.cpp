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

#include <odom/factor/analytic_diff/image_feature_factor.h>
#include <odom/factor/analytic_diff/trajectory_value_factor.h>
#include <odom/trajectory_manager.h>
#include <ros/assert.h>
#include <utils/log_utils.h>

#include <fstream>
std::fstream myfile_t_ba;
namespace cocolic
{

  TrajectoryManager::TrajectoryManager(const YAML::Node &node,
                                       Trajectory::Ptr trajectory)
      : verbose(false),
        cur_img_time_(-1),
        process_cur_img_(false),
        opt_weight_(OptWeight(node)),
        trajectory_(trajectory),
        lidar_marg_info(nullptr),
        cam_marg_info(nullptr)
  {
    std::string config_path = node["config_path"].as<std::string>();
    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);
    imu_state_estimator_ = std::make_shared<ImuStateEstimator>(imu_node);

    if_use_init_bg_ = imu_node["if_use_init_bg"].as<bool>();

    lidar_prior_ctrl_id = std::make_pair(0, 0);

    InitFactorInfo(trajectory_->GetSensorEP(CameraSensor),
                   trajectory_->GetSensorEP(LiDARSensor),
                   opt_weight_.image_weight, opt_weight_.local_velocity_info_vec);

    division_ = 0;
    use_marg_ = true;

    opt_cnt = 0;
    t_opt_sum = 0.0;

    v_points_.clear();
    px_obss_.clear();
  }

  void TrajectoryManager::InitFactorInfo(
      const ExtrinsicParam &Ep_CtoI, const ExtrinsicParam &Ep_LtoI,
      const double image_feature_weight,
      const Eigen::Vector3d &local_velocity_weight)
  {
    if (image_feature_weight > 1e-5)
    {
      Eigen::Matrix2d sqrt_info =
          image_feature_weight * Eigen::Matrix2d::Identity();

      analytic_derivative::ImageFeatureFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
      analytic_derivative::ImageFeatureFactor::sqrt_info = sqrt_info;

      analytic_derivative::Image3D2DFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
      analytic_derivative::Image3D2DFactor::sqrt_info = sqrt_info;

      analytic_derivative::ImageFeatureOnePoseFactor::SetParam(Ep_CtoI.so3,
                                                               Ep_CtoI.p);
      analytic_derivative::ImageFeatureOnePoseFactor::sqrt_info = sqrt_info;

      analytic_derivative::ImageDepthFactor::sqrt_info = sqrt_info;

      analytic_derivative::EpipolarFactor::SetParam(Ep_CtoI.so3, Ep_CtoI.p);
    }
    analytic_derivative::LoamFeatureOptMapPoseFactor::SetParam(Ep_LtoI.so3,
                                                               Ep_LtoI.p);
    analytic_derivative::RalativeLoamFeatureFactor::SetParam(Ep_LtoI.so3,
                                                             Ep_LtoI.p);
  }

  void TrajectoryManager::SetSystemState(const SystemState &sys_state, double distance0)
  {
    gravity_ = sys_state.g;

    SetOriginalPose(sys_state.q, sys_state.p);

    trajectory_->AddKntNs(0.0 * S_TO_NS);       // add knot t3   （t0、t1、t2 have been added in the constructor）
    trajectory_->AddKntNs(distance0 * S_TO_NS); // add knot t4
    trajectory_->SetMaxTimeNsNURBS(trajectory_->knts.back());

    SO3d R0(sys_state.q);
    for (size_t i = 0; i < trajectory_->numKnots(); i++)
    {
      trajectory_->setKnotSO3(R0, i);
    }
    LOG(INFO) << "[debug numKnots] " << trajectory_->numKnots(); // 4

    tparam_.last_bias_time = trajectory_->maxTimeNsNURBS();
    tparam_.cur_bias_time = trajectory_->maxTimeNsNURBS();

    // TODO
    all_imu_bias_[tparam_.last_bias_time] = sys_state.bias;
    if (!if_use_init_bg_)
    {
      all_imu_bias_[tparam_.last_bias_time].gyro_bias = Eigen::Vector3d::Zero();
      all_imu_bias_[tparam_.last_bias_time].accel_bias = Eigen::Vector3d::Zero();
    }
  }

  void TrajectoryManager::SetOriginalPose(Eigen::Quaterniond q,
                                          Eigen::Vector3d p)
  {
    original_pose_.orientation.setQuaternion(q);
    original_pose_.position = p;
  }

  void TrajectoryManager::AddIMUData(const IMUData &data)
  {
    if (trajectory_->GetDataStartTime() < 0)
    {
      trajectory_->SetDataStartTime(data.timestamp);
    }
    imu_data_.emplace_back(data);
    imu_data_.back().timestamp -= trajectory_->GetDataStartTime();

    imu_state_estimator_->FeedIMUData(imu_data_.back());
  }

  void TrajectoryManager::AddPoseData(const PoseData &data)
  {
    pose_data_.emplace_back(data);
    pose_data_.back().timestamp -= trajectory_->GetDataStartTime();
  }

  void TrajectoryManager::RemoveIMUData(int64_t t_window_min)
  {
    if (t_window_min < 0)
      return;

    // https://stackoverflow.com/questions/991335/
    // how-to-erase-delete-pointers-to-objects-stored-in-a-vector
    for (auto iter = imu_data_.begin(); iter != imu_data_.end();)
    {
      if (iter->timestamp < t_window_min)
      {
        iter = imu_data_.erase(iter);
      }
      else
      {
        break;
      }
    }
  }

  void TrajectoryManager::RemovePoseData(int64_t t_window_min)
  {
    if (t_window_min < 0)
      return;

    // https://stackoverflow.com/questions/991335/
    // how-to-erase-delete-pointers-to-objects-stored-in-a-vector
    for (auto iter = pose_data_.begin(); iter != pose_data_.end();)
    {
      if (iter->timestamp < t_window_min)
      {
        iter = pose_data_.erase(iter);
      }
      else
      {
        break;
      }
    }
  }

  void TrajectoryManager::UpdateIMUInlio()
  {
    int64_t t_min = opt_min_t_ns;
    int64_t t_max = opt_max_t_ns;

    for (auto iter = imu_data_.begin(); iter != imu_data_.end(); ++iter)
    {
      if (iter->timestamp >= t_min)
      {
        if (iter->timestamp >= t_max)
        {
          continue;
        }
        tparam_.lio_imu_idx[0] = std::distance(imu_data_.begin(), iter);
        tparam_.lio_imu_time[0] = iter->timestamp;
        break;
      }
    }

    for (auto rter = imu_data_.rbegin(); rter != imu_data_.rend(); ++rter)
    {
      if (rter->timestamp < t_max)
      {
        tparam_.lio_imu_idx[1] =
            std::distance(imu_data_.begin(), rter.base()) - 1;
        tparam_.lio_imu_time[1] = rter->timestamp;
        break;
      }
    }
  }

  void TrajectoryManager::PredictTrajectory(int64_t scan_time_min, int64_t scan_time_max,
                                            int64_t traj_max_time_ns, int knot_add_num, bool non_uniform)
  {
    if (imu_data_.empty() || imu_data_.size() == 1)
    {
      LOG(ERROR) << "[AppendWithIMUData] IMU data empty! ";
      return;
    }

    /// newly added interval：[opt_min_t_ns, opt_max_t_ns)
    opt_min_t_ns = trajectory_->maxTimeNsNURBS();

    /// extend trajectory by adding control points
    trajectory_->SetMaxTimeNsNURBS(traj_max_time_ns);
    opt_max_t_ns = trajectory_->maxTimeNsNURBS();
    SE3d last_knot = trajectory_->getLastKnot();
    trajectory_->extendKnotsTo(knot_add_num, last_knot);

    ////// color control point for visualization
    int intensity = 0;
    if (knot_add_num == 1)
    {
      intensity = 100;
    }
    else if (knot_add_num == 2)
    {
      intensity = 200;
    }
    else if (knot_add_num == 3)
    {
      intensity = 300;
    }
    else if (knot_add_num == 4)
    {
      intensity = 400;
    }
    for (int i = 0; i < knot_add_num; i++)
    {
      trajectory_->intensity_map[trajectory_->numKnots() + i] = intensity;
    }
    ////// color control point for visualization

    LOG(INFO) << "[max_time_ns] " << opt_max_t_ns;
    LOG(INFO) << "[numKnots aft extension] " << trajectory_->numKnots();

    tparam_.last_bias_time = tparam_.cur_bias_time;  // opt_min_t_ns
    tparam_.cur_bias_time = opt_max_t_ns;
    LOG(INFO) << "[last_bias_time] " << tparam_.last_bias_time << " "
              << "[cur_bias_time] " << tparam_.cur_bias_time;
    tparam_.UpdateCurScan(scan_time_min, scan_time_max);
    UpdateIMUInlio();  // determine the imu data involved in this optimization

    /// optimization
    InitTrajWithPropagation();
  }

  void TrajectoryManager::InitTrajWithPropagation()
  {
    TrajectoryEstimatorOptions option;
    option.lock_ab = true;
    option.lock_wb = true;
    option.lock_g = true;
    option.lock_tran = false; // note
    option.show_residual_summary = verbose;
    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option, "Init Traj"));

    estimator->SetFixedIndex(3);

    // [0] prior factor
    if (true && lidar_marg_info)
    {
      estimator->AddMarginalizationFactor(lidar_marg_info,
                                          lidar_marg_parameter_blocks);
    }

    // [1] imu factor
    double *para_bg = all_imu_bias_.rbegin()->second.gyro_bias.data();
    double *para_ba = all_imu_bias_.rbegin()->second.accel_bias.data();
    for (int i = tparam_.lio_imu_idx[0]; i <= tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i).timestamp < opt_min_t_ns)
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)
        continue;
      estimator->AddIMUMeasurementAnalyticNURBS(imu_data_.at(i),
                                                para_bg, para_ba,
                                                gravity_.data(), //(0, 0, 9.8)
                                                opt_weight_.imu_info_vec);
    }

    ceres::Solver::Summary summary = estimator->Solve(50, false);
    static int init_cnt = 0;
    init_cnt++;
    LOG(INFO) << init_cnt << " TrajInitSolver " << summary.BriefReport();
    LOG(INFO) << init_cnt << " TrajInit Successful/Unsuccessful steps: "
              << summary.num_successful_steps << "/"
              << summary.num_unsuccessful_steps;
  }

  bool TrajectoryManager::UpdateTrajectoryWithLIC(
      int lidar_iter, int64_t img_time_stamp,
      const Eigen::aligned_vector<PointCorrespondence> &point_corrs,
      const Eigen::aligned_vector<Eigen::Vector3d> &pnp_3ds,
      const Eigen::aligned_vector<Eigen::Vector2d> &pnp_2ds,
      const int iteration)
  {
    if (point_corrs.empty() || imu_data_.empty() || imu_data_.size() == 1)
    {
      LOG(WARNING) << " input empty data " << point_corrs.size() << ", "
                   << imu_data_.size();
      return false;
    }

    LOG(INFO) << "[point_corrs size] " << point_corrs.size();
    LOG(INFO) << "[opt_domain]: "
              << "[" << opt_min_t_ns * NS_TO_S << ", " << opt_max_t_ns * NS_TO_S << ")";

    IMUBias last_bias = all_imu_bias_.rbegin()->second;
    all_imu_bias_[tparam_.cur_bias_time] = last_bias;
    std::map<int, double *> para_bg_vec;
    std::map<int, double *> para_ba_vec;
    {
      auto &bias0 = all_imu_bias_[tparam_.last_bias_time]; // bi
      para_bg_vec[0] = bias0.gyro_bias.data();
      para_ba_vec[0] = bias0.accel_bias.data();

      auto &bias1 = all_imu_bias_[tparam_.cur_bias_time]; // bj
      para_bg_vec[1] = bias1.gyro_bias.data();
      para_ba_vec[1] = bias1.accel_bias.data();
    }

    TrajectoryEstimatorOptions option;
    option.lock_ab = false;
    option.lock_wb = false;
    option.lock_g = true;
    option.show_residual_summary = verbose;
    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option, "Before LIO"));

    estimator->SetFixedIndex(3);

    // [0] prior factor
    if (true && lidar_marg_info)
    {
      estimator->AddMarginalizationFactor(lidar_marg_info,
                                          lidar_marg_parameter_blocks);
    }

    // [1] lidar factor
    SO3d S_LtoI = trajectory_->GetSensorEP(LiDARSensor).so3;
    Eigen::Vector3d p_LinI = trajectory_->GetSensorEP(LiDARSensor).p;
    SO3d S_GtoM = SO3d(Eigen::Quaterniond::Identity());
    Eigen::Vector3d p_GinM = Eigen::Vector3d::Zero();

    for (const auto &v : point_corrs)
    {
      if (v.t_point < opt_min_t_ns)
        continue;
      if (v.t_point >= opt_max_t_ns)
        continue;
      if (v.t_point < tparam_.last_scan[1])
        continue;
      if (use_lidar_scale)
      {
        estimator->AddLoamMeasurementAnalyticNURBS(v, S_GtoM, p_GinM, S_LtoI, p_LinI,
                                                   opt_weight_.lidar_weight * v.scale);
      }
      else
      {
        estimator->AddLoamMeasurementAnalyticNURBS(v, S_GtoM, p_GinM, S_LtoI, p_LinI,
                                                   opt_weight_.lidar_weight);
      }
    }

    // [2] imu factor
    for (int i = tparam_.lio_imu_idx[0]; i < tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i).timestamp < opt_min_t_ns)
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)
        continue;
      estimator->AddIMUMeasurementAnalyticNURBS(imu_data_.at(i), para_bg_vec[0],
                                                para_ba_vec[0], gravity_.data(),
                                                opt_weight_.imu_info_vec);
    }

    /// [3] bias factor
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> noise_covariance = Eigen::Matrix<double, 6, 6>::Zero();
    noise_covariance.block<3, 3>(0, 0) = (opt_weight_.imu_noise.sigma_wb_discrete * opt_weight_.imu_noise.sigma_wb_discrete) * Eigen::Matrix3d::Identity();
    noise_covariance.block<3, 3>(3, 3) = (opt_weight_.imu_noise.sigma_ab_discrete * opt_weight_.imu_noise.sigma_ab_discrete) * Eigen::Matrix3d::Identity();
    for (int i = tparam_.lio_imu_idx[0] + 1; i < tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i - 1).timestamp < opt_min_t_ns)
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)
        continue;
      double dt = (imu_data_[i].timestamp - imu_data_[i - 1].timestamp) * NS_TO_S;
      Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Zero();
      F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Zero();
      G.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * dt;
      G.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * dt;
      covariance = F * covariance * F.transpose() + G * noise_covariance * G.transpose();
    }

    Eigen::Matrix<double, 6, 6> sqrt_info_mat = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(covariance.inverse()).matrixL().transpose();
    sqrt_info_ << sqrt_info_mat(0, 0), sqrt_info_mat(1, 1), sqrt_info_mat(2, 2), sqrt_info_mat(3, 3), sqrt_info_mat(4, 4), sqrt_info_mat(5, 5);
    estimator->AddBiasFactor(para_bg_vec[0], para_bg_vec[1], para_ba_vec[0],
                             para_ba_vec[1], 1, sqrt_info_);

    /// [4] pnp factor
    v_points_.clear();
    px_obss_.clear();
    if (pnp_3ds.size() != 0)
    {
      v_points_ = pnp_3ds;
      px_obss_ = pnp_2ds;
      process_cur_img_ = true;
      cur_img_time_ = img_time_stamp;
      for (int i = 0; i < pnp_3ds.size(); i++)
      {
        estimator->AddPnPMeasurementAnalyticNURBS(
            pnp_3ds[i], pnp_2ds[i],
            img_time_stamp,
            trajectory_->GetSensorEP(CameraSensor).so3,
            trajectory_->GetSensorEP(CameraSensor).p,
            K_, opt_weight_.image_weight);
      }
    }
    else
    {
      process_cur_img_ = false;
    }

    TicToc t_opt;
    static int loam_cnt = 0;
    ceres::Solver::Summary summary = estimator->Solve(iteration, false);
    double opt_time = t_opt.toc();
    LOG(INFO) << "[t_opt] " << opt_time << std::endl;
    LOG(INFO) << "LoamSolver " << summary.BriefReport();
    LOG(INFO) << ++loam_cnt << " UpdateLio Successful/Unsuccessful steps: "
              << summary.num_successful_steps << "/"
              << summary.num_unsuccessful_steps;

    opt_cnt++;
    t_opt_sum += opt_time;

    LOG(INFO) << "[gyro_bias_new] " << all_imu_bias_.rbegin()->second.gyro_bias.x() << " "
              << all_imu_bias_.rbegin()->second.gyro_bias.y() << " "
              << all_imu_bias_.rbegin()->second.gyro_bias.z();
    LOG(INFO) << "[acce_bias_new] " << all_imu_bias_.rbegin()->second.accel_bias.x() << " "
              << all_imu_bias_.rbegin()->second.accel_bias.y() << " "
              << all_imu_bias_.rbegin()->second.accel_bias.z();

    return true;
  }

  void TrajectoryManager::UpdateLiDARAttribute(double scan_time_min,
                                               double scan_time_max)
  {
    if (trajectory_->maxTimeNsNURBS() > 25 * S_TO_NS)
    {
      int64_t t = trajectory_->maxTimeNsNURBS() - 15 * S_TO_NS;
      RemoveIMUData(t);
      RemovePoseData(t);
    }
  }

  void TrajectoryManager::UpdateLICPrior(
      const Eigen::aligned_vector<PointCorrespondence> &point_corrs)
  {
    TrajectoryEstimatorOptions option;
    option.is_marg_state = true;

    TrajectoryEstimator::Ptr estimator(
        new TrajectoryEstimator(trajectory_, option));  // AddControlPoint

    // construct a new prior
    MarginalizationInfo *marginalization_info = new MarginalizationInfo();

    // prepare the control points and biases to be marginalized
    int lhs_idx = trajectory_->numKnots() - 1 - division_ - 2;  // retain the last 3 control points in this optimization; remember, cubic spline is adopted
    int rhs_idx = trajectory_->numKnots() - 4;

    auto &last_bias = all_imu_bias_[tparam_.last_bias_time];  // marginalize the bias bi
    auto &cur_bias = all_imu_bias_[tparam_.cur_bias_time];
    std::vector<double *> drop_param;
    for (int i = lhs_idx; i <= rhs_idx; i++)
    {
      drop_param.emplace_back(trajectory_->getKnotSO3(i).data());
      drop_param.emplace_back(trajectory_->getKnotPos(i).data());
    }
    drop_param.emplace_back(last_bias.gyro_bias.data());
    drop_param.emplace_back(last_bias.accel_bias.data());

    // [0] prior factor marginalization
    if (lidar_marg_info)
    {
      std::vector<int> drop_set;
      for (int i = 0; i < lidar_marg_parameter_blocks.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (lidar_marg_parameter_blocks[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        MarginalizationFactor *cost_function = new MarginalizationFactor(lidar_marg_info);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Prior, cost_function, NULL,
                                                                       lidar_marg_parameter_blocks, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    // [1] imu factor marginalization
    for (int i = tparam_.lio_imu_idx[0]; i < tparam_.lio_imu_idx[1]; ++i)
    {
      if (imu_data_.at(i).timestamp < opt_min_t_ns)
        continue;
      if (imu_data_.at(i).timestamp >= opt_max_t_ns)
        continue;
      int64_t time_ns = imu_data_.at(i).timestamp;
      std::pair<int, double> su; // i u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);
      vec.emplace_back(last_bias.gyro_bias.data());
      vec.emplace_back(last_bias.accel_bias.data());

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        ceres::CostFunction *cost_function = new analytic_derivative::IMUFactorNURBS(
            time_ns, imu_data_.at(i), gravity_, opt_weight_.imu_info_vec, trajectory_->knts, su,
            blending_matrix, cumulative_blending_matrix);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_IMU, cost_function, NULL,
                                                                       vec, drop_set);
        marginalization_info->addResidualBlockInfo(residual_block_info);
      }
    }

    // [2] lidar factor marginalization
    SO3d S_LtoI = trajectory_->GetSensorEP(LiDARSensor).so3;
    Eigen::Vector3d p_LinI = trajectory_->GetSensorEP(LiDARSensor).p;
    SO3d S_GtoM = SO3d(Eigen::Quaterniond::Identity());
    Eigen::Vector3d p_GinM = Eigen::Vector3d::Zero();
    for (const auto &v : point_corrs)
    {
      if (v.t_point < opt_min_t_ns)
        continue;
      if (v.t_point >= opt_max_t_ns)
        continue;
      if (v.t_point < tparam_.last_scan[1])
        continue;
      int64_t time_ns = v.t_point;
      std::pair<int, double> su; // i and u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        double weight = opt_weight_.lidar_weight;
        if (use_lidar_scale)
        {
          weight *= v.scale;
        }
        ceres::CostFunction *cost_function = new analytic_derivative::LoamFeatureFactorNURBS(
            time_ns, v, su, blending_matrix, cumulative_blending_matrix,
            S_GtoM, p_GinM, S_LtoI, p_LinI, weight);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_LiDAR, cost_function, NULL,
                                                                       vec, drop_set);
        int num_residuals = cost_function->num_residuals();
        Eigen::MatrixXd residuals;
        residuals.setZero(num_residuals, 1);
        cost_function->Evaluate(vec.data(), residuals.data(), nullptr);
        double dist = (residuals / weight).norm();
        if (dist < 0.05)
        // if (dist < 0.01)
        {
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
      }
    }

    // [3] bias factor marginalization
    std::vector<double *> vec;
    vec.emplace_back(last_bias.gyro_bias.data());
    vec.emplace_back(cur_bias.gyro_bias.data());
    vec.emplace_back(last_bias.accel_bias.data());
    vec.emplace_back(cur_bias.accel_bias.data());

    std::vector<int> drop_set;
    drop_set.emplace_back(0); // bgi
    drop_set.emplace_back(2); // bai

    analytic_derivative::BiasFactor *cost_function = new analytic_derivative::BiasFactor(1, sqrt_info_);
    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Bias, cost_function, NULL,
                                                                   vec, drop_set);
    marginalization_info->addResidualBlockInfo(residual_block_info);

    /// [4] pnp factor marginalization
    if (process_cur_img_ && v_points_.size() != 0)
    {
      int64_t time_ns = cur_img_time_;
      std::pair<int, double> su; // i和u
      trajectory_->GetIdxT(time_ns, su);
      Eigen::Matrix4d blending_matrix = trajectory_->blending_mats[su.first - 3];
      Eigen::Matrix4d cumulative_blending_matrix = trajectory_->cumu_blending_mats[su.first - 3];
      std::vector<double *> vec;
      estimator->AddControlPointsNURBS(su.first - 3, vec);
      estimator->AddControlPointsNURBS(su.first - 3, vec, true);

      std::vector<int> drop_set;
      for (int i = 0; i < vec.size(); i++)
      {
        for (auto const &dp : drop_param)
        {
          if (vec[i] == dp)
          {
            drop_set.emplace_back(i);
            break;
          }
        }
      }

      if (!drop_set.empty())
      {
        Eigen::Matrix3d K;
        for (int i = 0; i < v_points_.size(); i++)
        {
          ceres::CostFunction *cost_function = new analytic_derivative::PnPFactorNURBS(
              time_ns, su,
              blending_matrix, cumulative_blending_matrix,
              v_points_[i], px_obss_[i],
              trajectory_->GetSensorEP(CameraSensor).so3,
              trajectory_->GetSensorEP(CameraSensor).p,
              K_, opt_weight_.image_weight);
          ceres::LossFunction *loss_function = NULL;
          loss_function = new ceres::CauchyLoss(10.0); // adopted from vins-mono
          ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(RType_Image, cost_function, loss_function,
                                                                         vec, drop_set);
          marginalization_info->addResidualBlockInfo(residual_block_info);
        }
      }
    }

    marginalization_info->preMarginalize();
    marginalization_info->marginalize();
    if (lidar_marg_info)
    {
      lidar_marg_info = nullptr;
    }
    lidar_marg_info.reset(marginalization_info);
    lidar_marg_parameter_blocks = marginalization_info->getParameterBlocks();
  }

} // namespace cocolic
