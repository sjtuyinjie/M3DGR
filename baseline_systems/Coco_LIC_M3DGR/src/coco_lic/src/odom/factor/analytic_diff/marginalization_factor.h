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
#include <pthread.h>
#include <unordered_map>

#include <utils/tic_toc.h>
#include <utils/eigen_utils.hpp>

const int NUM_THREADS = 4;

enum ResidualType {
  RType_Pose = 0,
  RType_IMU,
  RType_Bias,
  RType_Gravity,
  RType_LiDAR,
  RType_LiDAR_Relative,
  RType_LiDAROpt,
  RType_PreIntegration,
  RType_GlobalVelocity,
  RType_LocalVelocity,
  RType_Local6DoFVel,
  RType_Image,
  RType_Epipolar,
  RType_Prior
};

const std::string ResidualTypeStr[] = {
    "Pose           ",  //
    "IMU            ",  //
    "Bias           ",  //
    "Gravity        ",  //
    "LiDAR          ",  //
    "LiDAR_Relative ",  //
    "LiDAROptMap    ",  //
    "PreIntegration ",  //
    "GlobalVelocity ",  //
    "LocalVelocity  ",  //
    "Local6DoFVel   ",  //
    "Image          ",  //
    "Epipolar       ",  //
    "Prior          "   //
};

// 
struct ResidualBlockInfo {
  ResidualBlockInfo(ResidualType _residual_type,
                    ceres::CostFunction *_cost_function,
                    ceres::LossFunction *_loss_function,
                    std::vector<double *> _parameter_blocks,
                    std::vector<int> _drop_set)
      : residual_type(_residual_type),
        cost_function(_cost_function),
        loss_function(_loss_function),
        parameter_blocks(_parameter_blocks),
        drop_set(_drop_set) {}

  // 
  void Evaluate();

  ResidualType residual_type;

  ceres::CostFunction *cost_function;
  ceres::LossFunction *loss_function;
  std::vector<double *> parameter_blocks;  // 
  std::vector<int> drop_set;               // 

  // 
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      jacobians;
  Eigen::VectorXd residuals;

  int localSize(int size) { return size == 4 ? 3 : size; }
};

struct ThreadsStruct {
  std::vector<ResidualBlockInfo *> sub_factors;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::unordered_map<long, int> parameter_block_size;  // parameter size
  std::unordered_map<long, int> parameter_block_idx;   // position in H matrix
};

// 
// 
class MarginalizationInfo {
 public:
  typedef std::shared_ptr<MarginalizationInfo> Ptr;

  ~MarginalizationInfo();

  int localSize(int size) const;

  // 
  void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);

  // [1] 
  // [2] 
  void preMarginalize();

  // 
  bool marginalize();

  // 
  std::vector<double *> getParameterBlocks(
      std::unordered_map<long, double *> &addr_shift);

  // 
  std::vector<double *> getParameterBlocks();

  // 
  std::vector<ResidualBlockInfo *> factors;

  // m:  localsize
  // n:  localsize
  int m, n;

  // 
  std::unordered_map<long, int> parameter_block_size;

  // 
  // 
  // 
  std::unordered_map<long, int> parameter_block_idx;

  // 
  std::unordered_map<long, double *> parameter_block_data;

  // 
  std::vector<int> keep_block_size;
  // 
  std::vector<int> keep_block_idx;
  // 
  std::vector<double *> keep_block_data;

  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-15;
};

class MarginalizationFactor : public ceres::CostFunction {
 public:
  //
  MarginalizationFactor(MarginalizationInfo::Ptr& _marginalization_info);
  // 
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;

  MarginalizationInfo* marginalization_info;
};