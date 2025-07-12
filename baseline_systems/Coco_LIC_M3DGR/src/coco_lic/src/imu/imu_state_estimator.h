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

#include <utils/yaml_utils.h>

#include <spline/trajectory.h>
#include <utils/parameter_struct.h>

/////////////

namespace cocolic {

enum MotionState {
  start_motionless = 0,  /// 
  uniform_motion,        /// 
  moving,                /// 
  motionless             /// 
};

// 
class ImuStateEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<ImuStateEstimator> Ptr;

  ImuStateEstimator(const YAML::Node& node);

  void FeedIMUData(const IMUData& imu_data);

  // 
  void Propagate(const IMUState& imu_state, double from_timestamp,
                 double to_timestamp);

  const IMUState& GetPropagateStartState() const {
    return propagate_start_state_;
  }

  const IMUState& GetIMUState() const { return latest_state_; }

  MotionState GetMotionState() const { return motion_state_; }

 private:
  void UpdateMotionState(const IMUState& imu_state,
                         const Eigen::aligned_vector<IMUData>& imu_cache);

  Eigen::aligned_vector<IMUData> imu_data_;

  // 
  IMUState propagate_start_state_;

  /// 
  IMUState latest_state_;

  /// 
  MotionState motion_state_;

  double accel_excite_threshold_;

  double gyro_excite_threshold_;
};

}  // namespace cocolic
