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

#include <imu/imu_state_estimator.h>
#include <ros/assert.h>
#include <utils/log_utils.h>

namespace cocolic {

ImuStateEstimator::ImuStateEstimator(const YAML::Node& node)
    : motion_state_(MotionState::start_motionless) {
  accel_excite_threshold_ =
      yaml::GetValue<double>(node, "accel_excite_threshold", 0.5);
  gyro_excite_threshold_ =
      yaml::GetValue<double>(node, "gyro_excite_threshold", 0.5);
}

void ImuStateEstimator::FeedIMUData(const IMUData& imu_data) {
  imu_data_.push_back(imu_data);
}

void ImuStateEstimator::Propagate(const IMUState& imu_state,
                                  double from_timestamp, double to_timestamp) {
  if (imu_data_.empty()) {
    LOG(WARNING) << " no imu data !";
  }
  if (imu_data_.front().timestamp > to_timestamp) {
    LOG(WARNING) << " input time too small." << to_timestamp << "|"
                 << imu_data_.front().timestamp;
  }

  Eigen::aligned_vector<IMUData> imu_cache;
  auto iter = imu_data_.begin();
  while (iter != imu_data_.end()) {
    // 
    if (next(iter)->timestamp < from_timestamp) {
      iter = imu_data_.erase(iter);
    } else if (iter->timestamp < to_timestamp) {
      imu_cache.push_back(*iter);
      iter++;
    } else
      break;
  }

  if (imu_cache.size() < 1) {
    LOG(INFO) << "IMU cache size : " << imu_cache.size();
    return;
  }

  // 
  if (imu_cache[0].timestamp < from_timestamp && imu_cache.size() > 1) {
    IMUData mid_imudata;
    mid_imudata.timestamp = from_timestamp;
    double dt_1 = mid_imudata.timestamp - imu_cache[0].timestamp;
    double dt_2 = imu_cache[1].timestamp - mid_imudata.timestamp;

    assert(dt_1 + dt_2 > 0);
    double w2 = dt_1 / (dt_1 + dt_2);
    double w1 = dt_2 / (dt_1 + dt_2);
    // 
    if (dt_2 != 0) {
      mid_imudata.accel = w1 * imu_cache[0].accel + w2 * imu_cache[1].accel;
      mid_imudata.gyro = w1 * imu_cache[0].gyro + w2 * imu_cache[1].gyro;
      imu_cache[0] = mid_imudata;
    }
  }

  IMUState x_now = imu_state;
  if (motion_state_ != MotionState::start_motionless) {
    // 
    IMUBias bias = x_now.bias;  // 
    for (size_t i = 1; i < imu_cache.size(); i++) {
      IMUData last_imu = imu_cache[i - 1];

      double dt = imu_cache[i].timestamp - last_imu.timestamp;
      x_now.timestamp = imu_cache[i].timestamp;
      Eigen::Vector3d un_acc_0 =
          x_now.q * (last_imu.accel - bias.accel_bias) - x_now.g;
      Eigen::Vector3d un_gyro =
          0.5 * (last_imu.gyro + imu_cache[i].gyro) - bias.gyro_bias;
      x_now.q *= Eigen::Quaterniond(1, un_gyro(0) * dt / 2, un_gyro(1) * dt / 2,
                                    un_gyro(2) * dt / 2)
                     .normalized();
      Eigen::Vector3d un_acc_1 =
          x_now.q * (imu_cache[i].accel - bias.accel_bias) - x_now.g;
      Eigen::Vector3d un_accel = 0.5 * (un_acc_0 + un_acc_1);
      x_now.p += x_now.v * dt + 0.5 * un_accel * dt * dt;
      x_now.v += un_accel * dt;
    }
  }
  latest_state_ = x_now;
  latest_state_.timestamp = imu_cache.back().timestamp;

  propagate_start_state_ = imu_state;
  propagate_start_state_.timestamp = imu_cache.front().timestamp;

  LOG(INFO) << "[IMU Propagate] desire time [" << from_timestamp << ", "
            << to_timestamp << "]; actual time [" << imu_cache.front().timestamp
            << ", " << imu_cache.back().timestamp << "]";
  LOG(INFO) << "[IMU Propagate] start pose: "
            << VectorToString(propagate_start_state_.p)
            << "; end pose: " << VectorToString(latest_state_.p);

  UpdateMotionState(imu_state, imu_cache);
}

// 
void ImuStateEstimator::UpdateMotionState(
    const IMUState& imu_state,
    const Eigen::aligned_vector<IMUData>& imu_cache) {
  Eigen::Vector3d accel_avg(0, 0, 0);
  Eigen::Vector3d gyro_avg(0, 0, 0);
  for (auto const& v : imu_cache) {
    accel_avg += v.accel;
    gyro_avg += v.gyro;
  }
  accel_avg /= (double)imu_cache.size();
  gyro_avg /= (double)imu_cache.size();

  double accel_var = 0;
  double gyro_var = 0;
  for (size_t i = 0; i < imu_cache.size(); i++) {
    accel_var +=
        (imu_cache[i].accel - accel_avg).dot(imu_cache[i].accel - accel_avg);
    gyro_var +=
        (imu_cache[i].gyro - gyro_avg).dot(imu_cache[i].gyro - gyro_avg);
  }
  accel_var = std::sqrt(accel_var / ((double)imu_cache.size() - 1));
  gyro_var = std::sqrt(gyro_var / ((double)imu_cache.size() - 1));

  if (motion_state_ == MotionState::start_motionless) {
    if (accel_var >= accel_excite_threshold_)
      motion_state_ = MotionState::moving;
  } else {
    // 
    // Eigen::Vector3d velo_avg = Eigen::Vector3d::Zero();
    // for (auto const& x : integrate_imu_state_) {
    //   velo_avg += x.v;
    // }
    // velo_avg /= (double)integrate_imu_state_.size();

    // if (gyro_avg.norm() < 0.01 && velo_avg.norm() < 0.12) {
    //   motion_state_ = MotionState::motionless;
    //   std::cout << "motionless at  " << imu_cache.back().timestamp <<
    //   std::endl;
    // } else {
    //   motion_state_ = MotionState::moving;
    // }

    // LOG(INFO) << "[UpdateMotionState] time: [" << imu_cache.front().timestamp
    //           << ", " << imu_cache.back().timestamp
    //           << "]; gyro_avg: " << gyro_avg.norm()
    //           << ", velo_avg: " << velo_avg.norm();
  }
}

}  // namespace cocolic
