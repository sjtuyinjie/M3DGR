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

#include <glog/logging.h>
#include <imu/imu_initializer.h>
#include <iomanip>

namespace cocolic {

IMUInitializer::IMUInitializer(const YAML::Node &node) : initial_done_(false) {
  window_length_ = yaml::GetValue<double>(node, "window_length", 1.0) * 1e9;
  imu_excite_threshold_ =
      yaml::GetValue<double>(node, "imu_excite_threshold", 1.0);

  double gravity_mag = yaml::GetValue<double>(node, "gravity_mag", 9.80);
  gravity_ << 0, 0, gravity_mag;
}

void IMUInitializer::FeedIMUData(const IMUData &imu_data) {
  imu_datas_.push_back(imu_data);

  // auto it0 = imu_datas_.begin();
  // while (it0 != imu_datas_.end() &&
  //        it0->timestamp < imu_data.timestamp - 2 * window_length_) {
  //   it0 = imu_datas_.erase(it0);
  // }
}

bool IMUInitializer::InitialIMUState() {
  if (StaticInitialIMUState()) return true;

  if (ActiveInitialIMUState()) return true;

  return false;
}

bool IMUInitializer::ActiveInitialIMUState() {
  if (imu_datas_.empty()) return false;

  Eigen::Quaterniond q_last = imu_datas_.back().orientation.unit_quaternion();
  if (std::fabs(q_last.x()) > 1e-9) {
    Eigen::Matrix3d R_I0toG = q_last.toRotationMatrix();
    double yaw = R2ypr(R_I0toG).x();
    R_I0toG = (ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_I0toG).eval();

    Eigen::Vector3d g_inI0 = R_I0toG * gravity_;

    imu_state_.q = R_I0toG;
    imu_state_.bias.accel_bias = Eigen::Vector3d::Zero();
    imu_state_.bias.gyro_bias = Eigen::Vector3d::Zero();
    imu_state_.g = gravity_;  // gw

    LOG(INFO) << "[InertialInitializer] ActiveInitialIMUState"
              << "first imu time: " << imu_datas_.front().timestamp
              << "\nimu_datas_: " << imu_datas_.size()
              << "\nAccel bias : " << imu_state_.bias.accel_bias.transpose()
              << "\nGyro bias : " << imu_state_.bias.gyro_bias.transpose()
              << "\ngravity in I0: " << g_inI0.transpose();

    initial_done_ = true;
    return true;
  }

  return false;
}

bool IMUInitializer::StaticInitialIMUState() {
  if (imu_datas_.empty()) return false;

  if (imu_datas_.back().timestamp - imu_datas_.begin()->timestamp <
      window_length_)
    return false;

  Eigen::Vector3d accel_avg(0, 0, 0);
  Eigen::Vector3d gyro_avg(0, 0, 0);

  std::vector<IMUData> imu_cache;
  for (size_t i = imu_datas_.size() - 1; i >= 0; i--) {
    accel_avg += imu_datas_[i].accel;
    gyro_avg += imu_datas_[i].gyro;
    imu_cache.push_back(imu_datas_[i]);
    if (imu_datas_.back().timestamp - imu_datas_[i].timestamp >= window_length_)
      break;
  }
  accel_avg /= (int)imu_cache.size();
  gyro_avg /= (int)imu_cache.size();

  double accel_var = 0;
  for (size_t i = 0; i < imu_cache.size(); i++) {
    accel_var +=
        (imu_cache[i].accel - accel_avg).dot(imu_cache[i].accel - accel_avg);
  }
  accel_var = std::sqrt(accel_var / ((int)imu_cache.size() - 1));

  if (accel_var >= imu_excite_threshold_) {
    LOG(WARNING) << "[IMUInitializer] Dont Move !";
    return false;
  }

  Eigen::Vector3d z_axis = accel_avg / accel_avg.norm();
  // std::cout << MAGENTA << "[accel_avg] " << accel_avg.transpose() << RESET << std::endl;
  // std::cout << MAGENTA << "[z_axis] " << z_axis.transpose() << RESET << std::endl;
  Eigen::Vector3d e_1(1, 0, 0);
  Eigen::Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  Eigen::Matrix<double, 3, 1> y_axis =
      Eigen::SkewSymmetric<double>(z_axis) * x_axis;

  Eigen::Matrix<double, 3, 3> Rot;  //R_GtoI0
  Rot.block<3, 1>(0, 0) = x_axis;
  Rot.block<3, 1>(0, 1) = y_axis;
  Rot.block<3, 1>(0, 2) = z_axis;
  Eigen::Vector3d g_inI0 = Rot * gravity_;

  Eigen::Matrix3d R_I0toG = Rot.inverse();
  // double yaw = R2ypr(R_I0toG).x();
  // LOG(INFO) << "[yaw] " << yaw;
  // R_I0toG = (ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R_I0toG).eval();
  imu_state_.q = R_I0toG;
  imu_state_.bias.accel_bias = accel_avg - g_inI0;
  imu_state_.bias.gyro_bias = gyro_avg;
  imu_state_.g = gravity_;  // gw

  LOG(INFO) << "[InertialInitializer] " << "first imu time : " << imu_datas_.front().timestamp * 1e-9;
  LOG(INFO) << "[InertialInitializer] " << "imu_datas_ : " << imu_datas_.size();
  LOG(INFO) << "[InertialInitializer] " << "imu_cache : " << imu_datas_.size();
  LOG(INFO) << "[InertialInitializer] " << "Accel bias : " << imu_state_.bias.accel_bias.transpose();
  LOG(INFO) << "[InertialInitializer] " << "Gyro bias : " << imu_state_.bias.gyro_bias.transpose();
  LOG(INFO) << "[InertialInitializer] " << "gravity in I0 : " << g_inI0.transpose();
  // std::cout << MAGENTA << "[Rwb0]\n" << R_I0toG << RESET << std::endl;

  initial_done_ = true;
  return true;
}

}  // namespace cocolic
