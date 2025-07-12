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

// #include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <utils/yaml_utils.h>
#include <Eigen/Core>
#include <cmath>

namespace cocolic {
struct IMUNoise {
  IMUNoise() {}

  IMUNoise(const YAML::Node& node) {
    sigma_w = yaml::GetValue<double>(node, "gyroscope_noise_density");
    sigma_wb = yaml::GetValue<double>(node, "gyroscope_random_walk");
    sigma_a = yaml::GetValue<double>(node, "accelerometer_noise_density");
    sigma_ab = yaml::GetValue<double>(node, "accelerometer_random_walk");

    imu_rate_gyro = yaml::GetValue<double>(node, "imu_info_vec_rate_gyro", 1.0);
    imu_rate_accel =
        yaml::GetValue<double>(node, "imu_info_vec_rate_accel", 1.0);

    sigma_w_2 = std::pow(sigma_w, 2);
    sigma_wb_2 = std::pow(sigma_wb, 2);
    sigma_a_2 = std::pow(sigma_a, 2);
    sigma_ab_2 = std::pow(sigma_ab, 2);

    rot_weight = yaml::GetValue<double>(node, "rot_weight");
    pos_weight = yaml::GetValue<double>(node, "pos_weight");

    print();
  }

  void print() {
    static bool first = true;
    if (!first) return;
    first = false;
  }

  // double imu_frequency = 200.;

  double imu_rate_gyro = 1.0;
  double imu_rate_accel = 1.0;

  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope white noise covariance
  double sigma_w_2 = std::pow(1.6968e-04, 2);

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Gyroscope random walk covariance
  double sigma_wb_2 = std::pow(1.9393e-05, 2);

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-3;

  /// Accelerometer white noise covariance
  double sigma_a_2 = std::pow(2.0000e-3, 2);

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;

  /// Accelerometer random walk covariance
  double sigma_ab_2 = std::pow(3.0000e-03, 2);

  double sigma_wb_discrete = 0;
  double sigma_ab_discrete = 0;

  double rot_weight = 100.0;
  double pos_weight = 100.0;
};

struct OptWeight {
  IMUNoise imu_noise;

  Eigen::Matrix<double, 6, 1> imu_info_vec;

  // [for nurbs vertification]
  double rot_weight = 100.0;
  double pos_weight = 100.0;

  double global_velocity;

  double local_velocity;
  Eigen::Vector3d local_velocity_info_vec;

  double relative_rotation;
  Eigen::Vector3d relative_rotation_info_vec;

  double lidar_weight;
  double image_weight;

  OptWeight() {}

  OptWeight(const YAML::Node& node) { LoadWeight(node); }

  void LoadWeight(const YAML::Node& node) {
    imu_noise = IMUNoise(node);

    Eigen::Vector3d one3d = Eigen::Vector3d::Ones();

    /////////////////////////////////////////////////
    imu_info_vec.block<3, 1>(0, 0) =
        1.0 / imu_noise.sigma_w * one3d;
    imu_info_vec.block<3, 1>(3, 0) =
        1.0 / imu_noise.sigma_a * one3d;
    imu_noise.sigma_wb_discrete = imu_noise.sigma_wb;
    imu_noise.sigma_ab_discrete = imu_noise.sigma_ab;
    /////////////////////////////////////////////////
    

    rot_weight = imu_noise.rot_weight;
    pos_weight = imu_noise.pos_weight;

    global_velocity = yaml::GetValue<double>(node, "global_velocity");

    local_velocity = yaml::GetValue<double>(node, "local_velocity");
    local_velocity_info_vec = local_velocity * one3d;

    relative_rotation = yaml::GetValue<double>(node, "relative_rotation");
    relative_rotation_info_vec = relative_rotation * one3d;

    lidar_weight = yaml::GetValue<double>(node, "lidar_weight");

    image_weight = yaml::GetValue<double>(node, "image_weight");

    print();
  }

  inline double w2(double w) { return w * w; }

  void print() {}
};

}  // namespace cocolic
