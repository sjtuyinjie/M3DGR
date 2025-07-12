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

#include <glog/logging.h>
#include "../utils/mypcl_cloud_type.h"
#include "../utils/parameter_struct.h"
#include "se3_spline.h"
#include <unordered_map>

namespace cocolic {

enum SensorType {
  IMUSensor = 0,  //  qurey pose and EP
  LiDARSensor,
  CameraSensor,
  Marker
};

class TrajectoryManager;

class Trajectory : public Se3Spline<SplineOrder, double> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<Trajectory> Ptr;

  static constexpr double NS_TO_S = 1e-9;  ///< Nanosecond to second conversion
  static constexpr double S_TO_NS = 1e9;   ///< Second to nanosecond conversion

  std::unordered_map<int, int> intensity_map;  //knot_idx, intensity

  Trajectory(double time_interval, double start_time = 0)
      : Se3Spline<SplineOrder, double>(time_interval * S_TO_NS,
                                       start_time * S_TO_NS),
        data_start_time_(-1),
        active_time_ns(-1),
        max_time_ns(-1),
        forced_fixed_time_(-1) {
    this->extendKnotsTo(SO3d(Eigen::Quaterniond::Identity()),
                        Eigen::Vector3d(0, 0, 0));
    
    intensity_map[0] = 400;
    intensity_map[1] = 400;
    intensity_map[2] = 400;
    intensity_map[3] = 400;

    ExtrinsicParam EP_StoI;
    EP_StoI_[IMUSensor] = EP_StoI;
  }

  void AddKnt(double time) {
    knts.push_back(time * S_TO_NS);
  }

  void AddKntNs(int64_t time) {
    knts.push_back(time);
  }

  void SetSensorExtrinsics(const SensorType type,
                           const ExtrinsicParam& EP_StoI) {
    EP_StoI_[type] = EP_StoI;
  }

  const ExtrinsicParam& GetSensorEP(const SensorType type) const {
    return EP_StoI_.at(type);
  }

  std::map<SensorType, ExtrinsicParam>& GetSensorEPs() { return EP_StoI_; };

  void UpdateExtrinsics() {
    for (auto& sensor_EP : EP_StoI_) {
      auto& EP = sensor_EP.second;
      EP.se3.so3() = EP.so3;
      EP.se3.translation() = EP.p;
      EP.q = EP.so3.unit_quaternion();
    }
  }

  void UpdateTimeOffset(
      const std::map<SensorType, double*>& t_offset_ns_params) {
    for (auto const& sensor_t : t_offset_ns_params) {
      if (EP_StoI_.find(sensor_t.first) != EP_StoI_.end()) {
        EP_StoI_.at(sensor_t.first).t_offset_ns = *(sensor_t.second);
      }
    }
  }

  int64_t GetDataStartTime() const { return data_start_time_; }

  int64_t GetActiveTime() const { return active_time_ns; }

  double GetForcedFixedTime() const { return forced_fixed_time_; }

  // ===================== Get pose/vel/accel ===================== //

  // double minTime() const { return this->minTimeNs() * NS_TO_S; }

  double maxTime() const { return this->maxTimeNs() * NS_TO_S; }

  double minTime(const SensorType type) const {
    double t_offset_ns = EP_StoI_.at(type).t_offset_ns;
    return NS_TO_S * this->minTimeNs() - NS_TO_S * t_offset_ns;
  }

  double maxTime(const SensorType type) const {
    double t_offset_ns = EP_StoI_.at(type).t_offset_ns;
    return NS_TO_S * this->maxTimeNs() - NS_TO_S * t_offset_ns;
  }

  /// [nurbs]
  // double maxTimeNURBS() const {
  //   return NS_TO_S * this->maxTimeNsNURBS();
  // }

  double maxTimeNURBS() const {
    return NS_TO_S * this->maxTimeNsNURBS();
  }

  void GetIMUState(double time, IMUState& imu_state) const;

  Eigen::Vector3d GetPositionWorld(const double timestamp) const {
    auto& Ep = EP_StoI_.at(IMUSensor);
    return this->positionWorld(timestamp * S_TO_NS + Ep.t_offset_ns);
  }

  Eigen::Vector3d GetTransVelWorld(const double timestamp) const {
    auto& Ep = EP_StoI_.at(IMUSensor);
    return this->transVelWorld(timestamp * S_TO_NS + Ep.t_offset_ns);
  }

  Eigen::Vector3d GetTransAccelWorld(const double timestamp) const {
    auto& Ep = EP_StoI_.at(IMUSensor);
    return this->transAccelWorld(timestamp * S_TO_NS + Ep.t_offset_ns);
  }

  Eigen::Vector3d GetRotVelBody(const double timestamp) const {
    auto& Ep = EP_StoI_.at(IMUSensor);
    return this->rotVelBody(timestamp * S_TO_NS + Ep.t_offset_ns);
  }

  SE3d GetIMUPose(const double timestamp) const {
    auto& Ep = EP_StoI_.at(IMUSensor);
    return this->poseNs(timestamp * S_TO_NS + Ep.t_offset_ns);
  }

  SE3d GetLidarPose(const double timestamp) const {
    return GetSensorPose(timestamp, EP_StoI_.at(LiDARSensor));
  }

  /// [nurbs]
  SE3d GetIMUPoseNURBS(const double timestamp) const {
    return this->poseNsNURBS(timestamp * S_TO_NS);
  }

  SE3d GetIMUPoseNsNURBS(const int64_t timestamp) const {
    return this->poseNsNURBS(timestamp);
  }

  SE3d GetLidarPoseNURBS(const int64_t timestamp) const {
    return GetSensorPoseNURBS(timestamp, EP_StoI_.at(LiDARSensor));
  }

  SE3d GetLidarPoseNURBS(const int64_t timestamp, int start_idx) const {
    return GetSensorPoseNURBS(timestamp, EP_StoI_.at(LiDARSensor), start_idx);
  }

  SE3d GetCameraPoseNURBS(const int64_t timestamp) const {
    return GetSensorPoseNURBS(timestamp, EP_StoI_.at(CameraSensor));
  }

  Eigen::Vector3d GetTransVelWorldNURBS(const std::pair<int, double> su,
    double delta_t, const Eigen::Matrix4d blend_mat) const {
    return this->transVelWorldNURBS(su, delta_t, blend_mat);
  }

  Eigen::Vector3d GetPositionWorldNURBS(const double timestamp) const {
    return this->positionWorldNURBS(timestamp * S_TO_NS);
  }

  Eigen::Vector3d GetPositionWorldNsNURBS(const int64_t timestamp) const {
    return this->positionWorldNURBS(timestamp);
  }

  Eigen::Vector3d GetRotVelBodyNURBS(const double timestamp) const {
    return this->rotVelBodyNURBS(timestamp * S_TO_NS);
  }

  Eigen::Vector3d GetRotVelBodyNsNURBS(const int64_t timestamp) const {
    return this->rotVelBodyNURBS(timestamp);
  }

  SE3d GetCameraPose(const double timestamp) const {
    return GetSensorPose(timestamp, EP_StoI_.at(CameraSensor));
  }

  // ================================================================ //
  void UndistortScan(const PosCloud& scan_raw, const int64_t target_timestamp,
                     PosCloud& scan_in_target) const;

  void UndistortScanInG(const PosCloud& scan_raw, const int64_t scan_raw_timestamp,
                        PosCloud& scan_in_target) const;

  void SetForcedFixedTime(double time) {
    // if (time < minTime(LiDARSensor))
    //   forced_fixed_time_ = minTime(LiDARSensor);
    if (time < 0.0)
      forced_fixed_time_ = 0.0;
    else
      forced_fixed_time_ = time;
  }

  void SetActiveTime(int64_t time) { active_time_ns = time; }

  void ToTUMTxt(std::string traj_path, int64_t maxtime, bool is_evo_viral, double dt = 0.1);

  void SetDataStartTime(int64_t time) { data_start_time_ = time; }

  double opt_min_init_time_tmp = 0;
  int opt_init_fixed_idx_tmp = -1;

  double opt_min_lio_time_tmp = 0;

  int64_t opt_min_loop_time_tmp = 0;
  int opt_loop_fixed_idx_tmp = -1;

 protected:
  // void UpdateActiveTime(double measurement_time) {
  //   int64_t time_ns = measurement_time * S_TO_NS;
  //   size_t knot_index = this->GetCtrlIndexNURBS(time_ns);
  //   // int64_t time_active_ns =
  //   //     ((int)knot_index - SplineOrder + 1) * this->getDtNs() -
  //   //     this->minTimeNs();
  //   int64_t time_active_ns = knts[knot_index - SplineOrder + 1];
  //   active_time_ = std::max(time_active_ns * NS_TO_S, active_time_);
  //   LOG(INFO) << "[ActiveTimes]  :  " << active_time_;
  // }

  SE3d GetSensorPose(const double timestamp,
                     const ExtrinsicParam& EP_StoI) const;
  
  SE3d GetSensorPoseNURBS(const int64_t timestamp,
                     const ExtrinsicParam& EP_StoI) const;

  SE3d GetSensorPoseNURBS(const int64_t timestamp,
                     const ExtrinsicParam& EP_StoI, const int start_idx) const;

 private:
  int64_t data_start_time_;    
  // double active_time_;       
  double forced_fixed_time_; 

  int64_t max_time_ns;
  int64_t active_time_ns;

  std::map<SensorType, ExtrinsicParam> EP_StoI_;

  friend TrajectoryManager;  
};

}  // namespace cocolic
