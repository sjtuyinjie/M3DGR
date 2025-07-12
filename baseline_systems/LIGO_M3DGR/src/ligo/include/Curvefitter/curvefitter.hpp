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

#ifndef TRAJECTORY_MANAGER_HPP
#define TRAJECTORY_MANAGER_HPP

// #include <odometry/imu_state_estimator.h>
// #include <sensor_data/imu_data.h>
// #include <sensor_data/loop_closure_data.h>
// #include <utils/tic_toc.h>
// #include <odometry/lidar_odometry.hpp>
// #include <sensor_data/calibration.hpp>
#include <Curvefitter/se3_trajectory.hpp>
#include <Curvefitter/trajectory_estimator.hpp>
// #include <Curvefitter/trajectory_viewer.hpp>

#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace curvefitter {

template <int _N>
class TrajectoryManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using TrajectoryN = Trajectory<_N>;

  typedef std::shared_ptr<TrajectoryManager<_N>> Ptr;

  TrajectoryManager()
      : trajectory_(new Trajectory<_N>(0.1)) {}

  ~TrajectoryManager() {}

  void SetTrajectory(std::shared_ptr<TrajectoryN> trajectory) {
    trajectory_ = trajectory; //.reset(trajectory.get());
    // pose_graph_key_pose.swap(std::vector<PoseData>());
    // LiDAR_points.swap(std::vector<Eigen::Vector3d>());
    // pose_time_vector.swap(std::vector<double>());
  }

  void ResetTrajectory(std::vector<PoseData> &pose_graph_key_pose
    , std::vector<double> &pose_time_vector, std::vector<std::vector<Eigen::Vector3d> > &LiDAR_points, int &feats_num) {
    // trajectory_ = trajectory;
    std::vector<curvefitter::PoseData>().swap(pose_graph_key_pose);
    std::vector<std::vector<Eigen::Matrix<double, 3, 1> > >().swap(LiDAR_points);
    std::vector<double>().swap(pose_time_vector);
    pose_graph_key_pose.reserve(1000);
    LiDAR_points.reserve(1000);
    pose_time_vector.reserve(1000);
    feats_num = 0;
  }

  std::shared_ptr<TrajectoryN> get_trajectory() { return trajectory_; }

  void IntegrateIMUMeasurement(double scan_min, double scan_max, std::vector<PoseData> &pose_graph_key_pose);

  void SetInitialPoseRotation(Eigen::Quaterniond q) {
    init_pose.orientation.setQuaternion(q);
  }

  void SetInitialPosePosition(Eigen::Vector3d p) {
    init_pose.position = p;
  }

  void FitCurve(Eigen::Quaterniond q, Eigen::Vector3d p, double scan_time_min, double scan_time_max, std::vector<PoseData> &pose_graph_key_pose) {
    SetInitialPosePosition(p);
    SetInitialPoseRotation(q);
    IntegrateIMUMeasurement(scan_time_min, scan_time_max, pose_graph_key_pose);
    return;
  }

  void AddGraphPose(Eigen::Quaterniond q, Eigen::Vector3d p, std::vector<Eigen::Vector3d> lidar_points, double pose_time, std::vector<PoseData> &pose_graph_key_pose
    , std::vector<double> &pose_time_vector, std::vector<std::vector<Eigen::Vector3d> > &LiDAR_points, int &feats_num) {
    PoseData pose_data;
    pose_data.orientation.setQuaternion(q);
    pose_data.position = p;
    pose_data.timestamp = pose_time;
    pose_time_vector.push_back(pose_time);
    pose_graph_key_pose.push_back(pose_data);
    LiDAR_points.push_back(lidar_points);
    feats_num += lidar_points.size();
    return;
  }

  std::vector<Eigen::Vector3d> GetUpdatedMapPoints(std::vector<double> &pose_time_vector, std::vector<std::vector<Eigen::Vector3d> > &LiDAR_points) {
    std::vector<Eigen::Vector3d> MapPointsUpdated;
    for (int i = 0; i < pose_time_vector.size() - 1; i++) {
      double pose_time = pose_time_vector[i];
      // if (pose_time > trajectory_->maxTime() - _N * trajectory_->dt_) {
      auto pose = trajectory_->GetLidarPose(pose_time); // useful
      // poseUpdated.push_back(pose);
      for (int j = 0; j < LiDAR_points[i].size(); j++) {
          Eigen::Vector3d mappoint = pose * LiDAR_points[i][j];
          MapPointsUpdated.push_back(mappoint);
      }
      // }
    }
    return MapPointsUpdated;
  }

  // std::vector<PoseData> pose_graph_key_pose;

  // std::vector<double> pose_time_vector; // = std::vector<double>();
  
  double rot_weight = 1, pos_weight = 1;
  std::shared_ptr<TrajectoryN> trajectory_ = nullptr;
 
 private:

  // std::vector<std::vector<Eigen::Vector3d> > LiDAR_points;

  bool trajectory_init_ = false; //

  PoseData init_pose; // give value

};

template <int _N>
void TrajectoryManager<_N>::IntegrateIMUMeasurement(double scan_time_min, // right here
                                                    double scan_time_max, std::vector<PoseData> &pose_graph_key_pose) {
  // trajectory_->extendKnotsTo(scan_time_min, init_pose.orientation, init_pose.position); // set the initial knot as the last knot
  Sophus::SE3d last_kont(init_pose.orientation, init_pose.position);
  //  = init_pose; // trajectory_->getLastKnot(); // get the last knot of the tra0jectory 
  std::shared_ptr<TrajectoryEstimator<_N>> estimator =
      std::make_shared<TrajectoryEstimator<_N>>(trajectory_); // initialize an estimator
  trajectory_->setStartTime(scan_time_min); // set the start time of the trajectory
  // trajectory_->GetActiveTime();
  trajectory_->extendKnotsTo(scan_time_max, last_kont); // set the initial knot as the last knot
  estimator->LockTrajectory(false); // unlock trajectory
  for (const auto& state : pose_graph_key_pose) { // get state values in the integrate_imu_state_
    estimator->AddLiDARPoseMeasurement(state, rot_weight, pos_weight); // imitate this function
  }
  ceres::Solver::Summary summary = estimator->Solve(50, false); // after solve, how to update the trajectory_?
}

}  // namespace clins

#endif

