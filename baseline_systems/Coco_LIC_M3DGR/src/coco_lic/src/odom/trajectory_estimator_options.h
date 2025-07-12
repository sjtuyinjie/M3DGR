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

#include <spline/trajectory.h>

namespace cocolic
{

  struct LockExtrinsic
  {
    bool lock_P = true;
    bool lock_R = true;
    bool lock_t_offset = true;
  };

  struct TrajectoryEstimatorOptions
  {
    TrajectoryEstimatorOptions()
    {
      LockExtrinsic l_extrinsic;
      lock_EPs[IMUSensor] = l_extrinsic;
      lock_EPs[LiDARSensor] = l_extrinsic;
      lock_EPs[CameraSensor] = l_extrinsic;
    }
    bool spline_back_up = true;

    // lock the extrinsic position/rotation/t_offset between imu and sensor
    std::map<SensorType, LockExtrinsic> lock_EPs;

    bool use_auto_diff = false;

    // If estimating the time offset, the max/min value of time offset
    int64_t t_offset_padding_ns = 1e7; // 0.02;

    // If we should optimize the trajectory
    bool lock_traj = false;
    bool lock_tran = false;

    // lock the imu bias/gravity
    bool lock_ab = true;
    bool lock_wb = true;
    bool lock_g = true;

    // ======= Marginalization ======= //
    int division;

    bool is_marg_state = false;

    int ctrl_to_be_opt_now = 0;
    int ctrl_to_be_opt_later = 0;

    bool marg_bias_param = true;

    bool marg_gravity_param = true;

    bool marg_t_offset_param = true;

    // If we use the analytic factor
    // bool use_analytic_factor = true;

    // for debug
    bool show_residual_summary = false;
  };

} // namespace cocolic
