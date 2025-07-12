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
#include <ceres/internal/port.h>
#include <ceres/iteration_callback.h>
#include <glog/logging.h>

namespace cocolic {

class CheckStateCallback : public ceres::IterationCallback {
 public:
  CheckStateCallback() : iteration_(0u) {}

  ~CheckStateCallback() {}

  void addCheckState(const std::string& description, size_t block_size,
                     double* param_block) {
    parameter_block_descr.push_back(description);
    parameter_block_sizes.push_back(block_size);
    parameter_blocks.push_back(param_block);
  }

#if true
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    if (iteration_ == 0) {
      auto&& log = COMPACT_GOOGLE_LOG_INFO;
      log.stream() << "Iteration ";
      for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
        log.stream() << parameter_block_descr.at(i) << " ";
      }
      log.stream() << "\n";
    }
    
    auto&& log = COMPACT_GOOGLE_LOG_INFO;
    log.stream() << iteration_ << " ";
    for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
      for (size_t k = 0; k < parameter_block_sizes.at(i); ++k)
        log.stream() << parameter_blocks.at(i)[k] << " ";
    }

    log.stream() << "\n";

    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }
#else
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    auto&& log = COMPACT_GOOGLE_LOG_INFO;

    if (iteration_ == 0) {
      std::cout << "Iteration ";
      for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
        std::cout << parameter_block_descr.at(i) << " ";
      }
      std::cout << "\n";
    }

    std::cout << iteration_ << " ";
    for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
      for (size_t k = 0; k < parameter_block_sizes.at(i); ++k)
        std::cout << parameter_blocks.at(i)[k] << " ";
    }

    std::cout << "\n";

    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }
#endif

 private:
  std::vector<std::string> parameter_block_descr;
  std::vector<size_t> parameter_block_sizes;
  std::vector<double*> parameter_blocks;

  // Count iterations locally
  size_t iteration_;
};

}  // namespace cocolic
