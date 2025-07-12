/*
 * CLINS: Continuous-Time Trajectory Estimation for LiDAR-Inertial System
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Kewei Hu
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

#ifndef _CERES_CALLBACKS_
#define _CERES_CALLBACKS_

#include <fstream>
#include <iostream>
#include <sstream>

#include <ceres/ceres.h>
#include <ceres/internal/port.h>
#include <ceres/iteration_callback.h>

namespace clins {

class CheckStateCallback : public ceres::IterationCallback {
 public:
  CheckStateCallback(std::string filename = "")
      : iteration_(0u), filename_(filename) {}

  ~CheckStateCallback() {}

  void addCheckState(const std::string& description, size_t block_size,
                     double* param_block) {
    parameter_block_descr.push_back(description);
    parameter_block_sizes.push_back(block_size);
    parameter_blocks.push_back(param_block);
  }

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    if (iteration_ == 0 && filename_ != "") {
      std::stringstream ss;
      ss << "Iteration";
      for (auto const& str : parameter_block_descr) ss << " " << str;
      ss << std::endl;

      std::ofstream outfile;
      outfile.open(filename_);
      outfile << ss.str();
      outfile.close();
      ss.clear();
    }
    std::stringstream ss;
    ss << iteration_;
    for (size_t i = 0; i < parameter_block_descr.size(); ++i) {
      if (filename_ == "") ss << " " << parameter_block_descr.at(i);
      for (size_t k = 0; k < parameter_block_sizes.at(i); ++k)
        ss << " " << parameter_blocks.at(i)[k];
    }
    ss << std::endl;

    if (filename_ == "") {
      std::cout << ss.str();
    } else {
      std::ofstream outfile;
      outfile.open(filename_, std::ios::app);
      outfile << ss.str();
      outfile.close();
    }

    ++iteration_;
    return ceres::SOLVER_CONTINUE;
  }

 private:
  std::vector<std::string> parameter_block_descr;
  std::vector<size_t> parameter_block_sizes;
  std::vector<double*> parameter_blocks;

  // Count iterations locally
  size_t iteration_;
  std::string filename_;
};

}  // namespace clins

#endif
