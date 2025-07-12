#pragma once

#include <glog/logging.h>
#include <Eigen/Eigen>

#include "../spline/trajectory.h"
#include "eigen_utils.hpp"
#include "tic_toc.h"

#include <libgen.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace cocolic {

inline std::string VectorToString(const Eigen::Vector3d& data,
                                  int precision = 3) {
  std::stringstream ss;
  ss.precision(precision);
  ss.setf(std::ios::fixed);
  ss << data[0] << ", " << data[1] << ", " << data[2];

  // std::string param_info = ss.str();
  return ss.str();
}

struct TimeStatistics {
  std::vector<double> time_cost;
  std::vector<std::string> time_descri;

  TicToc timer;
  TicToc timer_all;

  TimeStatistics() {
    timer.tic();
    timer_all.tic();

    time_cost.reserve(10);
    time_descri.reserve(10);
  }

  void Tic() { timer.tic(); }

  void ReStart() {
    timer.tic();
    timer_all.tic();

    time_cost.clear();
    time_descri.clear();
  }

  void TocAndTic(std::string descri) {
    double t = timer.toc();
    timer.tic();

    time_cost.push_back(t);
    time_descri.push_back(descri);
  }

  void LogTimeStatistics(double time_now) {
    double t_all = timer_all.toc();

    time_cost.push_back(t_all);
    time_descri.push_back("all");

    auto&& log = COMPACT_GOOGLE_LOG_INFO;
    log.stream() << "TimeSummary:" << time_now << ";";
    for (size_t i = 0; i < time_cost.size(); i++) {
      log.stream() << time_descri.at(i) << ":" << time_cost.at(i) << ";";
    }
    log.stream() << "\n";
  }
};

}  // namespace cocolic