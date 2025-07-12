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

#include <ceres/jet.h>
#include <Eigen/Dense>
#include <cstdint>
#include <iomanip>

namespace cocolic {

// Define time types
using time_span_t = std::pair<int64_t, int64_t>;
using time_init_t = std::initializer_list<time_span_t>;

static constexpr double NS_TO_S = 1e-9;  ///< Nanosecond to second conversion
static constexpr double S_TO_NS = 1e9;   ///< Second to nanosecond conversion

struct MetaData {
  virtual size_t NumParameters() const = 0;
};

template <int _N>
struct SplineSegmentMeta : public MetaData {
  static constexpr int N = _N;        // Order of the spline.
  static constexpr int DEG = _N - 1;  // Degree of the spline.

  int64_t t0_ns;  // First valid time
  int64_t dt_ns;  // Knot spacing
  size_t n;       // Number of knots

  double pow_inv_dt[N];

  SplineSegmentMeta(int64_t _t0_ns, int64_t _dt_ns, size_t _n = 0)
      : t0_ns(_t0_ns), dt_ns(_dt_ns), n(_n) {
    pow_inv_dt[0] = 1.0;
    pow_inv_dt[1] = S_TO_NS / dt_ns;

    for (size_t i = 2; i < N; i++) {
      pow_inv_dt[i] = pow_inv_dt[i - 1] * pow_inv_dt[1];
    }
  }

  size_t NumParameters() const override { return n; }

  int64_t MinTimeNs() const { return t0_ns; }

  int64_t MaxTimeNs() const { return t0_ns + (n - DEG) * dt_ns; }

  std::pair<double, size_t> computeTIndexNs(int64_t time_ns) const {
    if (time_ns < MinTimeNs() || time_ns >= MaxTimeNs()) {
      std::cout << time_ns << " not in " << MinTimeNs() << ", " << MaxTimeNs()
                << "\n";
    }

    assert(time_ns >= MinTimeNs() && time_ns < MaxTimeNs() &&
           "computeTIndexNs from SplineSegmentMeta");

    // double st = timestamp - t0;
    // size_t s = std::floor(st / dt);
    // double u = (st - s * dt) / dt;
    int64_t st_ns = (time_ns - t0_ns);
    int64_t s = st_ns / dt_ns;
    double u = double(st_ns % dt_ns) / double(dt_ns);

    return std::make_pair(u, s);
  }

  template <typename T>
  size_t PotentiallyUnsafeFloor(T x) const {
    return static_cast<size_t>(std::floor(x));
  }

  // This way of treating Jets are potentially unsafe, hence the function name
  template <typename Scalar, int N>
  size_t PotentiallyUnsafeFloor(const ceres::Jet<Scalar, N>& x) const {
    return static_cast<size_t>(ceres::floor(x.a));
  };

  bool computeTIndexNs(const int64_t& time_ns, size_t& s, double& u) const {
    if (time_ns < MinTimeNs() || time_ns >= MaxTimeNs()) {
      return false;
    }

    int64_t st_ns = (time_ns - t0_ns);
    s = st_ns / dt_ns;                          
    u = double(st_ns % dt_ns) / double(dt_ns);  
    return true;
  }

  template <typename T>
  bool computeTIndexNs(const T& time_ns, size_t& s, T& u) const {
    if (time_ns < T(MinTimeNs()) || time_ns >= T(MaxTimeNs())) {
      return false;
    }

    T st = (time_ns - T(t0_ns)) / T(dt_ns);
    s = PotentiallyUnsafeFloor(st);  
    u = st - T(s);                  
    return true;
  }
};

template <int _N>
struct SplineMeta {
  std::vector<SplineSegmentMeta<_N>> segments;

  size_t NumParameters() const {
    size_t n = 0;
    for (auto& segment_meta : segments) {
      n += segment_meta.NumParameters();
    }
    return n;
  }

  bool ComputeSplineIndex(const int64_t& time_ns, size_t& idx,
                          double& u) const {
    idx = 0;
    for (auto const& seg : segments) {
      size_t s = 0;
      if (seg.computeTIndexNs(time_ns, s, u)) {
        idx += s;
        return true;
      } else {
        idx += seg.NumParameters();
      }
    }
    std::cout << std::fixed << std::setprecision(15)
              << "ComputeSplineIndex1 Problem :" << time_ns << " not in ["
              << segments[0].t0_ns << ", " << segments[0].MaxTimeNs() << "]\n";
    return false;
  }

  template <typename T>
  bool ComputeSplineIndex(const T& time_ns, size_t& idx, T& u) const {
    idx = 0;
    for (auto const& seg : segments) {
      size_t s = 0;
      if (seg.computeTIndexNs(time_ns, s, u)) {
        idx += s;
        return true;
      } else {
        idx += seg.NumParameters();
      }
    }
    std::cout << std::fixed << std::setprecision(15)
              << "ComputeSplineIndex2 Problem :" << time_ns << " not in ["
              << segments[0].t0_ns << ", " << segments[0].MaxTimeNs() << "]\n";
    return false;
  }
};

}  // namespace cocolic
