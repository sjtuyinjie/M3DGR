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

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/sort/spreadsort/spreadsort.hpp>
#include <iostream>

#include <utils/mypcl_cloud_type.h>

namespace pcl {
template <typename TPoint>
inline float PointNorm(const TPoint& p) {
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

template <typename TPoint>
inline float PointDistance(const TPoint& p1, const TPoint& p2) {
  return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                   (p1.y - p2.y) * (p1.y - p2.y) +
                   (p1.z - p2.z) * (p1.z - p2.z));
}

inline float GetCloudMaxTime(const RTPointCloud::Ptr cloud) {
  assert(cloud->size() > 0 && "[GetCloudMaxTime] input empty cloud.");
  double t_max = cloud->back().time;
  for (size_t i = cloud->size() - 1; i > 0; i--) {
    double t_point = cloud->points[i].time;
    if (t_max < t_point) t_max = t_point;
  }
  return t_max;
}

inline int64_t GetCloudMaxTimeNs(const RTPointCloud::Ptr cloud) {
  assert(cloud->size() > 0 && "[GetCloudMaxTime] input empty cloud.");
  int64_t t_max = cloud->back().time;
  for (size_t i = cloud->size() - 1; i > 0; i--) {
    int64_t t_point = cloud->points[i].time;
    if (t_max < t_point) t_max = t_point;
  }
  return t_max;
}

inline void FilterCloudByTimestamp(const RTPointCloud::Ptr& cloud_in,
                                   int64_t timestamp,
                                   RTPointCloud::Ptr& cloud_bef,
                                   RTPointCloud::Ptr& cloud_aft) {
  for (const auto& point : cloud_in->points) {
    if (point.time < timestamp)
      cloud_bef->push_back(point);
    else
      cloud_aft->push_back(point);
  }
}

inline void OusterPointCloud2RTPointCloud(
    const OusterPointCloud ::Ptr& input_cloud,
    RTPointCloud::Ptr& output_cloud) {
  output_cloud->header = input_cloud->header;
  output_cloud->height = input_cloud->height;
  output_cloud->width = input_cloud->width;
  output_cloud->resize(input_cloud->height * input_cloud->width);
  output_cloud->is_dense = input_cloud->is_dense;

  RTPoint zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.intensity = 0;
  zero_point.time = 0;
  for (size_t i = 0; i < input_cloud->size(); i++) {
    auto& src = input_cloud->points[i];
    auto& dst = output_cloud->points[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    dst.ring = src.ring;
    dst.time = src.t;

    if (PointNorm(dst) < 0.1 || dst.time > 0.3 * 1e9) {
      dst = zero_point;
      dst.ring = src.ring;
    }
  }
}

inline void RTPointCloudTmp2RTPointCloud(
    const RTPointCloudTmp ::Ptr& input_cloud,
    RTPointCloud::Ptr& output_cloud) {
  output_cloud->header = input_cloud->header;
  output_cloud->height = input_cloud->height;
  output_cloud->width = input_cloud->width;
  output_cloud->resize(input_cloud->height * input_cloud->width);
  output_cloud->is_dense = input_cloud->is_dense;

  RTPoint zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.intensity = 0;
  zero_point.time = -1;
  for (size_t i = 0; i < input_cloud->size(); i++) {
    auto& src = input_cloud->points[i];
    auto& dst = output_cloud->points[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    dst.ring = src.ring;
    dst.time = int64_t(src.time * 1e9);  // float src.time = 0.0995 

    if (PointNorm(dst) < 0.1 || dst.time > 0.11 * 1e9) {
      dst = zero_point;
      dst.ring = src.ring;
    }
  }
}

inline void RTPointCloudTmp2RTPointCloudHesai(
    const RTPointCloudTmpHesai ::Ptr& input_cloud,
    RTPointCloud::Ptr& output_cloud) {
  output_cloud->header = input_cloud->header;
  output_cloud->height = input_cloud->height;
  output_cloud->width = input_cloud->width;
  output_cloud->resize(input_cloud->height * input_cloud->width);
  output_cloud->is_dense = input_cloud->is_dense;

  RTPoint zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.intensity = 0;
  zero_point.time = -1;

  double first_timestamp = input_cloud->points[0].timestamp;
  for (size_t i = 0; i < input_cloud->size(); i++) {
    auto& src = input_cloud->points[i];
    auto& dst = output_cloud->points[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    dst.ring = src.ring;
    dst.time = int64_t((src.timestamp - first_timestamp) * 1e9);  // float src.time = 0.0995 

    if (PointNorm(dst) < 0.1 || dst.time > 0.11 * 1e9) {
      dst = zero_point;
      dst.ring = src.ring;
    }
  }
}

inline void OusterPointCloudTmp2RTPointCloud(
    const OusterPointCloudTmp ::Ptr& input_cloud,
    RTPointCloud::Ptr& output_cloud) {
  output_cloud->header = input_cloud->header;
  output_cloud->height = input_cloud->height;
  output_cloud->width = input_cloud->width;
  output_cloud->resize(input_cloud->height * input_cloud->width);
  output_cloud->is_dense = input_cloud->is_dense;

  RTPoint zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.intensity = 0;
  zero_point.time = -1;
  for (size_t i = 0; i < input_cloud->size(); i++) {
    auto& src = input_cloud->points[i];
    auto& dst = output_cloud->points[i];
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.intensity = src.intensity;
    dst.ring = src.ring;
    dst.time = int64_t(src.t);  // uint32_t src.t = 99870487 ns

    if (PointNorm(dst) < 0.1 || dst.time > 0.11 * 1e9) {
      dst = zero_point;
      dst.ring = src.ring;
    }
  }
}

inline void CloudToRelativeMeasureTime(RTPointCloud::Ptr& cloud,
                                       int64_t scan_timestamp,
                                       int64_t traj_start_time,
                                       int64_t duration = 0.11 * 1e9) {
  RTPoint zero_point;
  zero_point.x = 0;
  zero_point.y = 0;
  zero_point.intensity = 0;
  zero_point.time = -1;

  int64_t scan_to_start = scan_timestamp - traj_start_time;
  for (size_t i = 0; i < cloud->size(); i++) {
    auto& dst = cloud->points[i];
    int64_t t_wrt_traj_start = scan_to_start + dst.time;

    // assert(dst.time >= 0 && "lidar point time wrong!");
    if (dst.time < 0) continue;

    if (dst.time > duration) {
      dst = zero_point;
      dst.ring = dst.ring;
    } else {
      dst.time = t_wrt_traj_start;
    }
  }
}

inline void RTPointCloudToPosCloud(const RTPointCloud::Ptr input_cloud,
                                   PosCloud::Ptr output_cloud,
                                   int64_t* max_time = NULL) {
  assert(input_cloud->size() > 0 &&
         "[RTPointCloudToPosCloud] input empty cloud.");
  output_cloud->header = input_cloud->header;
  output_cloud->resize(input_cloud->size());
  if (max_time) *max_time = 0;

  size_t cnt = 0;
  for (auto const& v : input_cloud->points) {
    if (v.time < 0) continue;
    PosPoint p;
    p.x = v.x;
    p.y = v.y;
    p.z = v.z;
    p.intensity = v.intensity;

    p.timestamp = v.time;
    output_cloud->points[cnt++] = p;

    if (max_time && (*max_time < p.timestamp)) *max_time = p.timestamp;
  }

  // Resize to the correct size
  if (cnt != input_cloud->size()) output_cloud->resize(cnt);
}

// Remove timestamp from input_cloud
inline void PosCloudToVPointCloud(const PosCloud& input_cloud,
                                  VPointCloud& output_cloud) {
  output_cloud.resize(input_cloud.size());
  for (size_t i = 0; i < input_cloud.size(); i++) {
    VPoint p;
    p.x = input_cloud.points[i].x;
    p.y = input_cloud.points[i].y;
    p.z = input_cloud.points[i].z;
    p.intensity = input_cloud.points[i].intensity;
    output_cloud.at(i) = p;
  }
}

}  // namespace pcl

namespace cocolic {

struct cloud_point_index_idx {
  unsigned int idx;
  unsigned int cloud_point_index;

  cloud_point_index_idx() = default;
  cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_)
      : idx(idx_), cloud_point_index(cloud_point_index_) {}
  bool operator<(const cloud_point_index_idx& p) const { return (idx < p.idx); }
};

template <typename PointType>
class VoxelFilter {
 public:
  VoxelFilter() {}

  void SetResolution(float resolution) {
    resolution_ = resolution;
    inverse_resolution_ = 1.0 / resolution_;
  }

  void SetInputCloud(
      const typename pcl::PointCloud<PointType>::Ptr input_cloud) {
    input_cloud_ = input_cloud;
  }

  float PointDistanceSquare(Eigen::Vector3f p1, Eigen::Vector3f p2) {
    return (p1[0] - p2[0]) * (p1[0] - p2[0]) +
           (p1[1] - p2[1]) * (p1[1] - p2[1]) +
           (p1[2] - p2[2]) * (p1[2] - p2[2]);
  }

  void Filter(typename pcl::PointCloud<PointType>::Ptr output_cloud) {
    pcl::getMinMax3D<PointType>(*input_cloud_, min_map_, max_map_);
    min_b_[0] = static_cast<int>(floor(min_map_[0] * inverse_resolution_));
    max_b_[0] = static_cast<int>(floor(max_map_[0] * inverse_resolution_));
    min_b_[1] = static_cast<int>(floor(min_map_[1] * inverse_resolution_));
    max_b_[1] = static_cast<int>(floor(max_map_[1] * inverse_resolution_));
    min_b_[2] = static_cast<int>(floor(min_map_[2] * inverse_resolution_));
    max_b_[2] = static_cast<int>(floor(max_map_[2] * inverse_resolution_));

    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(input_cloud_->points.size());

    for (unsigned int i = 0; i < input_cloud_->points.size(); i++) {
      PointType p = input_cloud_->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;

      int ijk0 = static_cast<int>(floor(p.x * inverse_resolution_) -
                                  static_cast<float>(min_b_[0]));
      int ijk1 = static_cast<int>(floor(p.y * inverse_resolution_) -
                                  static_cast<float>(min_b_[1]));
      int ijk2 = static_cast<int>(floor(p.z * inverse_resolution_) -
                                  static_cast<float>(min_b_[2]));

      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(static_cast<unsigned int>(idx), i);
    }

    auto rightshift_func = [](const cloud_point_index_idx& x,
                              const unsigned offset) {
      return x.idx >> offset;
    };
    boost::sort::spreadsort::integer_sort(index_vector.begin(),
                                          index_vector.end(), rightshift_func);

    unsigned int total = 0;
    unsigned int index = 0;

    std::vector<std::pair<unsigned int, unsigned int> >
        first_and_last_indices_vector;
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size()) {
      unsigned int i = index + 1;
      while (i < index_vector.size() &&
             index_vector[i].idx == index_vector[index].idx)
        ++i;
      if (i - index >= 0) {
        ++total;
        first_and_last_indices_vector.emplace_back(index, i);
      }
      index = i;
    }

    for (auto leaf : first_and_last_indices_vector) {
      Eigen::Vector3f centroid(0, 0, 0);
      for (unsigned int i = leaf.first; i < leaf.second; i++) {
        centroid += Eigen::Vector3f(
            input_cloud_->points[index_vector[i].cloud_point_index].x,
            input_cloud_->points[index_vector[i].cloud_point_index].y,
            input_cloud_->points[index_vector[i].cloud_point_index].z);
      }
      centroid /= static_cast<float>(leaf.second - leaf.first);
      PointType p;
      float dis = 10000;
      for (unsigned int i = leaf.first; i < leaf.second; i++) {
        Eigen::Vector3f cp = Eigen::Vector3f(
            input_cloud_->points[index_vector[i].cloud_point_index].x,
            input_cloud_->points[index_vector[i].cloud_point_index].y,
            input_cloud_->points[index_vector[i].cloud_point_index].z);
        float disSqu = PointDistanceSquare(cp, centroid);
        if (disSqu <= dis) {
          p = input_cloud_->points[index_vector[i].cloud_point_index];
          dis = disSqu;
        }
      }

      output_cloud->push_back(p);
    }
  }

 private:
  float resolution_;
  float inverse_resolution_;

  typename pcl::PointCloud<PointType>::Ptr input_cloud_;

  Eigen::Vector4f min_map_;
  Eigen::Vector4f max_map_;

  Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
};

}  // namespace cocolic
