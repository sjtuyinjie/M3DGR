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

#include <Eigen/Eigen>
// #include <ceres/ceres.h>
#include <utils/mypcl_cloud_type.h>
#include <yaml-cpp/yaml.h>

namespace cocolic
{

  struct LiDARCloud
  {
    LiDARCloud() : timestamp(0), time_max(0), full_cloud(new PosCloud) {}

    int64_t timestamp;
    int64_t time_max; // [timestamp, time_max] full_features

    PosCloud::Ptr full_cloud;
  };

  struct LiDARFeature : public LiDARCloud
  {
    LiDARFeature()
        : corner_features(new PosCloud), surface_features(new PosCloud) {}

    //
    LiDARFeature(const LiDARFeature &fea)
    {
      timestamp = fea.timestamp;
      time_max = fea.time_max;
      corner_features = PosCloud::Ptr(new PosCloud);
      surface_features = PosCloud::Ptr(new PosCloud);
      full_cloud = PosCloud::Ptr(new PosCloud);

      *corner_features = *fea.corner_features;
      *surface_features = *fea.surface_features;
      *full_cloud = *fea.full_cloud;
    }

    LiDARFeature &operator=(const LiDARFeature &fea)
    {
      if (this != &fea)
      {
        LiDARFeature temp(fea);
        this->timestamp = temp.timestamp;
        this->time_max = temp.time_max;

        PosCloud::Ptr p_temp = temp.corner_features;
        temp.corner_features = this->corner_features;
        this->corner_features = p_temp;

        p_temp = temp.surface_features;
        temp.surface_features = this->surface_features;
        this->surface_features = p_temp;

        p_temp = temp.full_cloud;
        temp.full_cloud = this->full_cloud;
        this->full_cloud = p_temp;
      }

      return *this;
    }

    void Clear()
    {
      timestamp = 0;
      time_max = 0;
      corner_features->clear();
      surface_features->clear();
      full_cloud->clear();
    }

    PosCloud::Ptr corner_features;
    PosCloud::Ptr surface_features;
  };

  struct LiDARFeatureParam
  {
    LiDARFeatureParam(const YAML::Node &node)
    {
      edge_threshold = node["edge_threshold"].as<float>();
      surf_threshold = node["surf_threshold"].as<float>();
      odometry_surface_leaf_size = node["odometry_surface_leaf_size"].as<float>();

      min_distance = node["min_distance"].as<double>();
      max_distance = node["max_distance"].as<double>();
    }

    /// LOAM feature threshold
    float edge_threshold;
    float surf_threshold;

    double min_distance;
    double max_distance;

    float odometry_surface_leaf_size;
  };

  enum GeometryType
  {
    Line = 0,
    Plane
  };

  struct PointCorrespondence
  {
    int64_t t_point;
    int64_t t_map;
    double scale;
    Eigen::Vector3d point;
    Eigen::Vector3d point_raw; // 

    GeometryType geo_type;

    // 
    Eigen::Vector4d geo_plane;

    // 
    Eigen::Vector3d geo_normal;
    Eigen::Vector3d geo_point;
  };

} // namespace cocolic
