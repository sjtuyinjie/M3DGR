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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "lidar_feature.h"

#include <utils/cloud_tool.h>
#include <utils/mypcl_cloud_type.h>
#include <utils/parameter_struct.h>

// #include <livox_ros_driver/CustomMsg.h>
#include <cmath>

namespace cocolic
{

  struct smoothness_t
  {
    float value;
    size_t ind;
  };

  struct by_value
  {
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
      return left.value < right.value;
    }
  };

  class VelodyneFeatureExtraction
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<VelodyneFeatureExtraction> Ptr;

    VelodyneFeatureExtraction(const YAML::Node &node);

    void LidarHandler(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg);

    void LidarHandler(const RTPointCloud::Ptr raw_cloud);

    // 
    bool ParsePointCloud(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                         RTPointCloud::Ptr out_cloud) const;
    bool ParsePointCloudNoFeature(
        const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
        RTPointCloud::Ptr out_cloud);

    static bool CheckMsgFields(const sensor_msgs::PointCloud2 &cloud_msg,
                               std::string fields_name = "time");

    inline RTPointCloud::Ptr GetCornerFeature() const { return p_corner_cloud; }

    inline RTPointCloud::Ptr GetSurfaceFeature() const { return p_surface_cloud; }

  private:
    void AllocateMemory();

    void ResetParameters();

    // 
    void OrganizedCloudToRangeImage(const RTPointCloud::Ptr cur_cloud,
                                    cv::Mat &dist_image,
                                    RTPointCloud::Ptr &corresponding_cloud) const;

    // 
    void RTCloudToRangeImage(const RTPointCloud::Ptr cur_cloud,
                             cv::Mat &dist_image,
                             RTPointCloud::Ptr &corresponding_cloud) const;

    void CloudExtraction();

    // 
    void CaculateSmoothness();

    // 
    void MarkOccludedPoints();

    // 
    void ExtractFeatures();

    // 
    void PublishCloud(std::string frame_id);

  private:
    ros::NodeHandle nh;

    ros::Subscriber sub_lidar;
    ros::Publisher pub_corner_cloud;
    ros::Publisher pub_surface_cloud;
    ros::Publisher pub_full_cloud;
    ros::Publisher pub_feature_cloud;

    LiDARFeatureParam fea_param_;

    // 
    int n_scan;
    int horizon_scan;
    cv::Mat range_mat;
    RTPointCloud::Ptr p_full_cloud;

    RTPointCloud::Ptr p_extracted_cloud;

    std::vector<float> point_range_list;
    std::vector<int> point_column_id;
    std::vector<int> start_ring_index;
    std::vector<int> end_ring_index;

    std::vector<smoothness_t> cloud_smoothness;
    float *cloud_curvature;
    int *cloud_neighbor_picked;
    int *cloud_label;

    /// 
    RTPointCloud::Ptr p_corner_cloud;
    RTPointCloud::Ptr p_surface_cloud;

    VoxelFilter<RTPoint> down_size_filter;

    MODE work_mode_;
  };

} // namespace cocolic
