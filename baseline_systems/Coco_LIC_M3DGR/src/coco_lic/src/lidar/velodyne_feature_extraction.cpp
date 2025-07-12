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

#include "velodyne_feature_extraction.h"
#include <cocolic/feature_cloud.h>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg

using namespace std;

namespace cocolic
{

  VelodyneFeatureExtraction::VelodyneFeatureExtraction(const YAML::Node &node)
      : fea_param_(LiDARFeatureParam(node["VLP16"]))
  {
    n_scan = node["VLP16"]["N_SCAN"].as<int>();
    horizon_scan = node["VLP16"]["Horizon_SCAN"].as<int>();

    pub_corner_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/corner_cloud", 10);
    pub_surface_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/surface_cloud", 10);
    pub_full_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("lidar_feature/full_cloud", 10);

    pub_feature_cloud =
        nh.advertise<cocolic::feature_cloud>("lidar_feature/feature_cloud", 10);

    AllocateMemory();
    ResetParameters();
  }

  void VelodyneFeatureExtraction::AllocateMemory()
  {
    p_full_cloud.reset(new RTPointCloud());
    p_full_cloud->resize(n_scan * horizon_scan);

    range_mat = cv::Mat(n_scan, horizon_scan, CV_32F, cv::Scalar::all(FLT_MAX));

    p_extracted_cloud.reset(new RTPointCloud());

    p_corner_cloud.reset(new RTPointCloud());
    p_surface_cloud.reset(new RTPointCloud());

    point_range_list.assign(n_scan * horizon_scan, 0);
    point_column_id.assign(n_scan * horizon_scan, 0);
    start_ring_index.assign(n_scan, 0);
    end_ring_index.assign(n_scan, 0);

    cloud_smoothness.resize(n_scan * horizon_scan);

    cloud_curvature = new float[n_scan * horizon_scan];
    cloud_neighbor_picked = new int[n_scan * horizon_scan];
    cloud_label = new int[n_scan * horizon_scan];

    down_size_filter.SetResolution(fea_param_.odometry_surface_leaf_size);
  }

  void VelodyneFeatureExtraction::ResetParameters()
  {
    p_full_cloud->clear();
    p_full_cloud->resize(n_scan * horizon_scan);
    p_extracted_cloud->clear();
    range_mat = cv::Mat(n_scan, horizon_scan, CV_32F, cv::Scalar::all(FLT_MAX));
  }

  void VelodyneFeatureExtraction::LidarHandler(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg)
  {
    RTPointCloud::Ptr cur_cloud(new RTPointCloud());
    bool check_field_passed = ParsePointCloud(lidar_msg, cur_cloud);

    if (!check_field_passed)
      return;

    LidarHandler(cur_cloud);
  }

  void VelodyneFeatureExtraction::LidarHandler(
      const RTPointCloud::Ptr raw_cloud)
  {
    p_corner_cloud.reset(new RTPointCloud());
    p_surface_cloud.reset(new RTPointCloud());

    if (raw_cloud->isOrganized())
      OrganizedCloudToRangeImage(raw_cloud, range_mat, p_full_cloud);
    else
      RTCloudToRangeImage(raw_cloud, range_mat, p_full_cloud);

    // [range_mat] and [p_full_cloud] are ready.
    CloudExtraction();
    CaculateSmoothness();
    MarkOccludedPoints();
    ExtractFeatures();
    ResetParameters();

    PublishCloud("map");
  }

  bool VelodyneFeatureExtraction::CheckMsgFields(
      const sensor_msgs::PointCloud2 &cloud_msg, std::string fields_name)
  {
    bool flag = false;
    for (size_t i = 0; i < cloud_msg.fields.size(); ++i)
    {
      if (cloud_msg.fields[i].name == fields_name)
      {
        flag = true;
        break;
      }
    }

    // if (!flag) {
    //   LOG(WARNING) << "PointCloud2 channel [" << fields_name
    //                << "] not available, please configure your point cloud
    //                data!";
    //   // ros::shutdown();
    // }
    return flag;
  }

  bool VelodyneFeatureExtraction::ParsePointCloud(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud) const
  {
    static bool has_checked = false;
    static bool check_field_passed = false;
    static bool has_t_field = false;
    static bool has_time_field = false;
    static bool has_timestamp_field = false;

    /// Check ring channel and point time for the first msg
    if (!has_checked)
    {
      has_checked = true;
      bool has_ring_field = CheckMsgFields(*lidar_msg, "ring");
      has_time_field = CheckMsgFields(*lidar_msg, "time");           // float s -> Velodyne: lvi、lio
      has_t_field = CheckMsgFields(*lidar_msg, "t");                 // uint32_t ns -> Ouster: viral
      has_timestamp_field = CheckMsgFields(*lidar_msg, "timestamp"); // float s -> Hesai-PandarQT

      check_field_passed = has_ring_field && (has_time_field || has_t_field || has_timestamp_field);

      if (!has_ring_field)
        LOG(WARNING) << "[ParsePointCloud] input cloud NOT has [ring] field";

      if (!has_time_field && !has_t_field && !has_timestamp_field)
        LOG(WARNING)
            << "[ParsePointCloud] input cloud NOT has [time] or [t] or [timestamp] field";
    }

    /// convert cloud
    if (check_field_passed)
    {
      if (has_time_field)
      {
        // for velodyne
        RTPointCloudTmp::Ptr tmp_out_cloud(new RTPointCloudTmp);
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        RTPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);
      }
      else if (has_t_field)
      {
        // for ouster 64
        OusterPointCloudTmp::Ptr tmp_out_cloud(new OusterPointCloudTmp());
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        OusterPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);
      }
      else if (has_timestamp_field)
      {
        // for hesai pandar
        RTPointCloudTmpHesai::Ptr tmp_out_cloud(new RTPointCloudTmpHesai);
        pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

        RTPointCloudTmp2RTPointCloudHesai(tmp_out_cloud, out_cloud);
      }
    }
    return check_field_passed;
  }

  bool VelodyneFeatureExtraction::ParsePointCloudNoFeature(
      const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud)
  {
    RTPointCloudTmp::Ptr tmp_out_cloud(new RTPointCloudTmp);
    pcl::fromROSMsg(*lidar_msg, *tmp_out_cloud);

    int plsize = tmp_out_cloud->points.size();

    p_corner_cloud.reset(new RTPointCloud());
    p_surface_cloud.reset(new RTPointCloud());
    p_full_cloud.reset(new RTPointCloud());

    // p_corner_cloud->reserve(plsize);
    // p_surface_cloud->reserve(plsize);
    // p_full_cloud->resize(plsize);

    std::cout << "[plsize] " << plsize << std::endl;

    RTPointCloudTmp2RTPointCloud(tmp_out_cloud, out_cloud);

    for (int i = 0; i < plsize; i++)
    {
      const auto &pt = tmp_out_cloud->points[i];
      RTPoint added_pt;
      added_pt.x = pt.x;
      added_pt.y = pt.y;
      added_pt.z = pt.z;
      added_pt.intensity = pt.intensity;
      added_pt.ring = pt.ring;
      added_pt.time = int64_t(pt.time * 1e9);

      if (i % 4 == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (2.0 * 2.0))
        {
            // p_surface_cloud->points.push_back(added_pt);
            p_surface_cloud->push_back(added_pt);
        }
      }
    }

    /// 
    p_full_cloud->push_back((*p_surface_cloud)[0]);
    p_corner_cloud->push_back((*p_surface_cloud)[0]);

    return true;
  }

  void VelodyneFeatureExtraction::OrganizedCloudToRangeImage(
      const RTPointCloud::Ptr cur_cloud, cv::Mat &dist_image,
      RTPointCloud::Ptr &corresponding_cloud) const
  {
    for (size_t column_id = 0; column_id < cur_cloud->width; ++column_id)
    {
      for (size_t row_id = 0; row_id < cur_cloud->height; ++row_id)
      {
        const RTPoint &p = cur_cloud->at(column_id, row_id);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;

        if (row_id < 0 || row_id >= size_t(n_scan))
          continue;
        if (column_id < 0 || column_id >= size_t(horizon_scan))
          continue;

        float range = pcl::PointNorm<RTPoint>(p);
        if (range < fea_param_.min_distance || range > fea_param_.max_distance)
          continue;
        if (dist_image.at<float>(row_id, column_id) != FLT_MAX)
          continue;
        dist_image.at<float>(row_id, column_id) = range;

        /// 
        int index = column_id + row_id * horizon_scan;
        corresponding_cloud->points[index] = p;
      }
    }
  }

  void VelodyneFeatureExtraction::RTCloudToRangeImage(
      const RTPointCloud::Ptr cur_cloud, cv::Mat &dist_image,
      RTPointCloud::Ptr &corresponding_cloud) const
  {
    static float angle_resolution = 360.0 / float(horizon_scan);
    static float rad2deg = 180.0 / M_PI;

    for (const RTPoint &p : cur_cloud->points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      int row_id = p.ring;
      if (row_id < 0 || row_id >= n_scan)
        continue;
      float horizon_angle = atan2(p.x, p.y) * rad2deg;

      int column_id =
          -round((horizon_angle - 90.0) / angle_resolution) + horizon_scan / 2;
      if (column_id >= horizon_scan)
        column_id -= horizon_scan;
      if (column_id < 0 || column_id >= horizon_scan)
        continue;

      float range = pcl::PointNorm<RTPoint>(p);
      if (range < fea_param_.min_distance || range > fea_param_.max_distance)
        continue;

      if (dist_image.at<float>(row_id, column_id) != FLT_MAX)
        continue;
      dist_image.at<float>(row_id, column_id) = range;

      int index = column_id + row_id * horizon_scan;
      corresponding_cloud->points[index] = p;
    }
  }

  void VelodyneFeatureExtraction::CloudExtraction()
  {
    int point_index = 0;
    for (int i = 0; i < n_scan; i++)
    {
      start_ring_index[i] = point_index - 1 + 5;
      for (int j = 0; j < horizon_scan; j++)
      {
        if (range_mat.at<float>(i, j) != FLT_MAX)
        {
          // mark the points' column index for marking occlusion later
          point_column_id[point_index] = j;
          // save range info
          point_range_list[point_index] = range_mat.at<float>(i, j);
          // save extracted cloud
          p_extracted_cloud->push_back(
              p_full_cloud->points[j + i * horizon_scan]);
          // size of extracted cloud
          point_index++;
        }
      }
      end_ring_index[i] = point_index - 1 - 5;
    }
  }

  void VelodyneFeatureExtraction::CaculateSmoothness()
  {
    for (size_t i = 5; i < p_extracted_cloud->points.size() - 5; i++)
    {
      float diff_range = point_range_list[i - 5] + point_range_list[i - 4] +
                         point_range_list[i - 3] + point_range_list[i - 2] +
                         point_range_list[i - 1] - point_range_list[i] * 10 +
                         point_range_list[i + 1] + point_range_list[i + 2] +
                         point_range_list[i + 3] + point_range_list[i + 4] +
                         point_range_list[i + 5];
      cloud_curvature[i] = diff_range * diff_range;
      cloud_neighbor_picked[i] = 0;
      cloud_label[i] = 0;
      cloud_smoothness[i].value = cloud_curvature[i];
      cloud_smoothness[i].ind = i;
    }
  }

  void VelodyneFeatureExtraction::MarkOccludedPoints()
  {
    for (size_t i = 5; i < p_extracted_cloud->points.size() - 6; i++)
    {
      float depth1 = point_range_list[i];
      float depth2 = point_range_list[i + 1];
      int column_diff =
          std::abs(int(point_column_id[i + 1] - point_column_id[i]));

      if (column_diff < 10)
      {
        if (depth1 - depth2 > 0.3)
        {
          cloud_neighbor_picked[i - 5] = 1;
          cloud_neighbor_picked[i - 4] = 1;
          cloud_neighbor_picked[i - 3] = 1;
          cloud_neighbor_picked[i - 2] = 1;
          cloud_neighbor_picked[i - 1] = 1;
          cloud_neighbor_picked[i] = 1;
        }
        else if (depth2 - depth1 > 0.3)
        {
          cloud_neighbor_picked[i + 1] = 1;
          cloud_neighbor_picked[i + 2] = 1;
          cloud_neighbor_picked[i + 3] = 1;
          cloud_neighbor_picked[i + 4] = 1;
          cloud_neighbor_picked[i + 5] = 1;
          cloud_neighbor_picked[i + 6] = 1;
        }
      }

      float diff1 =
          std::abs(float(point_range_list[i - 1] - point_range_list[i]));
      float diff2 =
          std::abs(float(point_range_list[i + 1] - point_range_list[i]));

      if (diff1 > 0.02 * point_range_list[i] &&
          diff2 > 0.02 * point_range_list[i])
        cloud_neighbor_picked[i] = 1;
    }
  }

  void VelodyneFeatureExtraction::ExtractFeatures()
  {
    RTPointCloud::Ptr surface_cloud_scan(new RTPointCloud());
    RTPointCloud::Ptr surface_cloud_scan_downsample(new RTPointCloud());

    for (int i = 0; i < n_scan; i++)
    {
      surface_cloud_scan->clear();

      /// 
      for (int j = 0; j < 6; j++)
      {
        int sp = (start_ring_index[i] * (6 - j) + end_ring_index[i] * j) / 6;
        int ep =
            (start_ring_index[i] * (5 - j) + end_ring_index[i] * (j + 1)) / 6 - 1;
        if (sp >= ep)
          continue;
        std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep,
                  by_value());

        /// 
        int largest_picked_num = 0;
        for (int k = ep; k >= sp; k--)
        {
          int index = cloud_smoothness[k].ind;
          if (cloud_neighbor_picked[index] == 0 &&
              cloud_curvature[index] > fea_param_.edge_threshold)
          {
            largest_picked_num++;
            if (largest_picked_num <= 20)
            {
              cloud_label[index] = 1;
              p_corner_cloud->push_back(p_extracted_cloud->points[index]);
            }
            else
            {
              break;
            }

            cloud_neighbor_picked[index] = 1;
            for (int l = 1; l <= 5; l++)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l - 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l + 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
          }
        }

        /// 
        for (int k = sp; k <= ep; k++)
        {
          int index = cloud_smoothness[k].ind;
          if (cloud_neighbor_picked[index] == 0 &&
              cloud_curvature[index] < fea_param_.surf_threshold)
          {
            cloud_label[index] = -1;
            cloud_neighbor_picked[index] = 1;

            for (int l = 1; l <= 5; l++)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l - 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
            for (int l = -1; l >= -5; l--)
            {
              int column_diff = std::abs(int(point_column_id[index + l] -
                                             point_column_id[index + l + 1]));
              if (column_diff > 10)
                break;
              cloud_neighbor_picked[index + l] = 1;
            }
          }
        }
        for (int k = sp; k <= ep; k++)
        {
          if (cloud_label[k] <= 0)
          {
            surface_cloud_scan->push_back(p_extracted_cloud->points[k]);
          }
        }
      }

      surface_cloud_scan_downsample->clear();
      down_size_filter.SetInputCloud(surface_cloud_scan);
      down_size_filter.Filter(surface_cloud_scan_downsample);
      *p_surface_cloud += *surface_cloud_scan_downsample;
    }
  }

  void VelodyneFeatureExtraction::PublishCloud(std::string frame_id)
  {
    bool pub_fea = (pub_full_cloud.getNumSubscribers() != 0);

    cocolic::feature_cloud feature_msg;
    if (pub_fea || pub_corner_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 corner_msg;
      pcl::toROSMsg(*p_corner_cloud, corner_msg);
      corner_msg.header.stamp = ros::Time::now();
      corner_msg.header.frame_id = frame_id;

      pub_corner_cloud.publish(corner_msg);
      feature_msg.corner_cloud = corner_msg;
    }
    if (pub_fea || pub_surface_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 surface_msg;
      pcl::toROSMsg(*p_surface_cloud, surface_msg);
      surface_msg.header.stamp = ros::Time::now();
      surface_msg.header.frame_id = frame_id;

      pub_surface_cloud.publish(surface_msg);
      feature_msg.surface_cloud = surface_msg;
    }

    if (pub_fea || pub_full_cloud.getNumSubscribers() != 0)
    {
      sensor_msgs::PointCloud2 full_msg;
      pcl::toROSMsg(*p_full_cloud, full_msg);
      full_msg.header.stamp = ros::Time::now();
      full_msg.header.frame_id = frame_id;

      pub_full_cloud.publish(full_msg);
      feature_msg.full_cloud = full_msg;
    }

    if (pub_fea)
    {
      feature_msg.header.stamp = ros::Time::now();
      feature_msg.header.frame_id = frame_id;

      pub_feature_cloud.publish(feature_msg);
    }

    //  rosbag::Bag bagWrite;
    //  bagWrite.open("/home/ha/rosbag/liso-bag/simu_bag/sim_feature.bag",
    //  rosbag::bagmode::Append); bagWrite.write("/feature_cloud",
    //  feature_msg.header.stamp, feature_msg); bagWrite.close();
  }

} // namespace cocolic
