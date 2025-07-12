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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <livox_ros_driver/CustomMsg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <algorithm>
#include <map>
#include <vector>

#include <lidar/livox_feature_extraction.h>
#include <lidar/velodyne_feature_extraction.h>

#include <utils/log_utils.h>
#include <utils/parameter_struct.h>
#include <utils/eigen_utils.hpp>

namespace cocolic
{

  enum OdometryMode
  {
    LIO = 0, //
    LICO = 1,
  };

  enum LiDARType
  {
    VLP = 0,
    LIVOX,
  };

  struct NextMsgs
  {
    NextMsgs()
        : scan_num(0),
          lidar_timestamp(-1),
          lidar_max_timestamp(-1),
          lidar_raw_cloud(new RTPointCloud),
          lidar_surf_cloud(new RTPointCloud),
          lidar_corner_cloud(new RTPointCloud),
          if_have_image(false),
          image_timestamp(-1),
          image(cv::Mat()) {}

    void Clear()
    {
      scan_num = 0;
      lidar_timestamp = -1;
      lidar_max_timestamp = -1;
      lidar_raw_cloud->clear();
      lidar_surf_cloud->clear();
      lidar_corner_cloud->clear();

      // image_feature_msgs.clear();
    }

    void CheckData()
    {
      double max_time[3];
      max_time[0] = pcl::GetCloudMaxTimeNs(lidar_surf_cloud) * NS_TO_S;
      max_time[1] = pcl::GetCloudMaxTimeNs(lidar_corner_cloud) * NS_TO_S;
      max_time[2] = pcl::GetCloudMaxTimeNs(lidar_raw_cloud) * NS_TO_S;
      LOG(INFO) << "[surf | corn | raw | max] " << max_time[0] << " " << max_time[1] << " " << max_time[2] << " " << lidar_max_timestamp * NS_TO_S;
      for (int i = 0; i < 3; i++)
      {
        if ((max_time[i] - lidar_max_timestamp * NS_TO_S) > 1e-6)
          std::cout << YELLOW << "[CheckData] Problem !! " << i
                    << " desired max time: " << lidar_max_timestamp * NS_TO_S
                    << "; computed max_time: " << max_time[i] << "\n"
                    << RESET;
      }

      if (image.empty())
      {
        // std::cout << "[CheckData current scan img empty]\n";
      }
    }

    int scan_num = 0;
    int64_t lidar_timestamp;     // w.r.t. the start time of the trajectory
    int64_t lidar_max_timestamp; // w.r.t. the start time of the trajectory
    RTPointCloud::Ptr lidar_raw_cloud;
    RTPointCloud::Ptr lidar_surf_cloud;
    RTPointCloud::Ptr lidar_corner_cloud;

    bool if_have_image;      // if has image in current time interval
    int64_t image_timestamp; // w.r.t. the start time of the trajectory
    cv::Mat image;           // raw image
  };

  struct LiDARCloudData
  {
    LiDARCloudData()
        : lidar_id(0),
          timestamp(0),
          max_timestamp(0),
          raw_cloud(new RTPointCloud),
          surf_cloud(new RTPointCloud),
          corner_cloud(new RTPointCloud),
          is_time_wrt_traj_start(false) {}
    int lidar_id;
    int64_t timestamp;
    int64_t max_timestamp;

    RTPointCloud::Ptr raw_cloud;
    RTPointCloud::Ptr surf_cloud;
    RTPointCloud::Ptr corner_cloud;

    bool is_time_wrt_traj_start;

    void ToRelativeMeasureTime(int64_t traj_start_time)
    {
      CloudToRelativeMeasureTime(raw_cloud, timestamp, traj_start_time);
      CloudToRelativeMeasureTime(surf_cloud, timestamp, traj_start_time);
      CloudToRelativeMeasureTime(corner_cloud, timestamp, traj_start_time);
      timestamp -= traj_start_time;

      max_timestamp = pcl::GetCloudMaxTimeNs(raw_cloud);

      int64_t surf_max = pcl::GetCloudMaxTimeNs(surf_cloud);
      int64_t corner_max = pcl::GetCloudMaxTimeNs(corner_cloud);
      if (surf_max > max_timestamp || corner_max > max_timestamp)
      {
        std::cout << RED << "surf/corner cloud max time wrong!" << RESET << std::endl;
      }

      is_time_wrt_traj_start = true;
    }

    /// Sort function to allow for using of STL containers
    bool operator<(const LiDARCloudData &other) const
    {
      if (timestamp == other.timestamp)
      {
        if (max_timestamp == other.max_timestamp)
          return lidar_id < other.lidar_id; // 2
        else
          return max_timestamp < other.max_timestamp; // 1
      }
      else
      {
        return timestamp < other.timestamp; // 0
      }
    }
  };

  struct ImageData
  {
    ImageData() : timestamp(0), is_time_wrt_traj_start(false) {}

    void ToRelativeMeasureTime(int64_t traj_start_time)
    {
      timestamp -= traj_start_time;
      is_time_wrt_traj_start = true;
    }

    int64_t timestamp;
    cv::Mat image;
    bool is_time_wrt_traj_start;
  };

  class MsgManager
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<MsgManager> Ptr;

    MsgManager(const YAML::Node &node, ros::NodeHandle &nh);

    // 
    void SpinBagOnce();

    // 
    bool GetMsgs(NextMsgs &msgs, int64_t traj_last_max, int64_t traj_max, int64_t start_time);

    int64_t GetCurIMUTimestamp() const { return cur_imu_timestamp_; }

    // 
    void LogInfo() const;

    int NumLiDAR() const { return num_lidars_; }

    inline void IMUMsgToIMUData(const sensor_msgs::Imu::ConstPtr &imu_msg,
                                IMUData &data)
    {
      data.timestamp = imu_msg->header.stamp.toSec() * S_TO_NS;
      data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                  imu_msg->angular_velocity.y,
                                  imu_msg->angular_velocity.z);
      if (if_normalized_)
      {
        data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x * 9.81,
                                     imu_msg->linear_acceleration.y * 9.81,
                                     imu_msg->linear_acceleration.z * 9.81);
      }
      else
      {
        data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                     imu_msg->linear_acceleration.y,
                                     imu_msg->linear_acceleration.z);
      }
      //  imu_msg->linear_acceleration.z + 9.81);
      Eigen::Vector4d q(imu_msg->orientation.w, imu_msg->orientation.x,
                        imu_msg->orientation.y, imu_msg->orientation.z);
      if (std::fabs(q.norm() - 1) < 0.01)
      {
        data.orientation = SO3d(Eigen::Quaterniond(q[0], q[1], q[2], q[3]));
      }
    }

    static void CheckLidarMsgTimestamp(double ros_bag_time, double msg_time)
    {
      // 
      double delta_time = ros_bag_time - msg_time;
      if (delta_time < 0.08)
      {
        LOG(INFO) << "[CheckLidarMsgTimestamp] Delta Time : " << delta_time;
      }
    }

    // 
    void RemoveBeginData(int64_t start_time, int64_t relative_start_time = 0);

  private:
    void LoadBag(const YAML::Node &node);

    bool HasEnvMsg() const;

    // 
    bool CheckMsgIsReady(double traj_max, double start_time, double knot_dt,
                         bool in_scan_unit) const;

    bool AddImageToMsg(NextMsgs &msgs, const ImageData &image, int64_t traj_max);

    // 
    bool AddToMsg(NextMsgs &msgs, std::deque<LiDARCloudData>::iterator scan, int64_t traj_max);

    void IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg);

    void VelodyneMsgHandle(const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg,
                           int lidar_id);
    void VelodyneMsgHandleNoFeature(
        const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id);

        void LivoxMsgHandle(const livox_ros_driver::CustomMsg::ConstPtr &livox_msg,
                            int lidar_id);

    void ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg);
    void ImageMsgHandle(const sensor_msgs::CompressedImageConstPtr &msg);

  public:
    bool has_valid_msg_;

    std::string bag_path_;

    NextMsgs cur_msgs;
    NextMsgs next_msgs;
    NextMsgs next_next_msgs;

    double t_offset_imu_;
    double t_offset_camera_;

    int64_t imu_period_ns_;
    // double add_extra_timeoffset_s_;

    double t_image_ms_;
    double t_lidar_ms_;

    std::deque<ImageData> image_buf_;
    std::vector<int64_t> nerf_time_;
    Eigen::aligned_deque<IMUData> imu_buf_;
    std::deque<LiDARCloudData> lidar_buf_;
    std::vector<int64_t> lidar_max_timestamps_;
    int64_t image_max_timestamp_;

    Eigen::aligned_deque<PoseData> pose_buf_;
    PoseData init_pose_;

    int64_t cur_imu_timestamp_;
    int64_t cur_pose_timestamp_;

  private:
    // int64_t cur_imu_timestamp_;
    // int64_t cur_pose_timestamp_;

    bool use_image_;

    bool lidar_timestamp_end_;
    bool remove_wrong_time_imu_;
    bool if_normalized_;
    bool if_compressed_;

    std::string imu_topic_;
    int num_lidars_;
    std::vector<LiDARType> lidar_types;
    std::vector<std::string> lidar_topics_;
    std::string image_topic_;

    // std::string pose_topic_;

    std::vector<ExtrinsicParam> EP_LktoI_;

    /// lidar_k 到 lidar_0 的外参
    Eigen::aligned_vector<Eigen::Matrix4d> T_LktoL0_vec_;

    rosbag::Bag bag_;
    rosbag::View view_;

    ros::Subscriber sub_imu_;
    std::vector<ros::Subscriber> subs_vlp16_;
    std::vector<ros::Subscriber> subs_livox_;
    ros::Subscriber sub_image_;

    VelodyneFeatureExtraction::Ptr velodyne_feature_extraction_;
    LivoxFeatureExtraction::Ptr livox_feature_extraction_;

    ros::Publisher pub_img_;
  };

} // namespace cocolic
