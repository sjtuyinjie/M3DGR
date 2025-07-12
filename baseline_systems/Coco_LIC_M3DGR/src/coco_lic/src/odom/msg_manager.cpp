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

#include <odom/msg_manager.h>
#include <utils/parameter_struct.h>

#include <pcl/common/transforms.h>

namespace cocolic
{

  MsgManager::MsgManager(const YAML::Node &node, ros::NodeHandle &nh)
      : has_valid_msg_(true),
        t_offset_imu_(0),
        t_offset_camera_(0),
        cur_imu_timestamp_(-1),
        cur_pose_timestamp_(-1),
        use_image_(false),
        lidar_timestamp_end_(false),
        remove_wrong_time_imu_(false),
        if_normalized_(false),
        image_topic_("")
  {
    std::string config_path = node["config_path"].as<std::string>();

    OdometryMode odom_mode = OdometryMode(node["odometry_mode"].as<int>());

    nh.param<std::string>("bag_path", bag_path_, "");
    if (bag_path_ == "")
    {
      bag_path_ = node["bag_path"].as<std::string>();
    }

    /// imu topic
    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);
    imu_topic_ = imu_node["imu_topic"].as<std::string>();
    // pose_topic_ = imu_node["pose_topic"].as<std::string>();
    // remove_wrong_time_imu_ = imu_node["remove_wrong_time_imu"].as<bool>();
    if_normalized_ = imu_node["if_normalized"].as<bool>();

    // double imu_frequency = node["imu_frequency"].as<double>();
    // double imu_period_s = 1. / imu_frequency;

    std::string cam_yaml = node["camera_yaml"].as<std::string>();
    YAML::Node cam_node = YAML::LoadFile(config_path + cam_yaml);
    if_compressed_ = cam_node["if_compressed"].as<bool>();

    // add_extra_timeoffset_s_ =
    //     yaml::GetValue<double>(node, "add_extra_timeoffset_s", 0);
    // LOG(INFO) << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_;
    // std::cout << "add_extra_timeoffset_s: " << add_extra_timeoffset_s_ << "\n";

    /// image topic
    if (odom_mode == OdometryMode::LICO)
      use_image_ = true;
    if (use_image_)
    {
      std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
      YAML::Node cam_node = YAML::LoadFile(cam_yaml);
      image_topic_ = cam_node["image_topic"].as<std::string>();

      pub_img_ = nh.advertise<sensor_msgs::Image>("/vio/test_img", 1000);
    }
    image_max_timestamp_ = -1;

    /// lidar topic
    std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
    YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);
    num_lidars_ = lidar_node["num_lidars"].as<int>();
    lidar_timestamp_end_ = lidar_node["lidar_timestamp_end"].as<bool>();

    bool use_livox = false;
    bool use_vlp = false;
    for (int i = 0; i < num_lidars_; ++i)
    {
      std::string lidar_str = "lidar" + std::to_string(i);
      const auto &lidar_i = lidar_node[lidar_str];
      bool is_livox = lidar_i["is_livox"].as<bool>();
      if (is_livox)
      {
        lidar_types.push_back(LIVOX);
        use_livox = true;
      }
      else
      {
        lidar_types.push_back(VLP);
        use_vlp = true;
      }
      lidar_topics_.push_back(lidar_i["topic"].as<std::string>());
      EP_LktoI_.emplace_back();
      EP_LktoI_.back().Init(lidar_i["Extrinsics"]);
    }

    for (int k = 0; k < num_lidars_; ++k)
    {
      lidar_max_timestamps_.push_back(0);
      Eigen::Matrix4d T_Lk_to_L0 = Eigen::Matrix4d::Identity();
      if (k > 0)
      {
        T_Lk_to_L0.block<3, 3>(0, 0) =
            (EP_LktoI_[0].q.inverse() * EP_LktoI_[k].q).toRotationMatrix();
        T_Lk_to_L0.block<3, 1>(0, 3) =
            EP_LktoI_[0].q.inverse() * (EP_LktoI_[k].p - EP_LktoI_[0].p);

        // std::cout << "lidar " << k << "\n"
        //           << T_Lk_to_L0 << std::endl;
      }
      T_LktoL0_vec_.push_back(T_Lk_to_L0);
    }

    if (use_livox)
      livox_feature_extraction_ =
          std::make_shared<LivoxFeatureExtraction>(lidar_node);
    if (use_vlp)
      velodyne_feature_extraction_ =
          std::make_shared<VelodyneFeatureExtraction>(lidar_node);

    LoadBag(node);
  }

  void MsgManager::LoadBag(const YAML::Node &node)
  {
    double bag_start = node["bag_start"].as<double>();
    double bag_durr = node["bag_durr"].as<double>();

    std::vector<std::string> topics;
    topics.push_back(imu_topic_); // imu
    if (use_image_)               // camera
      topics.push_back(image_topic_);
    for (auto &v : lidar_topics_) // lidar
      topics.push_back(v);
    // topics.push_back(pose_topic_);

    bag_.open(bag_path_, rosbag::bagmode::Read);

    rosbag::View view_full;
    view_full.addQuery(bag_);
    ros::Time time_start = view_full.getBeginTime();
    time_start += ros::Duration(bag_start);
    ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                           : time_start + ros::Duration(bag_durr);
    view_.addQuery(bag_, rosbag::TopicQuery(topics), time_start, time_finish);
    if (view_.size() == 0)
    {
      ROS_ERROR("No messages to play on specified topics.  Exiting.");
      ros::shutdown();
      return;
    }

    std::cout << "\n🍺 LoadBag " << bag_path_ << " start at " << bag_start
              << " with duration " << (time_finish - time_start).toSec() << ".\n";
    LOG(INFO) << "LoadBag " << bag_path_ << " start at " << bag_start
              << " with duration " << (time_finish - time_start).toSec();
  }

  void MsgManager::SpinBagOnce()
  {
    static rosbag::View::iterator view_iterator = view_.begin();
    if (view_iterator == view_.end())
    {
      has_valid_msg_ = false;
      LOG(INFO) << "End of bag";
      return;
    }

    const rosbag::MessageInstance &m = *view_iterator;
    std::string msg_topic = m.getTopic();
    auto msg_time = m.getTime();

    if (msg_topic == imu_topic_)  // imu
    {
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      IMUMsgHandle(imu_msg);
    }
    else if (std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic) !=
             lidar_topics_.end())  // lidar
    {
      auto it = std::find(lidar_topics_.begin(), lidar_topics_.end(), msg_topic);
      auto idx = std::distance(lidar_topics_.begin(), it);
      if (lidar_types[idx] == VLP)  //[rotating lidar: Velodyne、Ouster、Hesai]
      {
        if (!m.isType<sensor_msgs::PointCloud2>())
          std::cout << "Wrong type\n";

        auto lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
        CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
        VelodyneMsgHandle(lidar_msg, idx);
        // VelodyneMsgHandleNoFeature(lidar_msg, idx);
      }
      else if (lidar_types[idx] == LIVOX)  //[solid-state lidar: Livox]
      {
        if (!m.isType<livox_ros_driver::CustomMsg>())
          std::cout << "Wrong type\n";

        auto lidar_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        CheckLidarMsgTimestamp(msg_time.toSec(), lidar_msg->header.stamp.toSec());
        LivoxMsgHandle(lidar_msg, idx);
      }
    }
    else if (msg_topic == image_topic_)  // camera
    {
      if (if_compressed_)
      {
        sensor_msgs::CompressedImageConstPtr image_msg = m.instantiate<sensor_msgs::CompressedImage>();
        ImageMsgHandle(image_msg);
      }
      else
      {
        sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
        ImageMsgHandle(image_msg);
      }
    }

    view_iterator++;
  }

  void MsgManager::LogInfo() const
  {
    int m_size[3] = {0, 0, 0};
    m_size[0] = imu_buf_.size();
    m_size[1] = lidar_buf_.size();
    // if (use_image_) m_size[2] = feature_tracker_node_->NumImageMsg();
    LOG(INFO) << "imu/lidar/image msg left: " << m_size[0] << "/" << m_size[1]
              << "/" << m_size[2];
  }

  void MsgManager::RemoveBeginData(int64_t start_time, // not used
                                   int64_t relative_start_time)
  { // 0
    for (auto iter = lidar_buf_.begin(); iter != lidar_buf_.end();)
    {
      if (iter->timestamp < relative_start_time)
      {
        if (iter->max_timestamp <= relative_start_time)
        { // [1]
          iter = lidar_buf_.erase(iter);
          continue;
        }
        else
        { // [2]
          // int64_t t_aft = relative_start_time + 1e-3;  //1e-3
          int64_t t_aft = relative_start_time;
          LiDARCloudData scan_bef, scan_aft;
          scan_aft.timestamp = t_aft;
          scan_aft.max_timestamp = iter->max_timestamp;
          pcl::FilterCloudByTimestamp(iter->raw_cloud, t_aft,
                                      scan_bef.raw_cloud,
                                      scan_aft.raw_cloud);
          pcl::FilterCloudByTimestamp(iter->surf_cloud, t_aft,
                                      scan_bef.surf_cloud,
                                      scan_aft.surf_cloud);
          pcl::FilterCloudByTimestamp(iter->corner_cloud, t_aft,
                                      scan_bef.corner_cloud,
                                      scan_aft.corner_cloud);

          iter->timestamp = t_aft;
          *iter->raw_cloud = *scan_aft.raw_cloud;
          *iter->surf_cloud = *scan_aft.surf_cloud;
          *iter->corner_cloud = *scan_aft.corner_cloud;
        }
      }

      iter++;
    }

    if (use_image_)
    {
      for (auto iter = image_buf_.begin(); iter != image_buf_.end();)
      {
        if (iter->timestamp < relative_start_time)
        {
          iter = image_buf_.erase(iter); //
          continue;
        }
        iter++;
      }
    }
  }

  bool MsgManager::HasEnvMsg() const
  {
    int env_msg = lidar_buf_.size();
    if (cur_imu_timestamp_ < 0 && env_msg > 100)
      LOG(WARNING) << "No IMU data. CHECK imu topic" << imu_topic_;

    return env_msg > 0;
  }

  bool MsgManager::CheckMsgIsReady(double traj_max, double start_time,
                                   double knot_dt, bool in_scan_unit) const
  {
    double t_imu_wrt_start = cur_imu_timestamp_ - start_time;

    //
    if (t_imu_wrt_start < traj_max)
    {
      return false;
    }

    // 
    int64_t t_front_lidar = -1;
    // Count how many unique lidar streams
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);

      // 
      t_front_lidar = std::max(t_front_lidar, data.max_timestamp);
    }

    // 
    if ((int)unique_lidar_ids.size() != num_lidars_)
      return false;

    // 
    int64_t t_back_lidar = lidar_max_timestamps_[0];
    for (auto t : lidar_max_timestamps_)
    {
      t_back_lidar = std::min(t_back_lidar, t);
    }

    //  
    if (in_scan_unit)
    {
      // 
      if (t_front_lidar > t_imu_wrt_start)
        return false;
    }
    else
    {
      // 
      if (t_back_lidar < traj_max)
        return false;
    }

    return true;
  }

  bool MsgManager::AddImageToMsg(NextMsgs &msgs, const ImageData &image,
                                 int64_t traj_max)
  {
    if (image.timestamp >= traj_max)
      return false;
    msgs.if_have_image = true; // important!
    msgs.image_timestamp = image.timestamp;
    msgs.image = image.image;
    // msgs.image = image.image.clone();
    return true;
  }

  bool MsgManager::AddToMsg(NextMsgs &msgs, std::deque<LiDARCloudData>::iterator scan,
                            int64_t traj_max)
  {
    bool add_entire_scan = false;
    // if (scan->timestamp > traj_max) return add_entire_scan;

    if (scan->max_timestamp < traj_max)
    { // 
      *msgs.lidar_raw_cloud += (*scan->raw_cloud);
      *msgs.lidar_surf_cloud += (*scan->surf_cloud);
      *msgs.lidar_corner_cloud += (*scan->corner_cloud);

      // 
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan->timestamp;
        msgs.lidar_max_timestamp = scan->max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan->timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan->max_timestamp);
      }

      add_entire_scan = true;
    }
    else
    { // 
      LiDARCloudData scan_bef, scan_aft;
      pcl::FilterCloudByTimestamp(scan->raw_cloud, traj_max, scan_bef.raw_cloud,
                                  scan_aft.raw_cloud);
      pcl::FilterCloudByTimestamp(scan->surf_cloud, traj_max, scan_bef.surf_cloud,
                                  scan_aft.surf_cloud);
      pcl::FilterCloudByTimestamp(scan->corner_cloud, traj_max,
                                  scan_bef.corner_cloud, scan_aft.corner_cloud);
      //
      scan_bef.timestamp = scan->timestamp;
      scan_bef.max_timestamp = traj_max - 1e-9 * S_TO_NS;
      scan_aft.timestamp = traj_max;
      scan_aft.max_timestamp = scan->max_timestamp;

      // 
      scan->timestamp = traj_max;
      // *scan.max_timestamp = ； // 
      *scan->raw_cloud = *scan_aft.raw_cloud;
      *scan->surf_cloud = *scan_aft.surf_cloud;
      *scan->corner_cloud = *scan_aft.corner_cloud;

      *msgs.lidar_raw_cloud += (*scan_bef.raw_cloud);
      *msgs.lidar_surf_cloud += (*scan_bef.surf_cloud);
      *msgs.lidar_corner_cloud += (*scan_bef.corner_cloud);

      // 
      if (msgs.scan_num == 0)
      {
        // first scan
        msgs.lidar_timestamp = scan_bef.timestamp;
        msgs.lidar_max_timestamp = scan_bef.max_timestamp;
      }
      else
      {
        msgs.lidar_timestamp =
            std::min(msgs.lidar_timestamp, scan_bef.timestamp);
        msgs.lidar_max_timestamp =
            std::max(msgs.lidar_max_timestamp, scan_bef.max_timestamp);
      }

      add_entire_scan = false;
    }

    // 
    msgs.scan_num++;

    return add_entire_scan;
  }

  /// 
  bool MsgManager::GetMsgs(NextMsgs &msgs, int64_t traj_last_max, int64_t traj_max, int64_t start_time)
  {
    msgs.Clear();

    if (imu_buf_.empty() || lidar_buf_.empty())
    {
      return false;
    }
    if (cur_imu_timestamp_ - start_time < traj_max)
    {
      return false;
    }

    /// 1 
    // 
    std::vector<int> unique_lidar_ids;
    for (const auto &data : lidar_buf_)
    {
      if (std::find(unique_lidar_ids.begin(), unique_lidar_ids.end(),
                    data.lidar_id) != unique_lidar_ids.end())
        continue;
      unique_lidar_ids.push_back(data.lidar_id);
    }
    if (unique_lidar_ids.size() != num_lidars_)
    {
      return false;
    }
    // 
    for (auto t : lidar_max_timestamps_)
    {
      if (t < traj_max)
      {
        return false;
      }
    }
    // 
    if (use_image_)
    {
      if (image_max_timestamp_ < traj_max)
      {
        return false;
      }
    }

    /// 2 
    for (auto it = lidar_buf_.begin(); it != lidar_buf_.end();)
    {
      if (it->timestamp >= traj_max)
      {
        ++it;
        continue;
      }
      bool add_entire_scan = AddToMsg(msgs, it, traj_max);
      if (add_entire_scan)
      {
        it = lidar_buf_.erase(it); // 
      }
      else
      {
        ++it; // 
      }
    }
    LOG(INFO) << "[msgs_scan_num] " << msgs.scan_num;

    /// 3 
    if (use_image_)
    {
      /// 
      int img_idx = INT_MAX;
      for (int i = 0; i < image_buf_.size(); i++)
      {
        if (image_buf_[i].timestamp >= traj_last_max &&
            image_buf_[i].timestamp < traj_max)
        {
          img_idx = i;
        }
        if (image_buf_[i].timestamp >= traj_max)
        {
          break;
        }
      }

      /// 
      // int img_idx = INT_MAX;
      // for (int i = 0; i < image_buf_.size(); i++)
      // {
      //   if (image_buf_[i].timestamp >= traj_last_max &&
      //       image_buf_[i].timestamp < traj_max)
      //   {
      //     img_idx = i;
      //     break;
      //   }
      // }

      if (img_idx != INT_MAX)
      {
        AddImageToMsg(msgs, image_buf_[img_idx], traj_max);
        // image_buf_.erase(image_buf_.begin() + img_idx);
      }
      else
      {
        msgs.if_have_image = false;
        // std::cout << "[GetMsgs does not get a image]\n";
        // std::getchar();
      }
    }

    return true;
  }

  void MsgManager::IMUMsgHandle(const sensor_msgs::Imu::ConstPtr &imu_msg)
  {
    int64_t t_last = cur_imu_timestamp_;
    // cur_imu_timestamp_ = imu_msg->header.stamp.toSec() - add_extra_timeoffset_s_;
    cur_imu_timestamp_ = imu_msg->header.stamp.toSec() * S_TO_NS;

    IMUData data;
    IMUMsgToIMUData(imu_msg, data);

    /// problem
    // data.timestamp -= add_extra_timeoffset_s_;

    // for trajectory_manager
    imu_buf_.emplace_back(data);
  }

  void MsgManager::VelodyneMsgHandle(
      const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id)
  {
    RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
    velodyne_feature_extraction_->ParsePointCloud(vlp16_msg, vlp_raw_cloud); // 

    // transform the input cloud to Lidar0 frame
    if (lidar_id != 0)
      pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
                               T_LktoL0_vec_[lidar_id]);

    // 
    velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

    lidar_buf_.emplace_back();
    lidar_buf_.back().lidar_id = lidar_id;
    if (lidar_timestamp_end_)
    {
      lidar_buf_.back().timestamp = (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS; // kaist、viral
    }
    else
    {
      lidar_buf_.back().timestamp = vlp16_msg->header.stamp.toSec() * S_TO_NS; // lvi、lio
    }
    lidar_buf_.back().raw_cloud = vlp_raw_cloud;
    lidar_buf_.back().surf_cloud =
        velodyne_feature_extraction_->GetSurfaceFeature();
    lidar_buf_.back().corner_cloud =
        velodyne_feature_extraction_->GetCornerFeature();
  }

  void MsgManager::VelodyneMsgHandleNoFeature(
      const sensor_msgs::PointCloud2::ConstPtr &vlp16_msg, int lidar_id)
  {
    RTPointCloud::Ptr vlp_raw_cloud(new RTPointCloud);
    velodyne_feature_extraction_->ParsePointCloudNoFeature(vlp16_msg, vlp_raw_cloud); // 

    // // transform the input cloud to Lidar0 frame
    // if (lidar_id != 0)
    //   pcl::transformPointCloud(*vlp_raw_cloud, *vlp_raw_cloud,
    //                            T_LktoL0_vec_[lidar_id]);

    // //
    // velodyne_feature_extraction_->LidarHandler(vlp_raw_cloud);

    lidar_buf_.emplace_back();
    lidar_buf_.back().lidar_id = lidar_id;
    if (lidar_timestamp_end_)
    {
      lidar_buf_.back().timestamp = (vlp16_msg->header.stamp.toSec() - 0.1003) * S_TO_NS; // kaist、viral
    }
    else
    {
      lidar_buf_.back().timestamp = vlp16_msg->header.stamp.toSec() * S_TO_NS; // lvi、lio
    }
    lidar_buf_.back().raw_cloud = vlp_raw_cloud;
    lidar_buf_.back().surf_cloud =
        velodyne_feature_extraction_->GetSurfaceFeature();
    lidar_buf_.back().corner_cloud =
        velodyne_feature_extraction_->GetCornerFeature();
  }

  void MsgManager::LivoxMsgHandle(
      const livox_ros_driver::CustomMsg::ConstPtr &livox_msg, int lidar_id)
  {
    RTPointCloud::Ptr livox_raw_cloud(new RTPointCloud);
    // 
    // livox_feature_extraction_->ParsePointCloud(livox_msg, livox_raw_cloud);
    // livox_feature_extraction_->ParsePointCloudNoFeature(livox_msg, livox_raw_cloud);
    livox_feature_extraction_->ParsePointCloudR3LIVE(livox_msg, livox_raw_cloud);

    LiDARCloudData data;
    data.lidar_id = lidar_id;
    data.timestamp = livox_msg->header.stamp.toSec() * S_TO_NS;
    data.raw_cloud = livox_raw_cloud;
    data.surf_cloud = livox_feature_extraction_->GetSurfaceFeature();
    data.corner_cloud = livox_feature_extraction_->GetCornerFeature();
    lidar_buf_.push_back(data);

    if (lidar_id != 0)
    {
      pcl::transformPointCloud(*data.raw_cloud, *data.raw_cloud,
                               T_LktoL0_vec_[lidar_id]);
      pcl::transformPointCloud(*data.surf_cloud, *data.surf_cloud,
                               T_LktoL0_vec_[lidar_id]);
      pcl::transformPointCloud(*data.corner_cloud, *data.corner_cloud,
                               T_LktoL0_vec_[lidar_id]);
    }
  }

  void MsgManager::ImageMsgHandle(const sensor_msgs::ImageConstPtr &msg)
  {
    if (pub_img_.getNumSubscribers() != 0)
    {
      pub_img_.publish(msg);
    }

    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cvImgPtr->image.empty())
    {
      std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET << std::endl;
      return;
    }

    image_buf_.emplace_back();
    image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
    image_buf_.back().image = cvImgPtr->image;
    nerf_time_.push_back(image_buf_.back().timestamp);

    if (image_buf_.back().image.cols == 640 || image_buf_.back().image.cols == 1280)
    {
      cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
    }

    // // for tiers
    // if (image_buf_.back().image.cols == 1920)
    // {
    //   cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(960, 540), 0, 0, cv::INTER_LINEAR);
    // }
  }

  void MsgManager::ImageMsgHandle(const sensor_msgs::CompressedImageConstPtr &msg)
  {
    if (pub_img_.getNumSubscribers() != 0)
    {
      cv_bridge::CvImagePtr cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      sensor_msgs::Image imgMsg = *(cvImgPtr->toImageMsg());
      imgMsg.header = msg->header; // 
      pub_img_.publish(msg);
    }

    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cvImgPtr->image.empty())
    {
      std::cout << RED << "[ImageMsgHandle get an empty img]" << RESET << std::endl;
      return;
    }

    image_buf_.emplace_back();
    image_buf_.back().timestamp = msg->header.stamp.toSec() * S_TO_NS;
    image_buf_.back().image = cvImgPtr->image;
    nerf_time_.push_back(image_buf_.back().timestamp);

    // std::cout << image_buf_.back().image.rows << " " << image_buf_.back().image.cols << std::endl;

    if (image_buf_.back().image.cols == 640 || image_buf_.back().image.cols == 1280)
    {
      cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(640, 512), 0, 0, cv::INTER_LINEAR);
    }

    // // for mars
    // if (image_buf_.back().image.cols == 2448)
    // {
    //   // cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(1224, 1024), 0, 0, cv::INTER_LINEAR);
    //   cv::resize(image_buf_.back().image, image_buf_.back().image, cv::Size(612, 512), 0, 0, cv::INTER_LINEAR);
    // }
  }

} // namespace cocolic
