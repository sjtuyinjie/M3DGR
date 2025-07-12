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

#include <eigen_conversions/eigen_msg.h>
#include <odom/odometry_manager.h>
#include <numeric>

#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>

std::fstream rgb_file;
std::fstream img_file;

namespace cocolic
{

  OdometryManager::OdometryManager(const YAML::Node &node, ros::NodeHandle &nh)
      : odometry_mode_(LIO), is_initialized_(false)
  {
    std::string config_path = node["config_path"].as<std::string>();

    std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
    YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);

    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);

    std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
    YAML::Node cam_node = YAML::LoadFile(cam_yaml);

    odometry_mode_ = OdometryMode(node["odometry_mode"].as<int>());
    std::cout << "\n🥥 Odometry Mode: ";
    if (odometry_mode_ == LICO)
    {
      std::cout << "LiDAR-Inertial-Camera Odometry 🥥" << std::endl;
    }
    else if (odometry_mode_ == LIO)
    {
      std::cout << "LiDAR-Inertial Odometry 🥥" << std::endl;
    }

    // extrinsic: sensor to imu
    ExtrinsicParam EP_LtoI, EP_CtoI, EP_ItoI, EP_MtoI;
    EP_LtoI.Init(lidar_node["lidar0"]["Extrinsics"]);
    if (odometry_mode_ == LICO)
      EP_CtoI.Init(cam_node["CameraExtrinsics"]);
    if (node["IMUExtrinsics"])
      EP_ItoI.Init(imu_node["IMUExtrinsics"]);
    EP_MtoI.Init(imu_node["MarkerExtrinsics"]);

    trajectory_ = std::make_shared<Trajectory>(-1, 0);
    trajectory_->SetSensorExtrinsics(SensorType::LiDARSensor, EP_LtoI);
    trajectory_->SetSensorExtrinsics(SensorType::CameraSensor, EP_CtoI);
    trajectory_->SetSensorExtrinsics(SensorType::IMUSensor, EP_ItoI);
    trajectory_->SetSensorExtrinsics(SensorType::Marker, EP_MtoI);

    // non-uniform b-spline
    t_add_ = node["t_add"].as<double>();
    t_add_ns_ = t_add_ * S_TO_NS;
    non_uniform_ = node["non_uniform"].as<bool>();
    distance0_ = node["distance0"].as<double>();

    // lidar
    lidar_iter_ = node["lidar_iter"].as<int>();
    use_lidar_scale_ = node["use_lidar_scale"].as<bool>();
    lidar_handler_ = std::make_shared<LidarHandler>(lidar_node, trajectory_);
    std::cout << "\n🍺 The number of multiple LiDARs is " << lidar_node["num_lidars"].as<int>() << "." << std::endl;

    // imu
    imu_initializer_ = std::make_shared<IMUInitializer>(imu_node);
    gravity_norm_ = imu_initializer_->GetGravity().norm();

    // camera
    camera_handler_ = std::make_shared<R3LIVE>(cam_node, EP_CtoI);
    t_begin_add_cam_ = node["t_begin_add_cam"].as<double>() * S_TO_NS;
    v_points_.clear();
    px_obss_.clear();
    double fx = cam_node["cam_fx"].as<double>();
    double fy = cam_node["cam_fy"].as<double>();
    double cx = cam_node["cam_cx"].as<double>();
    double cy = cam_node["cam_cy"].as<double>();
    K_ << fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0;

    // trajectory parameterized by b-spline
    trajectory_manager_ = std::make_shared<TrajectoryManager>(node, trajectory_);
    trajectory_manager_->use_lidar_scale = use_lidar_scale_;
    trajectory_manager_->SetIntrinsic(K_);

    int division_coarse = node["division_coarse"].as<int>();
    cp_add_num_coarse_ = division_coarse;
    trajectory_manager_->SetDivisionParam(division_coarse, -1);

    odom_viewer_.SetPublisher(nh);

    msg_manager_ = std::make_shared<MsgManager>(node, nh);  // load rosbag

    bool verbose;
    nh.param<double>("pasue_time", pasue_time_, -1);
    nh.param<bool>("verbose", verbose, false);
    trajectory_manager_->verbose = verbose;

    // evaluation
    is_evo_viral_ = node["is_evo_viral"].as<bool>();
    CreateCacheFolder(config_path, msg_manager_->bag_path_);

    std::cout << std::fixed << std::setprecision(4);
    LOG(INFO) << std::fixed << std::setprecision(4);
  }

  bool OdometryManager::CreateCacheFolder(const std::string &config_path,
                                          const std::string &bag_path)
  {
    boost::filesystem::path path_cfg(config_path);
    boost::filesystem::path path_bag(bag_path);
    if (path_bag.extension() != ".bag")
    {
      return false;
    }
    std::string bag_name_ = path_bag.stem().string();

    std::string cache_path_parent_ = path_cfg.parent_path().string();
    cache_path_ = cache_path_parent_ + "/data/" + bag_name_;
    // boost::filesystem::create_directory(cache_path_);
    return true;
  }

  void OdometryManager::RunBag()
  {
    while (ros::ok())
    {
      /// [1] process a newly arrived frame of data: lidar or imu or camera
      msg_manager_->SpinBagOnce();
      if (!msg_manager_->has_valid_msg_)
      {
        break;
      }

      /// [2] static initialization, do not move at the begging!
      if (!is_initialized_)
      {
        while (!msg_manager_->imu_buf_.empty())
        {
          imu_initializer_->FeedIMUData(msg_manager_->imu_buf_.front());
          msg_manager_->imu_buf_.pop_front();
        }

        if (imu_initializer_->StaticInitialIMUState())
        {
          SetInitialState();
          std::cout << "\n🍺 Static initialization succeeds.\n";
          std::cout << "\n🍺 Trajectory start time: " << trajectory_->GetDataStartTime() << " ns.\n";
        }
        else
        {
          continue;
        }
      }

      /// [3] prepare data for the latest time interval delta_t
      static bool is_two_seg_prepared = false;
      static int seg_msg_cnt = 0;
      if (!is_two_seg_prepared)
      {
        if (PrepareTwoSegMsgs(seg_msg_cnt))  // prepare interval0 and interval1
        {
          seg_msg_cnt++;
        }
        if (seg_msg_cnt == 2)  // if interval0 and interval1 are ready
        {
          is_two_seg_prepared = true;
          UpdateTwoSeg();
          trajectory_->InitBlendMat();  // blending matrix is computed by knots of b-spline
        }
        else
        {
          continue;
        }
      }

      /// [4] update trajectory segment in the latest time interval delta_t
      if (PrepareMsgs())
      {
        // decide control point placement in the time interval delta_t by imu 
        UpdateOneSeg();  
        int offset = cp_add_num_cur + cp_add_num_next + cp_add_num_next_next;
        for (int i = 0; i < cp_add_num_cur; i++)
        {
          trajectory_->AddBlendMat(offset - i);  // blending matrix is computed by knots of b-spline
        }
        trajectory_manager_->SetDivision(cp_add_num_cur);
        trajectory_->startIdx = trajectory_->knts.size() - 1 - offset - 2;  // 2 serves as a margin or tolerance
        if (trajectory_->startIdx < 0)
        {
          trajectory_->startIdx = 0;
        }

        // fusing lidar-imu-camera to update the trajectory
        SolveLICO();

        // deep copy
        msg_manager_->cur_msgs = NextMsgs();
        msg_manager_->cur_msgs = msg_manager_->next_msgs;
        msg_manager_->cur_msgs.image = msg_manager_->next_msgs.image.clone();
        msg_manager_->next_msgs = NextMsgs();
        msg_manager_->next_msgs = msg_manager_->next_next_msgs;
        msg_manager_->next_msgs.image = msg_manager_->next_next_msgs.image.clone();
        msg_manager_->next_next_msgs = NextMsgs();

        traj_max_time_ns_cur = traj_max_time_ns_next;
        traj_max_time_ns_next = traj_max_time_ns_next_next;
        cp_add_num_cur = cp_add_num_next;
        cp_add_num_next = cp_add_num_next_next;

        while (msg_manager_->image_buf_.size() > 10)
        {
          msg_manager_->image_buf_.pop_front();
        }
      }
    }
  }

  void OdometryManager::SolveLICO()
  {
    msg_manager_->LogInfo();
    if (msg_manager_->cur_msgs.lidar_timestamp < 0)
    {
      LOG(INFO) << "CANT SolveLICO!";
    }

    // lic optimization
    ProcessLICData();

    // prior update
    trajectory_manager_->UpdateLICPrior(
        lidar_handler_->GetPointCorrespondence());

    // remove old imu data
    auto &msg = msg_manager_->cur_msgs;
    trajectory_manager_->UpdateLiDARAttribute(msg.lidar_timestamp,
                                              msg.lidar_max_timestamp);
  }

  void OdometryManager::ProcessLICData()
  {
    auto &msg = msg_manager_->cur_msgs;  // fake points with timestamp -1 exist up to now
    msg.CheckData();

    bool process_image = msg.if_have_image && msg.image_timestamp > t_begin_add_cam_;
    if (process_image)
    {
      LOG(INFO) << "Process " << msg.scan_num << " scans in ["
                << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]"
                << "; image_time: " << msg.image_timestamp * NS_TO_S;
    }
    else
    {
      LOG(INFO) << "Process " << msg.scan_num << " scans in ["
                << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]";
    }

    /// [1] transform the format of lidar pointcloud -> feature_cur_、feature_cur_ds_
    lidar_handler_->FeatureCloudHandler(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                     msg.lidar_corner_cloud, msg.lidar_surf_cloud, msg.lidar_raw_cloud);  // fake points are removed

    /// [2] coarsely optimize trajectory based on prior、imu（served as good initial values）
    trajectory_manager_->PredictTrajectory(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                           traj_max_time_ns_cur, cp_add_num_cur, non_uniform_);

    /// [3] update lidar local map
    int active_idx = trajectory_->numKnots() - 1 - cp_add_num_cur - 2;
    trajectory_->SetActiveTime(trajectory_->knts[active_idx]);
    lidar_handler_->UpdateLidarSubMap();

    /// [4] update visual local map（tracking map points for the current image frame）
    // after upate, m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos
    v_points_.clear();
    px_obss_.clear();
    if (process_image)
    {
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);
      camera_handler_->UpdateVisualSubMap(msg.image, msg.image_timestamp * NS_TO_S, Twc.unit_quaternion(), Twc.translation());
      // v_points_.clear();
      // px_obss_.clear();
      auto &map_rgb_pts_in_last_frame_pos = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
      for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
      {
        RGB_pts *rgb_pt = ((RGB_pts *)it->first);
        v_points_.push_back(Eigen::Vector3d(rgb_pt->get_pos()(0, 0), rgb_pt->get_pos()(1, 0), rgb_pt->get_pos()(2, 0)));
        px_obss_.push_back(Eigen::Vector2d(it->second.x, it->second.y));
      }

      if (odom_viewer_.pub_track_img_.getNumSubscribers() != 0 || odom_viewer_.pub_sub_visual_map_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        VPointCloud visual_sub_map_debug;  // optical flow + ransac *2 -> 3d association（red）
        visual_sub_map_debug.clear();

        for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
        {
          RGB_pts *rgb_pt = ((RGB_pts *)it->first);
          cv::circle(img_debug, it->second, 2, cv::Scalar(0, 255, 0), -1, 8);  // optical flow + ransac *2 -> 2d association（green）
          VPoint temp_map;
          temp_map.x = rgb_pt->get_pos()(0, 0);
          temp_map.y = rgb_pt->get_pos()(1, 0);
          temp_map.z = rgb_pt->get_pos()(2, 0);
          temp_map.intensity = 0.;
          visual_sub_map_debug.push_back(temp_map);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishTrackImg(out_msg.toImageMsg());
        odom_viewer_.PublishSubVisualMap(visual_sub_map_debug);
      }
    }

    /// [5] finely optimize trajectory based on prior、lidar、imu、camera
    for (int iter = 0; iter < lidar_iter_; ++iter)
    {
      lidar_handler_->GetLoamFeatureAssociation();

      if (process_image)
      {
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), v_points_, px_obss_, 8);
      }
      else
      {
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), {}, {}, 8);
        trajectory_manager_->SetProcessCurImg(false);
      }
    }
    PublishCloudAndTrajectory();

    /// [6] update visual global map
    PosCloud::Ptr cloud_undistort = PosCloud::Ptr(new PosCloud);
    auto latest_feature_before_active_time = lidar_handler_->GetFeatureCurrent();
    PosCloud::Ptr cloud_distort = latest_feature_before_active_time.surface_features;
    if (cloud_distort->size() != 0)
    {
      trajectory_->UndistortScanInG(*cloud_distort, latest_feature_before_active_time.timestamp, *cloud_undistort);
      camera_handler_->UpdateVisualGlobalMap(cloud_undistort, latest_feature_before_active_time.time_max * NS_TO_S);
    }

    /// [7] associate new map points for the current image frame
    if (process_image)
    {
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);
      camera_handler_->AssociateNewPointsToCurrentImg(Twc.unit_quaternion(), Twc.translation());

      if (odom_viewer_.pub_undistort_scan_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        {
          for (int i = 0; i < cloud_undistort->points.size(); i++)
          {
            auto pt = cloud_undistort->points[i];
            Eigen::Vector3d pt_e(pt.x, pt.y, pt.z);
            Eigen::Matrix3d Rwc = Twc.unit_quaternion().toRotationMatrix();
            Eigen::Vector3d twc = Twc.translation();
            Eigen::Vector3d pt_cam = Rwc.transpose() * pt_e - Rwc.transpose() * twc;
            double X = pt_cam.x(), Y = pt_cam.y(), Z = pt_cam.z();
            cv::Point2f pix(K_(0, 0) * X / Z + K_(0, 2), K_(1, 1) * Y / Z + K_(1, 2));
            cv::circle(img_debug, pix, 2, cv::Scalar(0, 0, 255), -1, 8);
          }
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ros::Time::now();
          out_msg.encoding = sensor_msgs::image_encodings::BGR8;
          out_msg.image = img_debug;
          odom_viewer_.PublishUndistortScanInCurImg(out_msg.toImageMsg());
        }
      }

      if (odom_viewer_.pub_old_and_new_added_points_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        auto obss = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
        for (auto it = obss.begin(); it != obss.end(); it++)
        {
          cv::Point2f pix = it->second;
          cv::circle(img_debug, pix, 2, cv::Scalar(0, 255, 0), -1, 8);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishOldAndNewAddedPointsInCurImg(out_msg.toImageMsg());
      }
    }

    /// [8] visualize tf in rviz
    auto pose = trajectory_->GetLidarPoseNURBS(msg.lidar_timestamp);
    auto pose_debug = trajectory_->GetCameraPoseNURBS(msg.lidar_timestamp);
    odom_viewer_.PublishTF(pose.unit_quaternion(), pose.translation(), "lidar",
                           "map");
    odom_viewer_.PublishTF(pose_debug.unit_quaternion(), pose_debug.translation(), "camera",
                           "map");
    odom_viewer_.PublishTF(trajectory_manager_->GetGlobalFrame(),
                           Eigen::Vector3d::Zero(), "map", "global");
  }

  bool OdometryManager::PrepareTwoSegMsgs(int seg_idx)
  {
    if (!is_initialized_)
      return false;

    int64_t data_start_time = trajectory_->GetDataStartTime();
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);                             // 
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp; // 
      }
    }
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);
        msg_manager_->image_max_timestamp_ = data.timestamp;
      }
    }
    msg_manager_->RemoveBeginData(data_start_time, 0);

    // 
    // 
    int64_t traj_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_ * (seg_idx + 1);

    // 
    // 
    bool have_msg = false;
    if (seg_idx == 0)
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS();
      have_msg = msg_manager_->GetMsgs(msg_manager_->cur_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }
    if (seg_idx == 1)
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_;
      have_msg = msg_manager_->GetMsgs(msg_manager_->next_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }

    // 
    if (have_msg)
    {
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());
        msg_manager_->imu_buf_.pop_front();
      }
      if (seg_idx == 0)
      {
        traj_max_time_ns_cur = traj_max_time_ns;
      }
      if (seg_idx == 1)
      {
        traj_max_time_ns_next = traj_max_time_ns;
      }
      return true;
    }
    else
    {
      return false;
    }
  }

  void OdometryManager::UpdateTwoSeg()
  {
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// update the first seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < trajectory_->maxTimeNsNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < trajectory_->maxTimeNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      LOG(INFO) << "[aver_r_first] " << aver_r.norm() << " | [aver_a_first] " << aver_a.norm();
      LOG(INFO) << "[var_r_first] " << var_r << " | [var_a_first] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      LOG(INFO) << "[cp_add_num_first] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_cur - trajectory_->maxTimeNsNURBS()) / cp_add_num;
      LOG(INFO) << "[extend_step_first] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = trajectory_->maxTimeNsNURBS() + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_cur);

      cp_add_num_cur = cp_add_num;
    }

    /// update the second seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      LOG(INFO) << "[aver_r_second] " << aver_r.norm() << " | [aver_a_second] " << aver_a.norm();
      LOG(INFO) << "[var_r_second] " << var_r << " | [var_a_second] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      LOG(INFO) << "[cp_add_num_second] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_next - traj_max_time_ns_cur) / cp_add_num;
      LOG(INFO) << "[extend_step_second] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_cur + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_next);

      cp_add_num_next = cp_add_num;
    }
  }

  bool OdometryManager::PrepareMsgs()
  {
    if (!is_initialized_)
      return false;

    int64_t data_start_time = trajectory_->GetDataStartTime();
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);                             // 
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp; // 
      }
    }
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);
        msg_manager_->image_max_timestamp_ = data.timestamp;
      }
    }
    msg_manager_->RemoveBeginData(data_start_time, 0);

    int64_t traj_max_time_ns = traj_max_time_ns_next + t_add_ns_;

    int64_t traj_last_max_time_ns = traj_max_time_ns_next;
    bool have_msg = msg_manager_->GetMsgs(msg_manager_->next_next_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);

    if (have_msg)
    {
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());
        msg_manager_->imu_buf_.pop_front();
      }
      traj_max_time_ns_next_next = traj_max_time_ns;
      return true;
    }
    else
    {
      return false;
    }
  }

  void OdometryManager::UpdateOneSeg()
  {
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// update the first seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      LOG(INFO) << "[aver_r_new] " << aver_r.norm() << " | [aver_a_new] " << aver_a.norm();
      LOG(INFO) << "[var_r_new] " << var_r << " | [var_a_new] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      LOG(INFO) << "[cp_add_num_new] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_next_next - traj_max_time_ns_next) / cp_add_num;
      LOG(INFO) << "[extend_step_new] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_next + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_next_next);

      cp_add_num_next_next = cp_add_num;
    }
  }

  void OdometryManager::SetInitialState()
  {
    if (is_initialized_)
    {
      assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
      std::cout << RED << "[Error] system state has been initialized" << RESET << std::endl;
      return;
    }

    is_initialized_ = true;

    if (imu_initializer_->InitialDone())
    {
      SystemState sys_state = imu_initializer_->GetIMUState(); // I0toG
      trajectory_manager_->SetSystemState(sys_state, distance0_);

      trajectory_manager_->AddIMUData(imu_initializer_->GetIMUData().back());
      msg_manager_->imu_buf_.clear();
    }
    assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
  }

  void OdometryManager::PublishCloudAndTrajectory()
  {
    odom_viewer_.PublishDenseCloud(trajectory_, lidar_handler_->GetFeatureMapDs(),
                                   lidar_handler_->GetFeatureCurrent());

    odom_viewer_.PublishSplineTrajectory(
        trajectory_, 0.0, trajectory_->maxTimeNURBS(), 0.1);
  }

  double OdometryManager::SaveOdometry()
  {
    std::string descri;
    if (odometry_mode_ == LICO)
      descri = "LICO";
    else if (odometry_mode_ == LIO)
      descri = "LIO";

    if (msg_manager_->NumLiDAR() > 1)
      descri = descri + "2";

    ros::Time timer;
    std::string time_full_str = std::to_string(timer.now().toNSec());
    std::string t_str = "_" + time_full_str.substr(time_full_str.size() - 4);

    int idx = -1;
    int64_t true_maxtime = trajectory_->maxTimeNsNURBS();
    for (int i = trajectory_->knts.size() - 1; i >= 0; i--)
    {
      if (true_maxtime == trajectory_->knts[i])
      {
        idx = i;
        break;
      }
    }
    idx -= 1;
    int64_t maxtime = trajectory_->knts[idx];
    maxtime = trajectory_->maxTimeNsNURBS() - 0.1 * S_TO_NS;

    trajectory_->ToTUMTxt(cache_path_ + "_" + descri + ".txt", maxtime, is_evo_viral_,
                          0.01);  // 100Hz pose querying

    // int sum_cp = std::accumulate(cp_num_vec.begin(), cp_num_vec.end(), 0);
    // std::cout << GREEN << "ave_cp_num " << sum_cp * 1.0 / cp_num_vec.size() << RESET << std::endl;

    return trajectory_->maxTimeNURBS();
  }

} // namespace cocolic
