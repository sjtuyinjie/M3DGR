#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <chrono>

#include "my_utility.h"
#include "mutexDeque.hpp"

using namespace std;

enum class SensorType
{
  VELODYNE,
  OUSTER,
  ROBOSENSE,
  LIVOX
};
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]

using PointXYZIRT = VelodynePointXYZIRT;

class FeatureExtract
{
public:
  ros::NodeHandle nh;
  // Topics
  string pointCloudTopic;
  // Frames
  string lidarFrame;

  // Lidar Sensor Configuration
  SensorType sensor;
  int downsampleRate;
  float lidarMinRange;
  float lidarMaxRange;
  int point_filter_num;

  //  Second lidar config
  bool useAuxiliaryLidar;
  std::string auxiliaryCloudTopic;
  SensorType auxiliarySensor;
  int auxiliaryDSRate;
  float auxiliaryLidarMinRange;
  float auxiliaryLidarMaxRange;
  int auxiliaryPointFilterRate;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
  pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
  pcl::PointCloud<rsPointXYZIRT>::Ptr tmpRSCloudIn;
  pcl::PointCloud<PointType>::Ptr inputCloud;
  pcl::PointCloud<PointType>::Ptr auxiliaryCloud;
  pcl::PointCloud<PointType>::Ptr sampleCloud;

  pcl::VoxelGrid<PointType> downSizeFilter;

  double timeScanCur;
  double timeScanEnd;

  // voxel filter paprams
  float odometrySurfLeafSize;

  ros::Subscriber subPrimaryLidar, subAuxiliaryLidar;

  ros::Publisher pubSurfacePoints;

  std_msgs::Header cloudHeader;

  MutexDeque<sensor_msgs::PointCloud2> cloudQueue, auxiliaryCloudQueue;
  Eigen::Vector3d T_Ax2Pri;
  Eigen::Matrix3d R_Ax2Pri;

  FeatureExtract()
  {
    nh.param<std::string>("common/pointCloudTopic", pointCloudTopic, "points_raw");
    nh.param<std::string>("feature_extract/lidarFrame", lidarFrame, "base_link");

    std::string sensorStr;
    nh.param<std::string>("feature_extract/sensor", sensorStr, "");
    if (sensorStr == "velodyne")
    {
      sensor = SensorType::VELODYNE;
    }
    else if (sensorStr == "livox")
    {
      sensor = SensorType::LIVOX;
      std::cout << "sensor type: livox-" << int(sensor) << std::endl;
    }
    else if (sensorStr == "ouster")
    {
      sensor = SensorType::OUSTER;
    }
    else if (sensorStr == "robosense")
    {
      sensor = SensorType::ROBOSENSE;
    }
    else
    {
      ROS_ERROR_STREAM(
          "Invalid sensor type (must be either 'velodyne' 'ouster' 'robosense' or 'livox'): " << sensorStr);
      ros::shutdown();
    }

    //  FIXME: add auxiliary lidar here
    nh.param<bool>("feature_extract/useAuxiliaryLidar", useAuxiliaryLidar, false);
    if (useAuxiliaryLidar)
    {
      nh.param<std::string>("feature_extract/auxiliaryCloudTopic", auxiliaryCloudTopic, "points_raw");
      std::string auxiliarySensorStr;
      nh.param<std::string>("feature_extract/auxiliarySensor", auxiliarySensorStr, "");
      if (auxiliarySensorStr == "velodyne")
      {
        auxiliarySensor = SensorType::VELODYNE;
      }
      else if (auxiliarySensorStr == "livox")
      {
        auxiliarySensor = SensorType::LIVOX;
        std::cout << "sensor type: livox-" << int(auxiliarySensor) << std::endl;
      }
      else if (auxiliarySensorStr == "ouster")
      {
        auxiliarySensor = SensorType::OUSTER;
      }
      else if (auxiliarySensorStr == "robosense")
      {
        auxiliarySensor = SensorType::ROBOSENSE;
      }
      else
      {
        ROS_ERROR_STREAM(
            "Invalid sensor type (must be either 'velodyne' 'ouster' 'robosense' or 'livox'): " << auxiliarySensorStr);
        ros::shutdown();
      }

      std::vector<double> extrinT(3, 0.0);
      std::vector<double> extrinR(9, 0.0);
      nh.param<vector<double>>("feature_extract/extrinsic_T_auxiliray_to_primary", extrinT, std::vector<double>());
      nh.param<vector<double>>("feature_extract/extrinsic_R_auxiliray_to_primary", extrinR, std::vector<double>());

      T_Ax2Pri << VEC_FROM_ARRAY(extrinT);
      R_Ax2Pri << MAT_FROM_ARRAY(extrinR);
    }

    nh.param<int>("feature_extract/downsampleRate", downsampleRate, 1);
    nh.param<int>("feature_extract/pointFilterRate", point_filter_num, 1);
    nh.param<float>("feature_extract/lidarMinRange", lidarMinRange, 1.0);
    nh.param<float>("feature_extract/lidarMaxRange", lidarMaxRange, 1000.0);

    nh.param<float>("feature_extract/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);

    subPrimaryLidar = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 50, &FeatureExtract::cloudHandler, this);
    if (useAuxiliaryLidar)
      subAuxiliaryLidar = nh.subscribe<sensor_msgs::PointCloud2>(auxiliaryCloudTopic, 50, &FeatureExtract::auxiliaryCloudHandler, this);

    pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 1);

    allocateMemory();
  }
  void allocateMemory()
  {
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
    tmpRSCloudIn.reset(new pcl::PointCloud<rsPointXYZIRT>());
    inputCloud.reset(new pcl::PointCloud<PointType>());
    sampleCloud.reset(new pcl::PointCloud<PointType>());
    auxiliaryCloud.reset(new pcl::PointCloud<PointType>());

    downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laser_in)
  {
    cloudQueue.push_back(*laser_in);
  }

  void auxiliaryCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laser_in)
  {
    auxiliaryCloudQueue.push_back(*laser_in);
  }

  void run()
  {
    ros::Rate rate(200);
    while (ros::ok())
    {
      rate.sleep();

      if (!cachePointCloud())
        continue;

      processPointCloud(cloudQueue, inputCloud, true);

      if (useAuxiliaryLidar && !auxiliaryCloudQueue.empty())
      {
        //  获取时间范围内的所有激光数据 [-0.11~0.11]

        processAuxiliaryCloud(auxiliaryCloudQueue, auxiliaryCloud);
        std::cout << auxiliaryCloud->size() << std::endl;
      }

      samplePointCloud();
      publishSampleCloud();
    }
  }

  bool cachePointCloud()
  {
    if (cloudQueue.size() < 2) //  至少２帧数据，才能处理
      return false;
    return true;
  }

  void processAuxiliaryCloud(MutexDeque<sensor_msgs::PointCloud2> &queue, pcl::PointCloud<PointType>::Ptr &cloudin)
  {
    pcl::PointCloud<PointXYZIRT>::Ptr tmpAuxiliaryCloud(new pcl::PointCloud<PointXYZIRT>());
    while (!queue.empty())
    {
      auto msg = queue.front();
      if (msg.header.stamp.toSec() < timeScanCur - 0.11)
        queue.pop_front();
      else
        break;
    }
    cloudin->clear();
    std::cout << "time cloud: " << std::setprecision(20) << timeScanCur << "-->" << timeScanEnd << std::endl;
    for (auto iter = queue.begin(); iter < queue.end(); iter++)
    {
      sensor_msgs::PointCloud2 currentCloudMsg = *iter;
      double timespan;
      double tmpTimeScanCur = currentCloudMsg.header.stamp.toSec();
      std::cout << "auxi time: " << setprecision(20) << tmpTimeScanCur << std::endl;

      if (tmpTimeScanCur < timeScanCur - 0.11)
      {
        continue;
      }
      if (tmpTimeScanCur > timeScanCur + 0.11)
        break;
      if (sensor == SensorType::VELODYNE)
      {
        pcl::moveFromROSMsg(currentCloudMsg, *tmpAuxiliaryCloud);

        timespan = tmpAuxiliaryCloud->points.back().time;

        if (tmpTimeScanCur + timespan < timeScanCur)
        {
          continue;
        }
        // std::cout << __FUNCTION__ << ", " << tmpAuxiliaryCloud->points[0].time << " ,"
        //           << tmpAuxiliaryCloud->points[200].time << ", " << tmpAuxiliaryCloud->points[500].time
        //           << ", " << tmpAuxiliaryCloud->points.back().time << std::endl;
        for (size_t i = 0; i < tmpAuxiliaryCloud->size(); i++)
        {
          auto &src = tmpAuxiliaryCloud->points[i];
          PointType dst;
          dst.x = src.x;
          dst.y = src.y;
          dst.z = src.z;
          dst.intensity = src.intensity;
          dst.normal_y = src.ring; //  ring
          dst.normal_z = timeScanEnd - timeScanCur;

          if (tmpTimeScanCur + src.time >= timeScanCur && tmpTimeScanCur + src.time < timeScanEnd)
          {
            dst.normal_x = (tmpTimeScanCur + src.time - timeScanCur) / (timeScanEnd - timeScanCur);
            Eigen::Vector3d pt(dst.x, dst.y, dst.z);
            pt = R_Ax2Pri * pt + T_Ax2Pri;
            dst.x = pt[0], dst.y = pt[1], dst.z = pt[2];
            cloudin->push_back(dst);
          }
        }
      }
      else
      {
        ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
        ros::shutdown();
      }
    }
  }

  // #define TEST_LIO_SAM_6AXIS_DATA
  bool processPointCloud(MutexDeque<sensor_msgs::PointCloud2> &queue, pcl::PointCloud<PointType>::Ptr cloudin, bool save_header)
  {
    sensor_msgs::PointCloud2 currentCloudMsg = queue.front();
    double timespan;

    // get timestamp
    double tmpTimeScanCur = currentCloudMsg.header.stamp.toSec();

    if (sensor == SensorType::VELODYNE)
    {
      pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
      cloudin->points.resize(laserCloudIn->size());
      cloudin->is_dense = laserCloudIn->is_dense;
#ifndef TEST_LIO_SAM_6AXIS_DATA
      timespan = laserCloudIn->points.back().time;
#else
      timespan = laserCloudIn->points.back().time - laserCloudIn->points[0].time;
#endif
      for (size_t i = 0; i < laserCloudIn->size(); i++)
      {
        auto &src = laserCloudIn->points[i];
        auto &dst = cloudin->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring; //  ring
        dst.normal_z = timespan;
#ifndef TEST_LIO_SAM_6AXIS_DATA
        dst.normal_x = src.time / timespan;
#else
        dst.normal_x = (src.time + timespan) / timespan;
        tmpTimeScanCur = tmpTimeScanCur + timespan; //  fix here
#endif
      }
#ifdef TEST_LIO_SAM_6AXIS_DATA
      timespan = 0.0;
#endif
    }
    else if (sensor == SensorType::LIVOX)
    {
      pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
      cloudin->points.resize(laserCloudIn->size());
      cloudin->is_dense = laserCloudIn->is_dense;
      timespan = laserCloudIn->points.back().time;
      for (size_t i = 0; i < laserCloudIn->size(); i++)
      {
        auto &src = laserCloudIn->points[i];
        auto &dst = cloudin->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring; //  ring
        dst.normal_z = timespan;
        dst.normal_x = src.time / timespan;
      }
      std::cout << "stamp: " << laserCloudIn->points[0].time << ", " << laserCloudIn->points.back().time << std::endl;
    }
    else if (sensor == SensorType::OUSTER)
    {
      // Convert to Velodyne format
      pcl::fromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
      // pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
      cloudin->points.resize(tmpOusterCloudIn->size());
      cloudin->is_dense = tmpOusterCloudIn->is_dense;
      //  FIXME:偶现,最后一个点时间戳异常
      // timespan = tmpOusterCloudIn->points.back().t;
      timespan = tmpOusterCloudIn->points[tmpOusterCloudIn->size() - 2].t;
      for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
      {
        auto &src = tmpOusterCloudIn->points[i];
        auto &dst = cloudin->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring;
        dst.normal_z = timespan * 1e-9f;
        dst.normal_x = src.t / timespan; //        *1e-9f;
      }
      timespan = timespan * 1e-9f;
      // std::cout << std::endl;
      // for (int i = tmpOusterCloudIn->size() - 6; i < tmpOusterCloudIn->size(); i++)
      //     std::cout << tmpOusterCloudIn->points[i].t * 1e-9f << " ,";
      // std::cout << std::endl;
    }
    else if (sensor == SensorType::ROBOSENSE)
    {
      //  FIXME: robosense时间戳为最后一个点的数据
      pcl::fromROSMsg(currentCloudMsg, *tmpRSCloudIn);
      // inputCloud->points.resize(tmpRSCloudIn->size());
      // inputCloud->is_dense = tmpRSCloudIn->is_dense;
      timespan = tmpRSCloudIn->points[tmpRSCloudIn->size() - 1].timestamp - tmpRSCloudIn->points[0].timestamp;
      std::cout << "fist: " << tmpRSCloudIn->points[1].timestamp
                << ", 100: " << tmpRSCloudIn->points[100].timestamp
                << ",intervel: " << tmpRSCloudIn->points[tmpRSCloudIn->size() - 1].timestamp - tmpRSCloudIn->points[0].timestamp
                << ",t: " << tmpRSCloudIn->points[0].timestamp - currentCloudMsg.header.stamp.toSec()
                << "timespan: " << timespan << std::endl;
      for (size_t i = 0; i < tmpRSCloudIn->size(); i++)
      {
        auto &src = tmpRSCloudIn->points[i];
        if (!pcl_isfinite(src.x) || !pcl_isfinite(src.y) || !pcl_isfinite(src.z))
          continue;

        PointType dst;
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.normal_y = src.ring;
        dst.normal_z = timespan;
        dst.normal_x = (src.timestamp - tmpRSCloudIn->points[0].timestamp) / timespan;
        cloudin->push_back(dst);
      }
      timespan = 0.0;
    }
    else
    {
      ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
      ros::shutdown();
    }
    if (save_header)
    {
      cloudHeader = currentCloudMsg.header;
      timeScanCur = tmpTimeScanCur;
      timeScanEnd = tmpTimeScanCur + timespan;
    }

    queue.pop_front();

    return true;
  }

  void samplePointCloud()
  {
    sampleCloud->clear();
    int cloudSize = inputCloud->points.size();
    sampleCloud->reserve(cloudSize);
    for (int i = 0; i < cloudSize; ++i)
    {
      if (i % point_filter_num != 0)
        continue;

      PointType thisPoint;
      thisPoint.x = inputCloud->points[i].x;
      thisPoint.y = inputCloud->points[i].y;
      thisPoint.z = inputCloud->points[i].z;
      thisPoint.intensity = inputCloud->points[i].intensity;
      thisPoint.normal_x = inputCloud->points[i].normal_x;
      thisPoint.normal_y = inputCloud->points[i].normal_y;
      thisPoint.normal_z = inputCloud->points[i].normal_z;

      float range = pointDistance(thisPoint);
      if (range < lidarMinRange || range > lidarMaxRange)
        continue;
      sampleCloud->push_back(thisPoint);
    }
    if (useAuxiliaryLidar && auxiliaryCloud->size() > 0)
    {
      for (int i = 0; i < auxiliaryCloud->size(); i++)
      {
        if (i % point_filter_num != 0)
          continue;

        PointType thisPoint;
        thisPoint.x = auxiliaryCloud->points[i].x;
        thisPoint.y = auxiliaryCloud->points[i].y;
        thisPoint.z = auxiliaryCloud->points[i].z;
        thisPoint.intensity = auxiliaryCloud->points[i].intensity;
        thisPoint.normal_x = auxiliaryCloud->points[i].normal_x;
        thisPoint.normal_y = auxiliaryCloud->points[i].normal_y;
        thisPoint.normal_z = auxiliaryCloud->points[i].normal_z;

        float range = pointDistance(thisPoint);
        if (range < lidarMinRange || range > lidarMaxRange)
          continue;
        sampleCloud->push_back(thisPoint);
      }
      auxiliaryCloud->clear();
    }
  }

  void publishSampleCloud()
  {
    publishCloud(&pubSurfacePoints, sampleCloud, cloudHeader.stamp, lidarFrame);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ESKF_LIO");

  FeatureExtract FE;

  ROS_INFO("\033[1;32m----> Feature Extraction multi Started.\033[0m");

  std::thread processthread(&FeatureExtract::run, &FE);
  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();
  processthread.join();

  return 0;
}