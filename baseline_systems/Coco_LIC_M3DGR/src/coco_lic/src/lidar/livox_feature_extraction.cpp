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

#include "livox_feature_extraction.h"
#include <cocolic/feature_cloud.h>
#include <pcl_conversions/pcl_conversions.h> // pcl::fromROSMsg

using namespace std;

namespace cocolic
{

  LivoxFeatureExtraction::LivoxFeatureExtraction(const YAML::Node &node)
      : vx(0), vy(0), vz(0)
  {
    auto const &livox_node = node["Livox"];
    n_scan = livox_node["n_scan"].as<int>();
    blind = livox_node["blind"].as<double>();
    inf_bound = livox_node["inf_bound"].as<double>();
    group_size = livox_node["group_size"].as<int>();
    disA = livox_node["disA"].as<double>();
    disB = livox_node["disB"].as<double>();
    limit_maxmid = livox_node["limit_maxmid"].as<double>();
    limit_midmin = livox_node["limit_midmin"].as<double>();
    limit_maxmin = livox_node["limit_maxmin"].as<double>();
    p2l_ratio = livox_node["p2l_ratio"].as<double>();
    jump_up_limit = livox_node["jump_up_limit"].as<double>();
    jump_up_limit = cos(jump_up_limit / 180 * M_PI);
    jump_down_limit = livox_node["jump_down_limit"].as<double>();
    jump_down_limit = cos(jump_down_limit / 180 * M_PI);
    edgea = livox_node["edgea"].as<double>();
    edgeb = livox_node["edgeb"].as<double>();
    smallp_intersect = livox_node["smallp_intersect"].as<double>();
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);
    smallp_ratio = livox_node["smallp_ratio"].as<double>();
    point_filter_num = livox_node["point_filter_num"].as<int>();

    pub_corner_cloud = nh.advertise<sensor_msgs::PointCloud2>("corner_cloud", 10);
    pub_surface_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("surface_cloud", 10);
    pub_full_cloud = nh.advertise<sensor_msgs::PointCloud2>("full_cloud", 10);
    pub_feature_cloud = nh.advertise<cocolic::feature_cloud>("feature_cloud", 10);

    AllocateMemory();
    // clearState();
    //  ResetParameters();
  }

  bool LivoxFeatureExtraction::ParsePointCloud(
      const livox_ros_driver::CustomMsg::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud)
  {
    clearState();

    uint plsize = lidar_msg->point_num;
    p_corner_cloud->reserve(plsize);
    p_surface_cloud->reserve(plsize);
    p_full_cloud->resize(plsize);

    std::vector<RTPointCloud::Ptr> in_cloud_vec;
    in_cloud_vec.resize(n_scan);
    for (int i = 0; i < n_scan; i++)
    {
      in_cloud_vec[i] = RTPointCloud::Ptr(new RTPointCloud());
      in_cloud_vec[i]->reserve(plsize);
    }

    for (uint i = 1; i < plsize; i++)
    {
      if ((lidar_msg->points[i].line < n_scan) &&
          ((lidar_msg->points[i].tag & 0x30) == 0x10) &&
          (!IS_VALID(lidar_msg->points[i].x)) &&
          (!IS_VALID(lidar_msg->points[i].y)) &&
          (!IS_VALID(lidar_msg->points[i].z)))
      {
        (*p_full_cloud)[i].x = lidar_msg->points[i].x;
        (*p_full_cloud)[i].y = lidar_msg->points[i].y;
        (*p_full_cloud)[i].z = lidar_msg->points[i].z;
        (*p_full_cloud)[i].intensity = lidar_msg->points[i].reflectivity;
        // (*p_full_cloud)[i].time = lidar_msg->points[i].offset_time * 1e-9;
        (*p_full_cloud)[i].time = int64_t(lidar_msg->points[i].offset_time);

        if ((std::abs((*p_full_cloud)[i].x - (*p_full_cloud)[i - 1].x) > 1e-7) ||
            (std::abs((*p_full_cloud)[i].y - (*p_full_cloud)[i - 1].y) > 1e-7) ||
            (std::abs((*p_full_cloud)[i].z - (*p_full_cloud)[i - 1].z) > 1e-7))
        {
          in_cloud_vec[lidar_msg->points[i].line]->push_back((*p_full_cloud)[i]);
        }
      }
    }

    if (in_cloud_vec[0]->size() <= 7)
    {
      LOG(WARNING) << "[ParsePointCloud] input cloud size too small "
                   << in_cloud_vec[0]->size();
      return false;
    }

    std::vector<std::vector<orgtype>> typess(n_scan);
    for (int j = 0; j < n_scan; j++)
    {
      RTPointCloud &pl = (*in_cloud_vec[j]);
      vector<orgtype> &types = typess[j];
      plsize = pl.size();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
        // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
      }
      // plsize++;
      types[plsize].range =
          sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);

      giveFeature(pl, types, *p_corner_cloud, *p_surface_cloud);
    }

    for (auto const &v : in_cloud_vec)
    {
      *out_cloud += (*v);
    }

    PublishCloud("map");
    return true;
  }

  bool LivoxFeatureExtraction::ParsePointCloudR3LIVE(
      const livox_ros_driver::CustomMsg::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud)
  {
    clearState();

    std::vector<RTPointCloud::Ptr> in_cloud_vec;
    in_cloud_vec.resize(n_scan);
    std::vector<std::vector<orgtype>> typess(n_scan);

    uint plsize = lidar_msg->point_num;

    p_corner_cloud->reserve(plsize);
    p_surface_cloud->reserve(plsize);
    p_full_cloud->resize(plsize);

    for (int i = 0; i < n_scan; i++)
    {
      in_cloud_vec[i] = RTPointCloud::Ptr(new RTPointCloud());
      in_cloud_vec[i]->reserve(plsize);
    }
    // ANCHOR - remove nearing pts.
    for (uint i = 1; i < plsize; i++)
    {
      if ((lidar_msg->points[i].line < n_scan) && (!IS_VALID(lidar_msg->points[i].x)) && (!IS_VALID(lidar_msg->points[i].y)) && (!IS_VALID(lidar_msg->points[i].z)) && lidar_msg->points[i].x > 0.7)
      {
        // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
        // See [3.4 Tag Information]
        if ((lidar_msg->points[i].x > 2.0) && (((lidar_msg->points[i].tag & 0x03) != 0x00) || ((lidar_msg->points[i].tag & 0x0C) != 0x00)))
        {
          // Remove the bad quality points
          continue;
        }
        // clang-format on
        (*p_full_cloud)[i].x = lidar_msg->points[i].x;
        (*p_full_cloud)[i].y = lidar_msg->points[i].y;
        (*p_full_cloud)[i].z = lidar_msg->points[i].z;
        (*p_full_cloud)[i].intensity = lidar_msg->points[i].reflectivity;
        (*p_full_cloud)[i].time = int64_t(lidar_msg->points[i].offset_time);

        if ((std::abs((*p_full_cloud)[i].x - (*p_full_cloud)[i - 1].x) > 1e-7) || (std::abs((*p_full_cloud)[i].y - (*p_full_cloud)[i - 1].y) > 1e-7) ||
            (std::abs((*p_full_cloud)[i].z - (*p_full_cloud)[i - 1].z) > 1e-7))
        {
          in_cloud_vec[lidar_msg->points[i].line]->push_back((*p_full_cloud)[i]);
        }
      }
    }
    if (in_cloud_vec.size() != n_scan)
    {
      return false;
    }
    if (in_cloud_vec[0]->size() <= 7)
    {
      LOG(WARNING) << "[ParsePointCloud] input cloud size too small "
                   << in_cloud_vec[0]->size();
      return false;
    }

    for (int j = 0; j < n_scan; j++)
    {
      RTPointCloud &pl = *(in_cloud_vec[j]);
      std::vector<orgtype> &types = typess[j];
      plsize = pl.size();
      if (plsize < 7)
      {
        continue;
      }
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
      }
      // plsize++;
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
      giveFeatureR3LIVE(pl, types, *p_corner_cloud, *p_surface_cloud);
    }

    /// 
    p_corner_cloud->push_back((*p_full_cloud)[0]);
    for (auto const &v : in_cloud_vec)
    {
      *out_cloud += (*v);
    }

    // PublishCloud("map");
    return true;
  }

  bool LivoxFeatureExtraction::ParsePointCloudNoFeature(
      const livox_ros_driver::CustomMsg::ConstPtr &lidar_msg,
      RTPointCloud::Ptr out_cloud)
  {
    clearState();

    uint plsize = lidar_msg->point_num;
    p_corner_cloud->reserve(plsize);
    p_surface_cloud->reserve(plsize);
    p_full_cloud->resize(plsize);

    uint valid_num = 0;
    for (uint i = 1; i < plsize; i++)
    {
      if ((lidar_msg->points[i].line < n_scan) && ((lidar_msg->points[i].tag & 0x30) == 0x10 || (lidar_msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++;
        if (valid_num % point_filter_num == 0)
        {
          (*p_full_cloud)[i].x = lidar_msg->points[i].x;
          (*p_full_cloud)[i].y = lidar_msg->points[i].y;
          (*p_full_cloud)[i].z = lidar_msg->points[i].z;
          (*p_full_cloud)[i].intensity = lidar_msg->points[i].reflectivity;
          (*p_full_cloud)[i].time = int64_t(lidar_msg->points[i].offset_time);

          if (((abs((*p_full_cloud)[i].x - (*p_full_cloud)[i - 1].x) > 1e-7) || (abs((*p_full_cloud)[i].y - (*p_full_cloud)[i - 1].y) > 1e-7) || (abs((*p_full_cloud)[i].z - (*p_full_cloud)[i - 1].z) > 1e-7)) && ((*p_full_cloud)[i].x * (*p_full_cloud)[i].x + (*p_full_cloud)[i].y * (*p_full_cloud)[i].y + (*p_full_cloud)[i].z * (*p_full_cloud)[i].z > (blind * blind)))
          {
            p_surface_cloud->push_back((*p_full_cloud)[i]);
          }
        }
      }
    }

    p_corner_cloud->push_back((*p_full_cloud)[0]);

    *out_cloud = *p_full_cloud;

    PublishCloud("map");
    return true;
  }

// test
#if 0
void LivoxFeatureExtraction::LivoxHandler(
    const livox_ros_driver::CustomMsg::ConstPtr& lidar_msg) {
  clearState();

  std::vector<RTPointCloud::Ptr> out_cloud;
  for (int i = 0; i < GetScanNumber(); i++) {
    boost::shared_ptr<RTPointCloud> ptr(new RTPointCloud);
    out_cloud.push_back(ptr);
  }

  std::vector<std::vector<orgtype>> typess(n_scan);
  int cloud_size = lidar_msg->point_num;
  p_full_cloud->resize(cloud_size);
  p_corner_cloud->reserve(cloud_size);
  p_surface_cloud->reserve(cloud_size);

  for (int i = 0; i < n_scan; i++) {
    out_cloud[i]->reserve(cloud_size);
  }

  for (int i = 1; i < cloud_size; i++) {
    if ((lidar_msg->points[i].line < n_scan) &&
        ((lidar_msg->points[i].tag & 0x30) == 0x10) &&
        (!IS_VALID(lidar_msg->points[i].x)) &&
        (!IS_VALID(lidar_msg->points[i].y)) &&
        (!IS_VALID(lidar_msg->points[i].z))) {
      p_full_cloud->points[i].x = lidar_msg->points[i].x;
      p_full_cloud->points[i].y = lidar_msg->points[i].y;
      p_full_cloud->points[i].z = lidar_msg->points[i].z;
      p_full_cloud->points[i].intensity = lidar_msg->points[i].reflectivity;
      p_full_cloud->points[i].time =
          lidar_msg->points[i].offset_time / float(1000000000);
      //      p_full_cloud->points[i].time =
      //          (lidar_msg->timebase + lidar_msg->points[i].offset_time) /
      //          float(1000000);

      if ((std::abs(p_full_cloud->points[i].x - p_full_cloud->points[i - 1].x) >
           1e-7) ||
          (std::abs(p_full_cloud->points[i].y - p_full_cloud->points[i - 1].y) >
           1e-7) ||
          (std::abs(p_full_cloud->points[i].z - p_full_cloud->points[i - 1].z) >
           1e-7)) {
        out_cloud[lidar_msg->points[i].line]->push_back(
            p_full_cloud->points[i]);
      }
    }
  }

  if (out_cloud[0]->size() <= 7) {
    return;
  }

  for (int j = 0; j < n_scan; j++) {
    RTPointCloud& pl = *(out_cloud[j]);
    std::vector<orgtype>& types = typess[j];
    int plsize = pl.size();
    types.resize(plsize);
    plsize--;
    for (int i = 0; i < plsize; i++) {
      types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
      vx = pl[i].x - pl[i + 1].x;
      vy = pl[i].y - pl[i + 1].y;
      vz = pl[i].z - pl[i + 1].z;
      types[i].dista = vx * vx + vy * vy + vz * vz;
      // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
    }
    // plsize++;
    types[plsize].range =
        sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);

    ///
    giveFeature(pl, types, *p_corner_cloud, *p_surface_cloud);
  }

  PublishCloud("map");
}
#endif

  void LivoxFeatureExtraction::clearState()
  {
    vx = 0;
    vy = 0;
    vz = 0;
    p_full_cloud->clear();

    // p_corner_cloud->clear();
    // p_surface_cloud->clear(); // 
    p_corner_cloud.reset(new RTPointCloud);
    p_surface_cloud.reset(new RTPointCloud);
  }

  void LivoxFeatureExtraction::AllocateMemory()
  {
    p_full_cloud.reset(new RTPointCloud);
    p_corner_cloud.reset(new RTPointCloud);
    p_surface_cloud.reset(new RTPointCloud);
  }

  void LivoxFeatureExtraction::giveFeature(RTPointCloud &pl,
                                           std::vector<orgtype> &types,
                                           RTPointCloud &pl_corn,
                                           RTPointCloud &pl_surf)
  {
    size_t plsize = pl.size();
    size_t plsize2;
    if (plsize == 0)
    {
      printf("something wrong\n");
      return;
    }
    uint head = 0;

    while (types[head].range < blind)
    {
      head++;
    }

    // Surf
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0;
    uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    for (uint i = head; i < plsize2; i++)
    {
      if (types[i].range < blind)
      {
        continue;
      }
      // i_nex = i;
      i2 = i;
      // std::cout<<" i: "<<i<<" i_nex "<<i_nex<<"group_size: "<<group_size<<"
      // plsize "<<plsize<<" plsize2 "<<plsize2<<std::endl;
      plane_type = checkPlane(pl, types, i, i_nex, curr_direct);

      if (plane_type == 1)
      {
        for (uint j = i; j <= i_nex; j++)
        {
          if (j != i && j != i_nex)
          {
            types[j].ftype = Real_Plane;
          }
          else
          {
            types[j].ftype = Poss_Plane;
          }
        }

        // if(last_state==1 && fabs(last_direct.sum())>0.5)
        if (last_state == 1 && last_direct.norm() > 0.1)
        {
          double mod = last_direct.transpose() * curr_direct;
          if (mod > -0.707 && mod < 0.707)
          {
            types[i].ftype = Edge_Plane;
          }
          else
          {
            types[i].ftype = Real_Plane;
          }
        }

        i = i_nex - 1;
        last_state = 1;
      }
      else if (plane_type == 2)
      {
        i = i_nex;
        last_state = 0;
      }
      else if (plane_type == 0)
      {
        if (last_state == 1)
        {
          uint i_nex_tem;
          uint j;
          for (j = last_i + 1; j <= last_i_nex; j++)
          {
            uint i_nex_tem2 = i_nex_tem;
            Eigen::Vector3d curr_direct2;

            uint ttem = checkPlane(pl, types, j, i_nex_tem, curr_direct2);

            if (ttem != 1)
            {
              i_nex_tem = i_nex_tem2;
              break;
            }
            curr_direct = curr_direct2;
          }

          if (j == last_i + 1)
          {
            last_state = 0;
          }
          else
          {
            for (uint k = last_i_nex; k <= i_nex_tem; k++)
            {
              if (k != i_nex_tem)
              {
                types[k].ftype = Real_Plane;
              }
              else
              {
                types[k].ftype = Poss_Plane;
              }
            }
            i = i_nex_tem - 1;
            i_nex = i_nex_tem;
            i2 = j - 1;
            last_state = 1;
          }
        }
      }

      last_i = i2;
      last_i_nex = i_nex;
      last_direct = curr_direct;
    }

    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; i++)
    {
      if (types[i].range < blind || types[i].ftype >= Real_Plane)
      {
        continue;
      }

      if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
      {
        continue;
      }

      Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
      Eigen::Vector3d vecs[2];

      for (int j = 0; j < 2; j++)
      {
        int m = -1;
        if (j == 1)
        {
          m = 1;
        }

        if (types[i + m].range < blind)
        {
          if (types[i].range > inf_bound)
          {
            types[i].edj[j] = Nr_inf;
          }
          else
          {
            types[i].edj[j] = Nr_blind;
          }
          continue;
        }

        vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
        vecs[j] = vecs[j] - vec_a;

        types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
        if (types[i].angle[j] < jump_up_limit)
        {
          types[i].edj[j] = Nr_180;
        }
        else if (types[i].angle[j] > jump_down_limit)
        {
          types[i].edj[j] = Nr_zero;
        }
      }

      types[i].intersect =
          vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
      if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero &&
          types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
      {
        if (types[i].intersect > cos(160 * 180 / M_PI))
        {
          if (checkCorner(pl, types, i, Prev))
          {
            types[i].ftype = Edge_Jump;
          }
        }
      }
      else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor &&
               types[i - 1].dista > 0.0225 &&
               types[i - 1].dista > 4 * types[i].dista)
      {
        if (types[i].intersect > cos(160 * 180 / M_PI))
        {
          if (checkCorner(pl, types, i, Next))
          {
            types[i].ftype = Edge_Jump;
          }
        }
      }
      else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
      {
        if (checkCorner(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
      else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
      {
        if (checkCorner(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
      else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
      {
        if (types[i].ftype == Nor)
        {
          types[i].ftype = Wire;
        }
      }
    }

    plsize2 = plsize - 1;
    double ratio;
    for (uint i = head + 1; i < plsize2; i++)
    {
      if (types[i].range < blind || types[i - 1].range < blind ||
          types[i + 1].range < blind)
      {
        continue;
      }

      if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
      {
        continue;
      }

      if (types[i].ftype == Nor)
      {
        if (types[i - 1].dista > types[i].dista)
        {
          ratio = types[i - 1].dista / types[i].dista;
        }
        else
        {
          ratio = types[i].dista / types[i - 1].dista;
        }

        if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
        {
          if (types[i - 1].ftype == Nor)
          {
            types[i - 1].ftype = Real_Plane;
          }
          if (types[i + 1].ftype == Nor)
          {
            types[i + 1].ftype = Real_Plane;
          }
          types[i].ftype = Real_Plane;
        }
      }
    }

    int last_surface = -1;
    for (uint j = head; j < plsize; j++)
    {
      if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
      {
        if (last_surface == -1)
        {
          last_surface = j;
        }

        if (j == uint(last_surface + point_filter_num - 1))
        {
          RTPoint ap;

          ap.x = pl[j].x;
          ap.y = pl[j].y;
          ap.z = pl[j].z;
          ap.time = pl[j].time;
          pl_surf.push_back(ap);

          last_surface = -1;
        }
      }
      else
      {
        if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
        {
          pl_corn.push_back(pl[j]);
        }
        if (last_surface != -1)
        {
          RTPoint ap;
          for (uint k = last_surface; k < j; k++)
          {
            ap.x += pl[k].x;
            ap.y += pl[k].y;
            ap.z += pl[k].z;
            // ap.time += pl[k].time;
            ap.time = pl[k].time;
          }
          ap.x /= (j - last_surface);
          ap.y /= (j - last_surface);
          ap.z /= (j - last_surface);
          pl_surf.push_back(ap);
        }
        last_surface = -1;
      }
    }
  }

  void LivoxFeatureExtraction::giveFeatureR3LIVE(RTPointCloud &pl,
                                                 std::vector<orgtype> &types,
                                                 RTPointCloud &pl_corn,
                                                 RTPointCloud &pl_surf)
  {
    size_t plsize = pl.size();
    size_t plsize2;
    if (plsize == 0)
    {
      printf("something wrong\n");
      return;
    }
    uint head = 0;
    while (types[head].range < blind)
    {
      head++;
    }

    // Surf
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

    uint i_nex = 0, i2;
    uint last_i = 0;
    uint last_i_nex = 0;
    int last_state = 0;
    int plane_type;

    RTPoint ap;
    int g_LiDAR_sampling_point_step = 1;
    for (uint i = head; i < plsize2; i += g_LiDAR_sampling_point_step)
    {
      if (types[i].range > blind)
      {
        ap.x = pl[i].x;
        ap.y = pl[i].y;
        ap.z = pl[i].z;
        ap.time = pl[i].time;
        ap.intensity = pl[i].intensity;
        pl_surf.push_back(ap);
      }
    }

    return;
  }

  int LivoxFeatureExtraction::checkPlane(const RTPointCloud &pl,
                                         std::vector<orgtype> &types, uint i_cur,
                                         uint &i_nex,
                                         Eigen::Vector3d &curr_direct)
  {
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis = 0;
    vector<double> disarr;
    disarr.reserve(20);

    for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
    {
      if (types[i_nex].range < blind)
      {
        curr_direct.setZero();
        return 2;
      }
      disarr.push_back(types[i_nex].dista);
    }

    for (;;)
    {
      if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
        break;

      if (types[i_nex].range < blind)
      {
        curr_direct.setZero();
        return 2;
      }
      vx = pl[i_nex].x - pl[i_cur].x;
      vy = pl[i_nex].y - pl[i_cur].y;
      vz = pl[i_nex].z - pl[i_cur].z;
      two_dis = vx * vx + vy * vy + vz * vz;
      if (two_dis >= group_dis)
      {
        break;
      }
      disarr.push_back(types[i_nex].dista);
      i_nex++;
    }

    double leng_wid = 0;
    double v1[3], v2[3];
    for (uint j = i_cur + 1; j < i_nex; j++)
    {
      if ((j >= pl.size()) || (i_cur >= pl.size()))
        break;
      v1[0] = pl[j].x - pl[i_cur].x;
      v1[1] = pl[j].y - pl[i_cur].y;
      v1[2] = pl[j].z - pl[i_cur].z;

      v2[0] = v1[1] * vz - vy * v1[2];
      v2[1] = v1[2] * vx - v1[0] * vz;
      v2[2] = v1[0] * vy - vx * v1[1];

      double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
      if (lw > leng_wid)
      {
        leng_wid = lw;
      }
    }

    if ((two_dis * two_dis / leng_wid) < p2l_ratio)
    {
      curr_direct.setZero();
      return 0;
    }

    uint disarrsize = disarr.size();
    for (uint j = 0; j < disarrsize - 1; j++)
    {
      for (uint k = j + 1; k < disarrsize; k++)
      {
        if (disarr[j] < disarr[k])
        {
          leng_wid = disarr[j];
          disarr[j] = disarr[k];
          disarr[k] = leng_wid;
        }
      }
    }

    if (disarr[disarr.size() - 2] < 1e-16)
    {
      curr_direct.setZero();
      return 0;
    }

    //  if(lidar_type==MID || lidar_type==HORIZON)
    //  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
    //  }
    //  else
    //  {
    //    double dismax_min = disarr[0] / disarr[disarrsize-2];
    //    if(dismax_min >= limit_maxmin)
    //    {
    //      curr_direct.setZero();
    //      return 0;
    //    }
    //  }

    curr_direct << vx, vy, vz;
    curr_direct.normalize();
    return 1;
  }

  bool LivoxFeatureExtraction::checkCorner(const RTPointCloud &pl,
                                           std::vector<orgtype> &types, uint i,
                                           Surround nor_dir)
  {
    if (nor_dir == 0)
    {
      if (types[i - 1].range < blind || types[i - 2].range < blind)
      {
        return false;
      }
    }
    else if (nor_dir == 1)
    {
      if (types[i + 1].range < blind || types[i + 2].range < blind)
      {
        return false;
      }
    }
    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2)
    {
      d = d1;
      d1 = d2;
      d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);

    if (d1 > edgea * d2 || (d1 - d2) > edgeb)
    {
      return false;
    }

    return true;
  }

  void LivoxFeatureExtraction::PublishCloud(std::string frame_id)
  {
    sensor_msgs::PointCloud2 corner_msg;
    sensor_msgs::PointCloud2 surface_msg;

    pcl::toROSMsg(*p_corner_cloud, corner_msg);
    pcl::toROSMsg(*p_surface_cloud, surface_msg);

    corner_msg.header.stamp = ros::Time::now();
    corner_msg.header.frame_id = frame_id;
    surface_msg.header.stamp = ros::Time::now();
    surface_msg.header.frame_id = frame_id;

    if (pub_corner_cloud.getNumSubscribers() != 0)
      pub_corner_cloud.publish(corner_msg);
    if (pub_surface_cloud.getNumSubscribers() != 0)
      pub_surface_cloud.publish(surface_msg);
  }

} // namespace cocolic
