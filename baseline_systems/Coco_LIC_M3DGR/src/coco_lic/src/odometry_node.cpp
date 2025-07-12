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

#include <ros/package.h>
#include <ros/ros.h>

#include <odom/odometry_manager.h>

using namespace cocolic;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "cocolic");
  ros::NodeHandle nh("~");

  std::string config_path;
  nh.param<std::string>("config_path", config_path, "ct_odometry.yaml");
  ROS_INFO("Odometry load %s.", config_path.c_str());
    //config_path = "/media/zjj/Newsmy/linux/Coco-LIC/src/coco_lic/config/ct_odometry_r3live.yaml" ;
  YAML::Node config_node = YAML::LoadFile(config_path);

  std::string log_path = config_node["log_path"].as<std::string>();
  FLAGS_log_dir = log_path;
  FLAGS_colorlogtostderr = true;
  std::cout << "\n🥥 Start Coco-LIC Odometry 🥥";

  OdometryManager odom_manager(config_node, nh);
  odom_manager.RunBag();

  double t_traj_max = odom_manager.SaveOdometry();
  std::cout << "\n✨ All Done.\n\n";

  return 0;
}
