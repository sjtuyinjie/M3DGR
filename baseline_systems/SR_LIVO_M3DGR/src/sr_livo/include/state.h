#pragma once
// c++
#include <iostream>

// eigen 
#include <Eigen/Core>

// ceres
#include <ceres/ceres.h>

// utility
#include "utility.h"

class state
{
public:

	Eigen::Quaterniond rotation;
	Eigen::Vector3d translation;
	Eigen::Vector3d velocity;
	Eigen::Vector3d ba;
	Eigen::Vector3d bg;

	double fx, fy, cx, cy;

    Eigen::Matrix3d R_imu_camera;
    Eigen::Vector3d t_imu_camera;

    Eigen::Quaterniond q_world_camera;
    Eigen::Vector3d t_world_camera;

    Eigen::Quaterniond q_camera_world;
    Eigen::Vector3d t_camera_world;

    double fov_margin;
    double time_td;

	state();

	state(const Eigen::Quaterniond &rotation_, const Eigen::Vector3d &translation_, 
		const Eigen::Vector3d &velocity_, const Eigen::Vector3d& ba_, const Eigen::Vector3d& bg_);

	state(const state* state_temp);

	void release();
};