/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "../utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <map>

using namespace std;

const double FOCAL_LENGTH = 600.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;
const int LINE_MIN_OBS = 5; // line
// #define UNIT_SPHERE_ERROR
extern int USE_LINE;
extern int USE_YOLO;
extern int USE_MCC;
extern int ESTIMATE_EXTRINSIC_WHEEL;
extern int ESTIMATE_INTRINSIC_WHEEL;
extern int EQUALIZE;
extern double VEL_N_wheel;
extern double GYR_N_wheel;
extern double SX;
extern double SY;
extern double SW;

extern double ROLL_N, PITCH_N, ZPW_N;
extern double ROLL_N_INV, PITCH_N_INV, ZPW_N_INV;

extern Eigen::Matrix3d RIO;
extern Eigen::Vector3d TIO;

extern std::string IN_CALIB_RESULT_PATH;
extern std::string INTRINSIC_ITERATE_PATH;
extern std::string EXTRINSIC_WHEEL_ITERATE_PATH;
extern std::string EXTRINSIC_CAM_ITERATE_PATH;
extern std::string PROCESS_TIME_PATH;
extern std::string TD_WHEEL_PATH;
extern std::string TD_PATH;
extern std::string GROUNDTRUTH_PATH;
extern std::string WHEEL_TOPIC;

extern int USE_WHEEL;
extern int depth_threshold;
extern int w_replace;
extern int wdetect;
extern int stationary_detect;
extern bool is_imu_excited;
extern int vision_failure;
extern int USE_PLANE;
extern int USE_MOTION;
extern int wheel_ready;
extern int STATIONARY_INI;
extern int ONLY_INITIAL_WITH_WHEEL;

extern double OFFSET_SIM;
extern double TD_WHEEL;
extern int ESTIMATE_TD_WHEEL;

extern std::string FEATURE0_TOPIC, FEATURE1_TOPIC;
extern std::string GROUNDTRUTH_TOPIC;

enum CameraExtrinsicAdjustType
{
    ADJUST_CAM_TRANSLATION,
    ADJUST_CAM_ROTATION,
    ADJUST_CAM_ALL,
    ADJUST_CAM_NO_Z,
    ADJUST_CAM_NO_ROTATION_NO_Z
};
enum WheelExtrinsicAdjustType
{
    ADJUST_WHEEL_TRANSLATION,
    ADJUST_WHEEL_ROTATION,
    ADJUST_WHEEL_ALL,
    ADJUST_WHEEL_NO_Z,
    ADJUST_WHEEL_NO_ROTATION_NO_Z
};
extern CameraExtrinsicAdjustType CAM_EXT_ADJ_TYPE;
extern WheelExtrinsicAdjustType WHEEL_EXT_ADJ_TYPE;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string VINS_RESULT_PATH2;
extern std::string OUTPUT_FOLDER;
extern std::string IMU_TOPIC;
extern double TD;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern int ROW, COL;
extern int NUM_OF_CAM;
extern int STEREO;
extern int DEPTH;
extern int USE_IMU;
extern int MULTIPLE_THREAD;
// pts_gt for debug purpose;
extern map<int, Eigen::Vector3d> pts_gt;

extern std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int FLOW_BACK;

// gnss new
extern bool GNSS_ENABLE;
extern std::string GNSS_EPHEM_TOPIC;
extern std::string GNSS_GLO_EPHEM_TOPIC;
extern std::string GNSS_MEAS_TOPIC;
extern std::string GNSS_IONO_PARAMS_TOPIC;
extern std::string GNSS_TP_INFO_TOPIC;
extern std::vector<double> GNSS_IONO_DEFAULT_PARAMS;
extern bool GNSS_LOCAL_ONLINE_SYNC;
extern std::string LOCAL_TRIGGER_INFO_TOPIC;
extern double GNSS_LOCAL_TIME_DIFF;
extern double GNSS_ELEVATION_THRES;
extern double GNSS_PSR_STD_THRES;
extern double GNSS_DOPP_STD_THRES;
extern uint32_t GNSS_TRACK_NUM_THRES;
extern double GNSS_DDT_WEIGHT;
extern std::string GNSS_RESULT_PATH;

void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_ROTATION = 4,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_LINE = 4 // line
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
