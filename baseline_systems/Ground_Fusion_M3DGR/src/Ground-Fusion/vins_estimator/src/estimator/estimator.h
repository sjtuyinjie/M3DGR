/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <thread>
#include <mutex>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "feature_manager.h"
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../initial/solve_5pts.h"
#include "../initial/initial_sfm.h"
#include "../initial/initial_alignment.h"
#include "../initial/initial_ex_rotation.h"
#include "../factor/imu_factor.h"
#include "../factor/pose_local_parameterization.h"
#include "../factor/marginalization_factor.h"
#include "../factor/projectionTwoFrameOneCamFactor.h"
#include "../factor/projectionTwoFrameTwoCamFactor.h"
#include "../factor/projectionOneFrameTwoCamFactor.h"
#include "../factor/projection_factor.h"
#include "../featureTracker/feature_tracker.h"
#include "../linefeatureTracker/linefeature_tracker.h"
#include "../factor/plane_factor.h"
#include "../factor/motion_factor.h"
#include "../factor/wheel_factor.h"
#include "../factor/orientation_local_parameterization.h"

#include "../factor/line_parameterization.h" //line
#include "../factor/line_projection_factor.h"

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h> //box

// gnss new

#include "../factor/gnss_psr_dopp_factor.hpp"
#include "../factor/gnss_dt_ddt_factor.hpp"
#include "../factor/gnss_dt_anchor_factor.hpp"
#include "../factor/gnss_ddt_smooth_factor.hpp"
#include "../factor/pos_vel_factor.hpp"
#include "../factor/pose_anchor_factor.h"
#include "../initial/gnss_vi_initializer.h"

#include <opencv2/core/eigen.hpp>

#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_spp.hpp>

struct RetriveData // line
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Vector3d P_old;
    Matrix3d R_old;
    vector<cv::Point2f> measurements;
    vector<int> features_ids;
    bool relocalized;
    bool relative_pose;
    Vector3d relative_t;
    Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};

class Estimator
{
public:
    Estimator();

    void setParameter();
    // line
    void optimizationwithLine();
    void onlyLineOpt();
    // gnss new
    void processGNSS(const std::vector<ObsPtr> &gnss_mea);
    void inputEphem(EphemBasePtr ephem_ptr);
    void inputIonoParams(double ts, const std::vector<double> &iono_params);
    void inputGNSSTimeDiff(const double t_diff);

    // interface
    void initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);
    // void inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);
    void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void inputImagebox(double t, const darknet_ros_msgs::BoundingBoxesConstPtr &_boxes, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void inputImagewithline(double t, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header);
    // void processImagewithline(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header,map<int, vector<pair<int, Vector4d>>>&linefeature);
    void processImagewithline(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> &image, const double header, const map<int, vector<pair<int, Vector4d>>> &linefeature);
    void processMeasurements();
    void inputWheel(double t, const Vector3d &linearVelocity, const Vector3d &angularVelocity);
    void inputGNSS(double t, std::vector<ObsPtr> meas_msg);
    void inputrawodom(double t, const Vector3d &wheel_xyz);
    // void inputGroundtruth(double t, Eigen::Matrix<double, 7, 1>& data);
    void processWheel(double t, double dt, const Vector3d &linear_velocity, const Vector3d &angular_velocity);
    void integrateWheelPreintegration(double t, Eigen::Vector3d &P, Eigen::Quaterniond &Q, const Eigen::Matrix<double, 7, 1> &pose);
    void changeSensorType(int use_imu, int use_stereo);
    // void inputFeature(double t, const vector<cv::Point2f>& _features0, const vector<cv::Point2f>& _features1=vector<cv::Point2f>());//仿真的feature
    bool getWheelInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &velVector,
                          vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void fastPredictWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity);
    void fastPredictPureWheel(double t, Eigen::Vector3d linear_velocity, Eigen::Vector3d angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V);
    bool WheelAvailable(double t);
    void initPlane();
    bool getGNSSInterval(double t0, double t1);
    // bool GNSSAvailable(double t);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    bool GNSSVIAlign();
    void updateGNSSStatistics();

    // bool visualInitialAlignWithDepth();
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool relativePoseWithDepth(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    bool checkvisual(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void checkimuexcited();
    void checkimu();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    // void vector2double2();
    void vector2doubleline();
    void double2vector();
    void double2vectorline();
    void double2vector2(); // line
    bool failureDetection();
    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                        vector<pair<double, Eigen::Vector3d>> &gyrVector);
    void getPoseInWorldFrame(Eigen::Matrix4d &T);
    void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);
    void predictPtsInNextFrame();
    void outliersRejection(set<int> &removeIndex);
    void movingConsistencyCheckW(set<int> &removeIndex);
    double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                             Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                             double depth, Vector3d &uvi, Vector3d &uvj);
    double reprojectionError3D(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                               Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                               double depth, Vector3d &uvi, Vector3d &uvj);
    void updateLatestStates();
    void fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity);
    void fastPredictPureIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity, Eigen::Vector3d &P, Eigen::Quaterniond &Q, Eigen::Vector3d &V);
    bool IMUAvailable(double t);
    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    // anomaly
    Eigen::Vector3d dP_wheel, dP_imu, P_wheel_tmp, P_imu_tmp;

    // line
    double frame_cnt_ = 0;
    double sum_solver_time_ = 0.0;
    double mean_solver_time_ = 0.0;
    double sum_marg_time_ = 0.0;
    double mean_marg_time_ = 0.0;

    std::mutex mGTBuf;
    std::mutex mWheelBuf;
    std::mutex mRawodomBuf;
    std::mutex mVioBuf;
    std::mutex mPropagate;
    std::mutex mWheelPropagate;
    std::mutex mBuf;
    queue<pair<double, Eigen::Vector3d>> wheelVelBuf;
    queue<pair<double, Eigen::Vector3d>> wheelGyrBuf;
    queue<pair<double, Eigen::Vector3d>> wheelxyztBuf;
    queue<pair<double, Eigen::Vector3d>> vioxyztBuf;
    queue<pair<Eigen::Vector3d, Eigen::Vector3d>> syncBuf;

    // queue<pair<double, Eigen::Matrix<double,7,1>>> groundtruthBuf;
    double prevTime_wheel, curTime_wheel;
    bool openExWheelEstimation;
    bool openIxEstimation;
    bool openPlaneEstimation;
    bool openMotionEstimation;
    Matrix3d rio;
    Vector3d tio;
    Matrix3d rpw;
    double zpw;
    double sx = 1, sy = 1, sw = 1;
    double vx = 0, vy = 0, vz = 0;
    double td_wheel;
    WheelIntegrationBase *pre_integrations_wheel[(WINDOW_SIZE + 1)];
    Vector3d vel_0_wheel, gyr_0_wheel;
    vector<double> dt_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_velocity_buf_wheel[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf_wheel[(WINDOW_SIZE + 1)];

    bool first_wheel;
    bool Bas_calibok = false;
    double para_LineFeature[NUM_OF_F][SIZE_LINE]; // line
    double para_Ex_Pose_wheel[1][SIZE_POSE];
    double para_plane_R[1][SIZE_ROTATION];
    double para_motion[1][3];
    double para_plane_Z[1][1];
    double para_Ix_sx_wheel[1][1];
    double para_Ix_sy_wheel[1][1];
    double para_Ix_sw_wheel[1][1];
    double para_Td_wheel[1][1];
    WheelIntegrationBase *tmp_wheel_pre_integration;
    double latest_time_wheel;
    Eigen::Vector3d latest_P_wheel, latest_V_wheel, latest_vel_wheel_0, latest_gyr_wheel_0;
    double latest_sx, latest_sy, latest_sw;
    Eigen::Quaterniond latest_Q_wheel;

    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;
    queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>>> featureBuf;
    // queue<pair<double, map<int, vector<pair<int, Matrix<double, 5, 1>>>>>> linefeatureBuf;
    queue<pair<double, map<int, vector<pair<int, Vector4d>>>>> linefeatureBuf;

    // const
    double prevTime, curTime;
    bool openExEstimation;

    std::thread trackThread;
    std::thread processThread;

    FeatureTracker featureTracker;
    LineFeatureTracker linefeatureTracker; // line

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    Vector3d g;

    Matrix3d ric[2];
    Vector3d tic[2];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    // Vector4d vio_xyzt[];
    // Vector3d vio_xyz[];
    // Vector4d wheel_xyzt[];
    // Vector3d wheel_xyz[];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    double Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    // GNSS related
    queue<pair<double, std::vector<ObsPtr>>> GNSSBuf;
    std::vector<ObsPtr> gnss_msg;
    std::mutex mGNSSBuf;
    bool first_optimization;

    bool gnss_ready;
    Eigen::Vector3d anc_ecef;
    Eigen::Matrix3d R_ecef_enu;
    double yaw_enu_local;
    std::vector<ObsPtr> gnss_meas_buf[(WINDOW_SIZE + 1)];
    std::vector<EphemBasePtr> gnss_ephem_buf[(WINDOW_SIZE + 1)];
    std::vector<double> latest_gnss_iono_params;
    std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
    std::map<uint32_t, std::map<double, size_t>> sat2time_index;
    std::map<uint32_t, uint32_t> sat_track_status;
    double para_anc_ecef[3];
    double para_yaw_enu_local[1];
    double para_rcv_dt[(WINDOW_SIZE + 1) * 4];
    double para_rcv_ddt[WINDOW_SIZE + 1];
    // GNSS statistics
    double diff_t_gnss_local;
    Eigen::Matrix3d R_enu_local;
    Eigen::Vector3d ecef_pos, enu_pos, enu_vel, enu_ypr;

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;
    int inputImageCnt;

    FeatureManager f_manager;
    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;
    int ini_ok = 0;
    long double ini_starttime = 0;
    long double ini_endtime = 0;
    long double cur_time = 0;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[2][SIZE_POSE];
    double para_Retrive_Pose[SIZE_POSE];
    double para_Td[1][1];
    double para_Tr[1][1];

    int loop_window_index;

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    double latest_time;
    Eigen::Vector3d latest_P, latest_V, latest_Ba, latest_Bg, latest_acc_0, latest_gyr_0;
    Eigen::Quaterniond latest_Q;

    bool initFirstPoseFlag;
    bool wheelanomaly;
    bool visualstationary;
    bool varstationary;
    bool preintegrationstationary;
    bool imustationary;
    bool wheelstationary;
    bool systemstationary;
    bool lowspeed;
};
