#ifndef IMU_PROCESSING_HPP_
#define IMU_PROCESSING_HPP_
#include <cmath>
#include <math.h>
#include <Eigen/Eigen>
#include <omp.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include "tools/imu.h"
#include "tools/nav_state.h"
#include "common_lib.hpp"

#define DIM_OF_PROC_N 12

namespace zjloc
{

     class ImuProcess
     {

     public:
          ImuProcess(/* args */);

          ~ImuProcess();

          void set_extrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot);

          void Process(MeasureGroup &meas, StatesGroup &stat);

          bool isInitSuccess() { return !imu_need_init_; }

          void setLog(bool &log_print) { log_print_ = log_print; }

     private:
          void IMU_Initial(const MeasureGroup &meas, StatesGroup &state_inout, int &N);

          void UndistortPcl(MeasureGroup &meas, StatesGroup &state_inout);

     private:
          bool b_first_frame_ = true;
          bool imu_need_init_ = true;
          bool log_print_ = false;

          IMUPtr last_imu_;
          double last_lidar_end_time_ = 0;
          double start_timestamp_;
          int init_iter_num = 1;

          Eigen::Vector3d angvel_last;
          Eigen::Vector3d acc_s_last;

          Eigen::Vector3d cov_acc;
          Eigen::Vector3d cov_gyr;
          Eigen::Vector3d mean_acc;
          Eigen::Vector3d mean_gyr;

          Eigen::Matrix3d Lidar_R_wrt_IMU;
          Eigen::Vector3d Lidar_T_wrt_IMU;

          Eigen::Matrix<double, DIM_OF_PROC_N, 1> cov_proc_noise;
          std::vector<pose6d> IMUpose;
     };
}
#endif // IMU_PROCESSING_HPP_