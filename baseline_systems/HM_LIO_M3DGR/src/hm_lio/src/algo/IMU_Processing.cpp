#include "IMU_Processing.hpp"

namespace zjloc
{

     ImuProcess::ImuProcess(/* args */)
         : b_first_frame_(true), imu_need_init_(true), last_imu_(nullptr), start_timestamp_(-1)
     {
          cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
          cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
          mean_acc = Eigen::Vector3d(0, 0, -1.0);
          mean_gyr = Eigen::Vector3d(0, 0, 0);
          angvel_last = Eigen::Vector3d(0, 0, 0);
          ;
          Lidar_T_wrt_IMU = Eigen::Vector3d(0, 0, 0);
          ;
          Lidar_R_wrt_IMU = Eigen::Matrix3d::Identity();
          cov_proc_noise = Eigen::Matrix<double, DIM_OF_PROC_N, 1>::Zero();
          last_imu_.reset(new IMU());
     }

     ImuProcess::~ImuProcess() {}

     void ImuProcess::set_extrinsic(const Eigen::Vector3d &transl, const Eigen::Matrix3d &rot)
     {
          Lidar_T_wrt_IMU = transl;
          Lidar_R_wrt_IMU = rot;
     }

     void ImuProcess::Process(MeasureGroup &meas, StatesGroup &stat)
     {
          if (meas.imu_.empty())
          {
               std::cout << "no imu data" << std::endl;
               return;
          }
          if (imu_need_init_)
          {
               IMU_Initial(meas, stat, init_iter_num);
               imu_need_init_ = true;
               last_imu_ = meas.imu_.back();
               if (init_iter_num > 100)
               {
                    imu_need_init_ = false;
                    cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
                    cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);

                    printf("IMU Initials: Gravity: %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f\n",
                           stat.gravity[0], stat.gravity[1], stat.gravity[2], stat.bias_g[0], stat.bias_g[1], stat.bias_g[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
               }
               return;
          }
          UndistortPcl(meas, stat);
          last_imu_ = meas.imu_.back();
     }

     void ImuProcess::IMU_Initial(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
     {
          printf("IMU Initializing: %.1f %%\n", double(N) / 100 * 100);
          Eigen::Vector3d cur_acc, cur_gyr;

          if (b_first_frame_)
          {
               N = 1;
               b_first_frame_ = false;
               const auto &imu_acc = meas.imu_.front()->acce_;
               const auto &gyr_acc = meas.imu_.front()->gyro_;
               mean_acc << imu_acc[0], imu_acc[1], imu_acc[2];
               mean_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];
          }

          std::deque<IMUPtr> tmp_imu_ = meas.imu_;
          while (!tmp_imu_.empty())
          {
               auto imu = tmp_imu_.front();
               const auto &imu_acc = imu->acce_;
               const auto &gyr_acc = imu->gyro_;
               cur_acc << imu_acc[0], imu_acc[1], imu_acc[2];
               cur_gyr << gyr_acc[0], gyr_acc[1], gyr_acc[2];

               mean_acc += (cur_acc - mean_acc) / N;
               mean_gyr += (cur_gyr - mean_gyr) / N;

               cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
               cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
               tmp_imu_.pop_front();
               N++;
          }

          state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
          // state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(Eigen::Vector3d(0, 0, -1 / scale_gravity)));
          state_inout.bias_g = mean_gyr;
          state_inout.R_L_I = Lidar_R_wrt_IMU;
          state_inout.T_L_I = Lidar_T_wrt_IMU;

          last_imu_ = meas.imu_.back();
     }

     // const bool time_list(point3D &x, point3D &y) { return (x.timestamp < y.timestamp); };

     void ImuProcess::UndistortPcl(MeasureGroup &meas, StatesGroup &state_inout)
     {
          auto v_imu = meas.imu_;
          v_imu.push_front(last_imu_);

          const double &imu_beg_time = v_imu.front()->timestamp_;
          const double &imu_end_time = v_imu.back()->timestamp_;
          const double &pcl_beg_time = meas.lidar_begin_time_;
          const double &pcl_end_time = meas.lidar_end_time_;

          auto time_list = [&](point3D &point_1, point3D &point_2)
          {
               return (point_1.relative_time < point_2.relative_time);
          };

          std::sort(meas.lidar_.begin(), meas.lidar_.end(), time_list);

          IMUpose.clear();
          IMUpose.emplace_back(pose6d(0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

          /*** forward propagation at each imu point ***/
          Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
          Eigen::Matrix3d R_imu(state_inout.rot_end);
          Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
          Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
          double dt = 0;

          for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
          {
               auto &&head = *(it_imu);
               auto &&tail = *(it_imu + 1);
               if (tail->timestamp_ < last_lidar_end_time_)
                    continue;

               angvel_avr = 0.5 * (head->gyro_ + tail->gyro_);
               acc_avr = 0.5 * (head->acce_ + tail->acce_);

               angvel_avr -= state_inout.bias_g;
               acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

               if (head->timestamp_ < last_lidar_end_time_)
                    dt = tail->timestamp_ - last_lidar_end_time_;
               else
                    dt = tail->timestamp_ - head->timestamp_;

               /* covariance propagation */
               Eigen::Matrix3d acc_avr_skew;
               Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
               acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

               F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
               F_x.block<3, 3>(0, 15) = -Eye3d * dt;
               // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
               F_x.block<3, 3>(3, 12) = Eye3d * dt;
               F_x.block<3, 3>(12, 0) = -R_imu * acc_avr_skew * dt;
               F_x.block<3, 3>(12, 18) = -R_imu * dt;
               F_x.block<3, 3>(12, 21) = Eye3d * dt;

               Eigen::Matrix3d cov_acc_diag(Eye3d), cov_gyr_diag(Eye3d);
               cov_acc_diag.diagonal() = cov_acc;
               cov_gyr_diag.diagonal() = cov_gyr;
               cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt * 10000;
               cov_w.block<3, 3>(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt * 10000;
               //  TODO: extrinsic noise ???
               cov_w.block<3, 3>(12, 12) = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt * 10000;
               cov_w.block<3, 3>(15, 15).diagonal() = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt; // bias gyro covariance
               cov_w.block<3, 3>(18, 18).diagonal() = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt; // bias acc covariance

               state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w; //  ESKF 预测值和真值的协方差 --- [2]

               R_imu = R_imu * Exp_f;
               acc_imu = R_imu * acc_avr + state_inout.gravity;
               pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
               vel_imu = vel_imu + acc_imu * dt;
               angvel_last = angvel_avr;
               acc_s_last = acc_imu;
               double &&offs_t = tail->timestamp_ - pcl_beg_time;
               IMUpose.push_back(pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
               // std::cout << "add imu pose t: " << offs_t << std::endl;
          }
          dt = pcl_end_time - imu_end_time;
          state_inout.vel_end = vel_imu + acc_imu * dt;
          state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
          state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt; //  ESKF 预测值  --- [1]

          last_lidar_end_time_ = pcl_end_time;

          if (log_print_)
          {
               std::cout << "[ IMU Process ]: vel " << state_inout.vel_end.transpose() << " pos "
                         << state_inout.pos_end.transpose() << " ba" << state_inout.bias_a.transpose()
                         << " bg " << state_inout.bias_g.transpose() << std::endl;
               std::cout << "propagated cov: " << state_inout.cov.diagonal().transpose() << std::endl;
          }

          /*** undistort each lidar point (backward propagation) ***/
          auto it_point = meas.lidar_.end() - 1;
          for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
          {
               auto head = it_kp - 1;
               auto tail = it_kp;
               R_imu = head->rot;
               acc_imu << VEC_FROM_ARRAY(head->acc);

               vel_imu << VEC_FROM_ARRAY(head->vel);
               pos_imu << VEC_FROM_ARRAY(head->pos);
               angvel_avr << VEC_FROM_ARRAY(head->gyr);

               for (; it_point->relative_time > head->offset_time; it_point--)
               {
                    dt = it_point->relative_time - head->offset_time;
                    Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
                    Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end);
                    it_point->raw_point = state_inout.R_L_I.transpose() *
                                          (state_inout.rot_end.transpose() *
                                               (R_i * (state_inout.R_L_I * it_point->raw_point + state_inout.T_L_I) + T_ei) -
                                           state_inout.T_L_I);

                    if (it_point == meas.lidar_.begin())
                         break;
               }
          }
     }
}