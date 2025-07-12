#ifndef LIO_UTILS_H_
#define LIO_UTILS_H_

#include "common/eigen_types.h"
#include "tools/imu.h"
#include "common/math_utils.h"
#include "tools/nav_state.h"
#include "tools/odom.h"

#include "common/cloudMap.hpp"

// #define DIM_OF_STATES (18) // Dimension of states (Let Dim(SO(3)) = 3)     R    p    v    bg   ba   g
// #define INIT_COV (1)
// #define INIT_TIME (0)
// #define LASER_POINT_COV (0.0015)

// #define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
// #define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
// #define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

namespace zjloc
{

     /*  struct Neighborhood
       {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            Eigen::Vector3d normal = Eigen::Vector3d::Zero();
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
            double a2D = 1.0; // Planarity coefficient
       };

       struct MeasureGroup
       {
            double lidar_begin_time_ = 0; // 雷达包的起始时间
            double lidar_end_time_ = 0;   // 雷达的终止时间
            std::vector<point3D> lidar_;  // 雷达点云
            std::deque<IMUPtr> imu_;      // 上一时时刻到现在的IMU读数
       };

       struct StatesGroup
       {
            StatesGroup()
            {
                 this->rot_end = Eigen::Matrix3d::Identity();
                 this->pos_end = Zero3d;
                 this->R_L_I = Eigen::Matrix3d::Identity();
                 this->T_L_I = Zero3d;
                 this->vel_end = Zero3d;
                 this->bias_g = Zero3d;
                 this->bias_a = Zero3d;
                 this->gravity = Zero3d;
                 this->cov = Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity() * INIT_COV;
            };

            StatesGroup(const StatesGroup &b)
            {
                 this->rot_end = b.rot_end;
                 this->pos_end = b.pos_end;
                 this->R_L_I = b.R_L_I;
                 this->T_L_I = b.T_L_I;
                 this->vel_end = b.vel_end;
                 this->bias_g = b.bias_g;
                 this->bias_a = b.bias_a;
                 this->gravity = b.gravity;
                 this->cov = b.cov;
            };

            StatesGroup &operator=(const StatesGroup &b)
            {
                 this->rot_end = b.rot_end;
                 this->pos_end = b.pos_end;
                 this->R_L_I = b.R_L_I;
                 this->T_L_I = b.T_L_I;
                 this->vel_end = b.vel_end;
                 this->bias_g = b.bias_g;
                 this->bias_a = b.bias_a;
                 this->gravity = b.gravity;
                 this->cov = b.cov;
                 return *this;
            };

            StatesGroup operator+(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
            {
                 StatesGroup a;
                 a.rot_end = this->rot_end * zjloc::math::Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
                 a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
                 a.R_L_I = this->R_L_I * zjloc::math::Exp(state_add(6, 0), state_add(7, 0), state_add(8, 0));
                 a.T_L_I = this->T_L_I + state_add.block<3, 1>(9, 0);
                 a.vel_end = this->vel_end + state_add.block<3, 1>(12, 0);
                 a.bias_g = this->bias_g + state_add.block<3, 1>(15, 0);
                 a.bias_a = this->bias_a + state_add.block<3, 1>(18, 0);
                 a.gravity = this->gravity + state_add.block<3, 1>(21, 0);
                 a.cov = this->cov;
                 return a;
            };

            StatesGroup &operator+=(const Eigen::Matrix<double, DIM_OF_STATES, 1> &state_add)
            {
                 this->rot_end = this->rot_end * zjloc::math::Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
                 this->pos_end += state_add.block<3, 1>(3, 0);
                 this->R_L_I = this->R_L_I * zjloc::math::Exp(state_add(6, 0), state_add(7, 0), state_add(8, 0));
                 this->T_L_I += state_add.block<3, 1>(9, 0);
                 this->vel_end += state_add.block<3, 1>(12, 0);
                 this->bias_g += state_add.block<3, 1>(15, 0);
                 this->bias_a += state_add.block<3, 1>(18, 0);
                 this->gravity += state_add.block<3, 1>(21, 0);
                 return *this;
            };

            Eigen::Matrix<double, DIM_OF_STATES, 1> operator-(const StatesGroup &b)
            {
                 Eigen::Matrix<double, DIM_OF_STATES, 1> a;
                 Eigen::Matrix3d rotd(b.rot_end.transpose() * this->rot_end);
                 Eigen::Matrix3d rotLI(b.R_L_I.transpose() * this->R_L_I);
                 a.block<3, 1>(0, 0) = zjloc::math::Log(rotd);
                 a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
                 a.block<3, 1>(6, 0) = zjloc::math::Log(rotLI);
                 a.block<3, 1>(9, 0) = this->T_L_I - b.T_L_I;
                 a.block<3, 1>(12, 0) = this->vel_end - b.vel_end;
                 a.block<3, 1>(15, 0) = this->bias_g - b.bias_g;
                 a.block<3, 1>(18, 0) = this->bias_a - b.bias_a;
                 a.block<3, 1>(21, 0) = this->gravity - b.gravity;
                 return a;
            };

            Eigen::Matrix3d rot_end;                                 // the estimated attitude (rotation matrix) at the end lidar point
            Eigen::Vector3d pos_end;                                 // the estimated position at the end lidar point (world frame)
            Eigen::Matrix3d R_L_I;                                   // Rotation from Lidar frame L to IMU frame I
            Eigen::Vector3d T_L_I;                                   // Translation from Lidar frame L to IMU frame I
            Eigen::Vector3d vel_end;                                 // the estimated velocity at the end lidar point (world frame)
            Eigen::Vector3d bias_g;                                  // gyroscope bias
            Eigen::Vector3d bias_a;                                  // accelerator bias
            Eigen::Vector3d gravity;                                 // the estimated gravity acceleration
            Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES> cov; // states covariance
       };*/

     class numType
     {
     public:
          template <typename Derived>
          static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
          {
               typedef typename Derived::Scalar Scalar_t;

               Eigen::Quaternion<Scalar_t> dq;
               Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
               half_theta /= static_cast<Scalar_t>(2.0);
               dq.w() = static_cast<Scalar_t>(1.0);
               dq.x() = half_theta.x();
               dq.y() = half_theta.y();
               dq.z() = half_theta.z();
               dq.normalize();
               return dq;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &mat)
          {
               Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
               mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
                   mat(2), typename Derived::Scalar(0), -mat(0),
                   -mat(1), mat(0), typename Derived::Scalar(0);
               return mat_skew;
          }

          template <typename Derived>
          static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
          {
               return q;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
          {
               Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
               Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
               ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
               ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
               return ans;
          }

          template <typename Derived>
          static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
          {
               Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
               Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
               ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
               ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
               return ans;
          }
     };
}
#endif // LIO_UTILS_H_