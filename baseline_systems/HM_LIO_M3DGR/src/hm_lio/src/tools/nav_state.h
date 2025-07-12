//
// Created by xiang on 2021/7/19.
//

#ifndef SAD_NAV_STATE_H
#define SAD_NAV_STATE_H

#include <sophus/so3.hpp>
#include "common/eigen_types.h"
#include "common/math_utils.h"

namespace zjloc
{

    /**
     * 导航用的状态变量
     * @tparam T    标量类型
     *
     * 这是个封装好的类，部分程序使用本结构体，也有一部分程序使用散装的pvq，都是可以的
     */
    template <typename T>
    struct NavState
    {
        using Vec3 = Eigen::Matrix<T, 3, 1>;
        using SO3 = Sophus::SO3<T>;

        NavState()
        {
            this->R_ = SO3::exp(Vec3::Zero());
            this->p_ = Vec3::Zero();
            this->v_ = Vec3::Zero();
            this->bg_ = Vec3::Zero();
            this->ba_ = Vec3::Zero();
            this->g_ = Vec3::Zero();
            this->cov_ = Eigen::Matrix<T, 18, 18>::Identity() * 0.0001;
        }

        NavState(const NavState &b)
        {
            this->R_ = b.R_;
            this->p_ = b.p_;
            this->v_ = b.v_;
            this->bg_ = b.bg_;
            this->ba_ = b.ba_;
            this->g_ = b.g_;
            this->cov_ = b.cov_;
        }

        NavState &operator=(const NavState &b)
        {
            this->timestamp_ = b.timestamp_;
            this->R_ = b.R_;
            this->p_ = b.p_;
            this->v_ = b.v_;
            this->bg_ = b.bg_;
            this->ba_ = b.ba_;
            this->g_ = b.g_;
            this->cov_ = b.cov_;
            return *this;
        }

        NavState operator+(const Eigen::Matrix<double, 18, 1> &add_)
        {
            NavState a;
            a.R_ = this->R_ * SO3::exp(add_.block<3, 1>(0, 0));
            a.p_ = this->p_ + add_.block<3, 1>(3, 0);
            a.v_ = this->v_ + add_.block<3, 1>(6, 0);
            a.bg_ = this->bg_ + add_.block<3, 1>(9, 0);
            a.ba_ = this->ba_ + add_.block<3, 1>(12, 0);
            a.g_ = this->g_ + add_.block<3, 1>(15, 0);
            a.cov_ = this->cov_;
            return a;
        }

        NavState &operator+=(const Eigen::Matrix<double, 18, 1> &add_)
        {

            this->R_ = this->R_ * SO3::exp(add_.block<3, 1>(0, 0));
            this->p_ = this->p_ + add_.block<3, 1>(3, 0);
            this->v_ = this->v_ + add_.block<3, 1>(6, 0);
            this->bg_ = this->bg_ + add_.block<3, 1>(9, 0);
            this->ba_ = this->ba_ + add_.block<3, 1>(12, 0);
            this->g_ = this->g_ + add_.block<3, 1>(15, 0);
            // a.cov_ = this->cov_;
            return *this;
        }

        Eigen::Matrix<double, 18, 1> operator-(const NavState &b_)
        {
            Eigen::Matrix<double, 18, 1> a;
            a.block<3, 1>(0, 0) = (b_.R_.inverse() * this->R_).log();
            a.block<3, 1>(3, 0) = this->p_ - b_.p_;
            a.block<3, 1>(6, 0) = this->v_ - b_.v_;
            a.block<3, 1>(9, 0) = this->bg_ - b_.bg_;
            a.block<3, 1>(12, 0) = this->ba_ - b_.ba_;
            a.block<3, 1>(15, 0) = this->g_ - b_.g_;

            return a;
        }

        // from time, R, p, v, bg, ba
        explicit NavState(double time, const SO3 &R = SO3(), const Vec3 &t = Vec3::Zero(),
                          const Vec3 &v = Vec3::Zero(), const Vec3 &bg = Vec3::Zero(),
                          const Vec3 &ba = Vec3::Zero(), const Vec3 &g = Vec3::Zero(),
                          const Eigen::Matrix<T, 18, 18> &cov = Eigen::Matrix<T, 18, 18>::Identity())
            : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba), g_(g), cov_(cov)
        {
            // std::cout << __FUNCTION__ << ",t: " << time << std::endl;
        }

        // from pose and vel
        NavState(double time, const SE3 &pose, const Vec3 &vel = Vec3::Zero())
            : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {}

        /// 转换到Sophus
        Sophus::SE3<T> GetSE3() const { return SE3(R_, p_); }

        friend std::ostream &operator<<(std::ostream &os, const NavState<T> &s)
        {
            os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
               << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
               << ", ba: " << s.ba_.transpose();
            return os;
        }

        double timestamp_ = -1;          // 时间
        SO3 R_ = SO3::exp(Vec3::Zero()); // 旋转
        Vec3 p_ = Vec3::Zero();          // 平移
        SO3 R_L_I_ = SO3::exp(Vec3::Zero());
        Vec3 T_L_I_ = Vec3::Zero();
        Vec3 v_ = Vec3::Zero();  // 速度
        Vec3 bg_ = Vec3::Zero(); // gyro 零偏
        Vec3 ba_ = Vec3::Zero(); // acce 零偏
        Vec3 g_ = Vec3::Zero();
        Eigen::Matrix<T, 18, 18> cov_ = Eigen::Matrix<T, 18, 18>::Zero();
    };

    using NavStated = NavState<double>;
    using NavStatef = NavState<float>;

    struct pose6d
    {
        pose6d(double t, Eigen::Vector3d &acc0, Eigen::Vector3d &gyr0,
               Eigen::Vector3d &vel0, Eigen::Vector3d &pos0,
               Eigen::Matrix3d rot0)
            : offset_time(t), acc(acc0), gyr(gyr0), vel(vel0), pos(pos0), rot(rot0) {}

        double offset_time;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyr;
        Eigen::Vector3d vel;
        Eigen::Vector3d pos;
        Eigen::Matrix3d rot;
    };

} // namespace zjloc

#endif
