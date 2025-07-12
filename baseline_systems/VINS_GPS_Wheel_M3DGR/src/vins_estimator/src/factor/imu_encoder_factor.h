#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../parameters.h"
#include "integration_base.h"

#include <ceres/ceres.h>

class IMUEncoderFactor : public ceres::SizedCostFunction<18, 7, 9, 7, 9>
{
  public:
    IMUEncoderFactor() = delete;
    IMUEncoderFactor(IntegrationBase* _pre_integration):pre_integration(_pre_integration)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
            (Bgi - pre_integration->linearized_bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 18, 1>> residual(residuals);
        residual = pre_integration->evaluate(Pi, Qi, Vi, TIO, Quaterniond(RIO), Bai, Bgi,
                                            Pj, Qj, Vj, TIO, Quaterniond(RIO), Baj, Bgj);
        // ROS_INFO_STREAM("Residual of imu encoder factor: " << residual);

        Eigen::Matrix<double, 18, 18> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 18, 18>>(pre_integration->covariance_enc.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = sqrt_info * residual;

        if (jacobians)
        {
            double sum_dt = pre_integration->sum_dt;
            Eigen::Matrix3d dp_dba = pre_integration->jacobian_enc.template block<3, 3>(0, 12);
            Eigen::Matrix3d dp_dbg = pre_integration->jacobian_enc.template block<3, 3>(0, 15);

            Eigen::Matrix3d dq_dbg = pre_integration->jacobian_enc.template block<3, 3>(3, 15);

            Eigen::Matrix3d dv_dba = pre_integration->jacobian_enc.template block<3, 3>(6, 12);
            Eigen::Matrix3d dv_dbg = pre_integration->jacobian_enc.template block<3, 3>(6, 15);

            Eigen::Matrix3d do_dbg = pre_integration->jacobian_enc.template block<3, 3>(9, 15);

            MatrixXd::Index maxRow, maxCol;
            MatrixXd::Index minRow, minCol;
            if (pre_integration->jacobian_enc.maxCoeff(&maxRow, &maxCol) > 1e8 || pre_integration->jacobian_enc.minCoeff(&minRow, &minCol) < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration, max:%f, (%f, %f), min:%f, (%f, %f)", 
                        pre_integration->jacobian_enc.maxCoeff(), pre_integration->jacobian_enc.minCoeff(), maxRow, maxCol, minRow, minCol);
                //std::cout << pre_integration->jacobian << std::endl;
///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(0, 3) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_i.block<3, 3>(3, 3) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_i.block<3, 3>(6, 3) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                    jacobian_pose_i.block<3, 3>(9, 0) = -Qi.inverse().toRotationMatrix();
                    jacobian_pose_i.block<3, 3>(9, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj + Qj * TIO - Pi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff(&maxRow, &maxCol) > 1e8 || jacobian_pose_i.minCoeff(&minRow, &minCol) < -1e8)
                {
                    ROS_WARN("numerical unstable in jacobians, max:%f, (%f, %f), min:%f, (%f, %f)", 
                            jacobian_pose_i.maxCoeff(), jacobian_pose_i.minCoeff(), maxRow, maxCol, minRow, minCol);                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(0, 3) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(0, 6) = -dp_dbg;

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(3, 6) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(6, 0) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(6, 3) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(6, 6) = -dv_dbg;

                    jacobian_speedbias_i.block<3, 3>(9, 6) = -do_dbg;

                jacobian_speedbias_i.block<3, 3>(12, 3) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(15, 6) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(0, 0) = Qi.inverse().toRotationMatrix();

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
                jacobian_pose_j.block<3, 3>(3, 3) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif
                    jacobian_pose_j.block<3, 3>(9, 0) = Qi.inverse().toRotationMatrix();
                    jacobian_pose_j.block<3, 3>(9, 3) = -Qi.inverse().toRotationMatrix() * Qj * Utility::skewSymmetric(TIO);

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 18, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(6, 0) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(12, 3) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

//     virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
//     {

//         Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
//         Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

//         Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
//         Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
//         Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

//         Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
//         Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

//         Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
//         Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
//         Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

// //Eigen::Matrix<double, 15, 15> Fd;
// //Eigen::Matrix<double, 15, 12> Gd;

// //Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
// //Eigen::Quaterniond pQj = Qi * delta_q;
// //Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
// //Eigen::Vector3d pBaj = Bai;
// //Eigen::Vector3d pBgj = Bgi;

// //Vi + Qi * delta_v - g * sum_dt = Vj;
// //Qi * delta_q = Qj;

// //delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
// //delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
// //delta_q = Qi.inverse() * Qj;

// #if 0
//         if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
//             (Bgi - pre_integration->linearized_bg).norm() > 0.01)
//         {
//             pre_integration->repropagate(Bai, Bgi);
//         }
// #endif

//         Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
//         residual = pre_integration->evaluate(Pi, Qi, Vi, TIO, Quaterniond(RIO), Bai, Bgi,
//                                             Pj, Qj, Vj, TIO, Quaterniond(RIO), Baj, Bgj);
//         // residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
//         //                                     Pj, Qj, Vj, Baj, Bgj);
//         ROS_INFO_STREAM("Residual of imu encoder factor: " << residual);

//         Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
//         //sqrt_info.setIdentity();
//         residual = sqrt_info * residual;

//         if (jacobians)
//         {
//             double sum_dt = pre_integration->sum_dt;
//             Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(0, 9);
//             Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(0, 12);

//             Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(3, 12);

//             Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(6, 9);
//             Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(6, 12);

//             // Eigen::Matrix3d do_dbg = pre_integration->jacobian.template block<3, 3>(9, 15);

//             MatrixXd::Index maxRow, maxCol;
//             MatrixXd::Index minRow, minCol;
//             if (pre_integration->jacobian.maxCoeff(&maxRow, &maxCol) > 1e8 || pre_integration->jacobian.minCoeff(&minRow, &minCol) < -1e8)
//             {
//                 ROS_WARN("numerical unstable in preintegration, max:%f, (%f, %f), min:%f, (%f, %f)", 
//                         pre_integration->jacobian.maxCoeff(), pre_integration->jacobian.minCoeff(), maxRow, maxCol, minRow, minCol);
//                 //std::cout << pre_integration->jacobian << std::endl;
// ///                ROS_BREAK();
//             }

//             if (jacobians[0])
//             {
//                 Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
//                 jacobian_pose_i.setZero();

//                 jacobian_pose_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix();
//                 jacobian_pose_i.block<3, 3>(0, 3) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

// #if 0
//             jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
// #else
//                 Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
//                 jacobian_pose_i.block<3, 3>(3, 3) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
// #endif

//                 jacobian_pose_i.block<3, 3>(6, 3) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

//                     // jacobian_pose_i.block<3, 3>(9, 0) = -Qi.inverse().toRotationMatrix();
//                     // jacobian_pose_i.block<3, 3>(9, 3) = Utility::skewSymmetric(Qi.inverse() * (Pj + Qj * TIO - Pi));

//                 jacobian_pose_i = sqrt_info * jacobian_pose_i;

//                 if (jacobian_pose_i.maxCoeff(&maxRow, &maxCol) > 1e8 || jacobian_pose_i.minCoeff(&minRow, &minCol) < -1e8)
//                 {
//                     ROS_WARN("numerical unstable in jacobians, max:%f, (%f, %f), min:%f, (%f, %f)", 
//                             jacobian_pose_i.maxCoeff(), jacobian_pose_i.minCoeff(), maxRow, maxCol, minRow, minCol);                    //std::cout << sqrt_info << std::endl;
//                     //ROS_BREAK();
//                 }
//             }
//             if (jacobians[1])
//             {
//                 Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
//                 jacobian_speedbias_i.setZero();
//                 jacobian_speedbias_i.block<3, 3>(0, 0) = -Qi.inverse().toRotationMatrix() * sum_dt;
//                 jacobian_speedbias_i.block<3, 3>(0, 3) = -dp_dba;
//                 jacobian_speedbias_i.block<3, 3>(0, 6) = -dp_dbg;

// #if 0
//             jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
// #else
//                 //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
//                 //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
//                 jacobian_speedbias_i.block<3, 3>(3, 6) = -Utility::Qleft(Qj.inverse() * Qi * pre_integration->delta_q).bottomRightCorner<3, 3>() * dq_dbg;
// #endif

//                 jacobian_speedbias_i.block<3, 3>(6, 0) = -Qi.inverse().toRotationMatrix();
//                 jacobian_speedbias_i.block<3, 3>(6, 3) = -dv_dba;
//                 jacobian_speedbias_i.block<3, 3>(6, 6) = -dv_dbg;

//                     // jacobian_speedbias_i.block<3, 3>(9, 6) = -do_dbg;

//                 jacobian_speedbias_i.block<3, 3>(9, 3) = -Eigen::Matrix3d::Identity();

//                 jacobian_speedbias_i.block<3, 3>(12, 6) = -Eigen::Matrix3d::Identity();

//                 jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

//                 //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
//                 //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
//             }
//             if (jacobians[2])
//             {
//                 Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
//                 jacobian_pose_j.setZero();

//                 jacobian_pose_j.block<3, 3>(0, 0) = Qi.inverse().toRotationMatrix();

// #if 0
//             jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
// #else
//                 Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
//                 jacobian_pose_j.block<3, 3>(3, 3) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
// #endif
//                     // jacobian_pose_j.block<3, 3>(9, 0) = Qi.inverse().toRotationMatrix();
//                     // jacobian_pose_j.block<3, 3>(9, 3) = -Qi.inverse().toRotationMatrix() * Utility::skewSymmetric(Qj * TIO);

//                 jacobian_pose_j = sqrt_info * jacobian_pose_j;

//                 //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
//                 //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
//             }
//             if (jacobians[3])
//             {
//                 Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
//                 jacobian_speedbias_j.setZero();

//                 jacobian_speedbias_j.block<3, 3>(6, 0) = Qi.inverse().toRotationMatrix();

//                 jacobian_speedbias_j.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();

//                 jacobian_speedbias_j.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity();

//                 jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

//                 //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
//                 //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
//             }
//         }

//         return true;
//     }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    IntegrationBase* pre_integration;

};

